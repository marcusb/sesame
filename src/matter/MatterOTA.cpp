#include <app/clusters/ota-requestor/BDXDownloader.h>
#include <app/clusters/ota-requestor/CodegenIntegration.h>
#include <app/clusters/ota-requestor/DefaultOTARequestor.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorDriver.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorStorage.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorUserConsent.h>
#include <app/clusters/ota-requestor/OTADownloader.h>
#include <app/clusters/ota-requestor/OTARequestorInterface.h>
#include <crypto/CHIPCryptoPAL.h>
#include <lib/core/OTAImageHeader.h>
#include <platform/CHIPDeviceLayer.h>
#include <zephyr/sys/reboot.h>

#include "controller.h"
#include "matter_endpoints.h"
#include "ota.h"

#ifdef CONFIG_BOOTLOADER_MCUBOOT
namespace chip {

using Crypto::kSHA256_Hash_Length;

// BDX delivers the full Matter OTA image: a fixed + TLV header followed by the
// firmware payload. Only the payload is written to the MCUboot slot; the header
// is stripped here and the payload is integrity-checked against the header
// digest before the image is committed.
class OTAImageProcessorImpl : public OTAImageProcessorInterface {
   public:
    void SetOTADownloader(OTADownloader* downloader) {
        mDownloader = downloader;
    }

    CHIP_ERROR PrepareDownload() override {
        DeviceLayer::SystemLayer().ScheduleLambda([this] {
            int err = ota_init(&mOtaState);
            if (err == 0) {
                mHeaderParser.Init();
                (void)mHash.Begin();
                mParams.downloadedBytes = 0;
                mParams.totalFileBytes = 0;
                mDigestValid = false;
                (void)mDownloader->OnPreparedForDownload(CHIP_NO_ERROR);
            } else {
                (void)mDownloader->OnPreparedForDownload(CHIP_ERROR_INTERNAL);
            }
        });
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR Finalize() override {
        DeviceLayer::SystemLayer().ScheduleLambda([this] {
            if (mDigestValid) {
                uint8_t digest[kSHA256_Hash_Length];
                MutableByteSpan out(digest, sizeof(digest));
                CHIP_ERROR herr = mHash.Finish(out);
                if (herr != CHIP_NO_ERROR ||
                    memcmp(digest, mExpectedDigest, kSHA256_Hash_Length) != 0) {
                    ChipLogError(
                        SoftwareUpdate,
                        "OTA payload digest mismatch, discarding image");
                    return;
                }
            }
            int err = ota_finish(&mOtaState);
            if (err == 0) {
                mParams.downloadedBytes = 0;
            } else {
                ChipLogError(SoftwareUpdate, "OTA Finalize failed");
            }
        });
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR Apply() override {
        DeviceLayer::SystemLayer().ScheduleLambda(
            [] { sys_reboot(SYS_REBOOT_COLD); });
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR Abort() override { return CHIP_NO_ERROR; }

    CHIP_ERROR ProcessBlock(ByteSpan& block) override {
        uint8_t* buf = (uint8_t*)chip::Platform::MemoryAlloc(block.size());
        if (!buf) {
            return CHIP_ERROR_NO_MEMORY;
        }
        memcpy(buf, block.data(), block.size());
        size_t size = block.size();

        DeviceLayer::SystemLayer().ScheduleLambda([this, buf, size] {
            ByteSpan payload(buf, size);

            CHIP_ERROR err = ProcessHeader(payload);
            if (err != CHIP_NO_ERROR) {
                chip::Platform::MemoryFree(buf);
                mDownloader->EndDownload(err);
                return;
            }

            if (!payload.empty()) {
                (void)mHash.AddData(payload);
                int werr =
                    ota_write_chunk(&mOtaState, payload.data(), payload.size());
                if (werr != 0) {
                    chip::Platform::MemoryFree(buf);
                    mDownloader->EndDownload(CHIP_ERROR_WRITE_FAILED);
                    return;
                }
                mParams.downloadedBytes += payload.size();
            }

            chip::Platform::MemoryFree(buf);
            (void)mDownloader->FetchNextData();
        });
        return CHIP_NO_ERROR;
    }

    bool IsFirstImageRun() override { return false; }

    CHIP_ERROR ConfirmCurrentImage() override { return CHIP_NO_ERROR; }

   private:
    // Feeds the header parser. On success `block` is advanced past the header
    // (to the payload tail, which may be empty) and the expected digest is
    // captured. Returns CHIP_NO_ERROR once the payload tail is ready, or the
    // header parse error otherwise.
    CHIP_ERROR ProcessHeader(ByteSpan& block) {
        if (!mHeaderParser.IsInitialized()) {
            return CHIP_NO_ERROR;
        }
        OTAImageHeader header;
        CHIP_ERROR err = mHeaderParser.AccumulateAndDecode(block, header);
        if (err == CHIP_ERROR_BUFFER_TOO_SMALL) {
            return CHIP_NO_ERROR;
        }
        if (err != CHIP_NO_ERROR) {
            mHeaderParser.Clear();
            return err;
        }
        mParams.totalFileBytes = header.mPayloadSize;
        if (header.mImageDigestType == OTAImageDigestType::kSha256 &&
            header.mImageDigest.size() == kSHA256_Hash_Length) {
            memcpy(mExpectedDigest, header.mImageDigest.data(),
                   kSHA256_Hash_Length);
            mDigestValid = true;
        }
        mHeaderParser.Clear();
        return CHIP_NO_ERROR;
    }

    OTADownloader* mDownloader = nullptr;
    ota_upd_state_t mOtaState;
    OTAImageHeaderParser mHeaderParser;
    Crypto::Hash_SHA256_stream mHash;
    uint8_t mExpectedDigest[Crypto::kSHA256_Hash_Length] = {0};
    bool mDigestValid = false;
};

static DefaultOTARequestor gRequestorCore;
static DefaultOTARequestorStorage gRequestorStorage;
static DeviceLayer::DefaultOTARequestorDriver gRequestorUser;
static BDXDownloader gDownloader;
static OTAImageProcessorImpl gImageProcessor;

}  // namespace chip
#endif

#ifdef CONFIG_BOOTLOADER_MCUBOOT
void InitOTARequestor() {
    chip::SetRequestorInstance(&chip::gRequestorCore);
    chip::gRequestorStorage.Init(
        chip::Server::GetInstance().GetPersistentStorage());
    (void)chip::gRequestorCore.Init(
        chip::Server::GetInstance(), chip::gRequestorStorage,
        chip::gRequestorUser, chip::gDownloader,
        chip::GetOTARequestorAttributes(),
        chip::GetDefaultOTARequestorEventGenerator());
    chip::gImageProcessor.SetOTADownloader(&chip::gDownloader);
    chip::gDownloader.SetImageProcessorDelegate(&chip::gImageProcessor);
    chip::gRequestorUser.Init(&chip::gRequestorCore, &chip::gImageProcessor);
}
#else
void InitOTARequestor() {
    // OTA Mocked
}
#endif
