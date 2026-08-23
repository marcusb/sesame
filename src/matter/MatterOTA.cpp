#include <app/clusters/ota-requestor/OTADownloader.h>
#include <app/clusters/ota-requestor/OTARequestorInterface.h>
#include <app/clusters/ota-requestor/DefaultOTARequestor.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorStorage.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorUserConsent.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorDriver.h>
#include <app/clusters/ota-requestor/BDXDownloader.h>
#include <platform/CHIPDeviceLayer.h>
#include <zephyr/sys/reboot.h>

extern "C" {
#include "controller.h"
}
extern "C" {
#include "ota.h"
}

namespace chip {

class OTAImageProcessorImpl : public OTAImageProcessorInterface
{
public:
    void SetOTADownloader(OTADownloader * downloader) { mDownloader = downloader; }

    CHIP_ERROR PrepareDownload() override {
        DeviceLayer::SystemLayer().ScheduleLambda([this] {
            int err = ota_init(&mOtaState);
            if (err == 0) {
                mDownloader->OnPreparedForDownload(CHIP_NO_ERROR);
            } else {
                mDownloader->OnPreparedForDownload(CHIP_ERROR_INTERNAL);
            }
        });
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR Finalize() override {
        DeviceLayer::SystemLayer().ScheduleLambda([this] {
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
        DeviceLayer::SystemLayer().ScheduleLambda([] {
            sys_reboot(SYS_REBOOT_COLD);
        });
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR Abort() override {
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR ProcessBlock(ByteSpan & block) override {
        uint8_t* buf = (uint8_t*)chip::Platform::MemoryAlloc(block.size());
        if (!buf) {
            return CHIP_ERROR_NO_MEMORY;
        }
        memcpy(buf, block.data(), block.size());
        size_t size = block.size();
        
        DeviceLayer::SystemLayer().ScheduleLambda([this, buf, size] {
            int err = ota_write_chunk(&mOtaState, buf, size);
            chip::Platform::MemoryFree(buf);
            if (err == 0) {
                mParams.downloadedBytes += size;
                mDownloader->FetchNextData();
            } else {
                mDownloader->EndDownload(CHIP_ERROR_WRITE_FAILED);
            }
        });
        return CHIP_NO_ERROR;
    }

    bool IsFirstImageRun() override {
        return false;
    }

    CHIP_ERROR ConfirmCurrentImage() override {
        return CHIP_NO_ERROR;
    }

private:
    OTADownloader* mDownloader = nullptr;
    ota_upd_state_t mOtaState;
};

static DefaultOTARequestor gRequestorCore;
static DefaultOTARequestorStorage gRequestorStorage;
static DeviceLayer::DefaultOTARequestorDriver gRequestorUser;
static BDXDownloader gDownloader;
static OTAImageProcessorImpl gImageProcessor;

extern "C" void InitOTARequestor()
{
    SetRequestorInstance(&gRequestorCore);
    gRequestorStorage.Init(chip::Server::GetInstance().GetPersistentStorage());
    gRequestorCore.Init(chip::Server::GetInstance(), gRequestorStorage, gRequestorUser, gDownloader);
    gImageProcessor.SetOTADownloader(&gDownloader);
    gDownloader.SetImageProcessorDelegate(&gImageProcessor);
    gRequestorUser.Init(&gRequestorCore, &gImageProcessor);
}

} // namespace chip
