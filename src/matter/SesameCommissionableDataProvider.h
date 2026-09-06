#pragma once

#include <crypto/CHIPCryptoPAL.h>
#include <lib/support/Span.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/CommissionableDataProvider.h>

class SesameCommissionableDataProvider
    : public chip::DeviceLayer::CommissionableDataProvider {
   public:
    SesameCommissionableDataProvider() {}

    CHIP_ERROR Init() {
        uint8_t mac[6] = {0};
        chip::MutableByteSpan macSpan(mac);
        (void)chip::DeviceLayer::ConfigurationMgr().GetPrimaryMACAddress(
            macSpan);

        // Derive discriminator (12-bit) from MAC
        mDiscriminator = ((mac[4] << 8) | mac[5]) & 0x0FFF;
        if (mDiscriminator == 0) mDiscriminator = 3840;

        // Derive 8-digit PIN from MAC
        uint32_t pin = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];
        if (pin == 0) pin = 20202020;
        pin = (pin % 99999998) + 1;
        // Avoid invalid PINs like 11111111, 12345678
        if (pin % 11111111 == 0 || pin == 12345678 || pin == 87654321) {
            pin ^= 0x55555555;
            pin = (pin % 99999998) + 1;
        }
        mPin = pin;

        // Generate SPAKE2+ verifier
        const char* salt_str = "SesameSalt123456";
        chip::ByteSpan salt(reinterpret_cast<const uint8_t*>(salt_str),
                            strlen(salt_str));

        chip::Crypto::Spake2pVerifier verifier;
        CHIP_ERROR err = verifier.Generate(1000, salt, mPin);
        if (err != CHIP_NO_ERROR) return err;

        chip::MutableByteSpan serializedSpan(mSerializedVerifier);
        return verifier.Serialize(serializedSpan);
    }

    CHIP_ERROR GetSetupDiscriminator(uint16_t& setupDiscriminator) override {
        setupDiscriminator = mDiscriminator;
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR SetSetupDiscriminator(uint16_t setupDiscriminator) override {
        return CHIP_ERROR_NOT_IMPLEMENTED;
    }

    CHIP_ERROR GetSpake2pIterationCount(uint32_t& iterationCount) override {
        iterationCount = 1000;
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR GetSpake2pSalt(chip::MutableByteSpan& saltBuf) override {
        const char* salt_str = "SesameSalt123456";
        if (saltBuf.size() < strlen(salt_str))
            return CHIP_ERROR_BUFFER_TOO_SMALL;
        memcpy(saltBuf.data(), salt_str, strlen(salt_str));
        saltBuf.reduce_size(strlen(salt_str));
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR GetSpake2pVerifier(chip::MutableByteSpan& verifierBuf,
                                  size_t& outVerifierLen) override {
        if (verifierBuf.size() < sizeof(mSerializedVerifier))
            return CHIP_ERROR_BUFFER_TOO_SMALL;
        memcpy(verifierBuf.data(), mSerializedVerifier,
               sizeof(mSerializedVerifier));
        outVerifierLen = sizeof(mSerializedVerifier);
        verifierBuf.reduce_size(outVerifierLen);
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR GetSetupPasscode(uint32_t& setupPasscode) override {
        setupPasscode = mPin;
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR SetSetupPasscode(uint32_t setupPasscode) override {
        return CHIP_ERROR_NOT_IMPLEMENTED;
    }

   private:
    uint16_t mDiscriminator;
    uint32_t mPin;
    uint8_t
        mSerializedVerifier[chip::Crypto::kSpake2p_VerifierSerialized_Length];
};
