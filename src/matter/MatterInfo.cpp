#include <app/server/Server.h>
#include <credentials/FabricTable.h>
#include <inttypes.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>
#include <stdio.h>

#include "matter_task.h"

extern "C" void matter_get_fabric_info_json(char* buf, size_t max_len) {
    if (!buf || max_len == 0) {
        return;
    }

    size_t offset = 0;
    offset += snprintf(buf + offset, max_len - offset, "{\"fabrics\": [");

    bool first = true;
    size_t fabric_count = 0;

    chip::FabricTable& fabricTable =
        chip::Server::GetInstance().GetFabricTable();
    for (auto it = fabricTable.cbegin(); it != fabricTable.cend(); ++it) {
        const chip::FabricInfo& fabricInfo = *it;
        if (!first) {
            if (offset + 2 < max_len) {
                buf[offset++] = ',';
                buf[offset++] = ' ';
                buf[offset] = '\0';
            }
        }
        first = false;
        fabric_count++;

        chip::NodeId nodeId = fabricInfo.GetNodeId();
        chip::FabricId fabricId = fabricInfo.GetFabricId();
        uint16_t vendorId = fabricInfo.GetVendorId();

        char labelBuf[33] = {0};
        auto labelSpan = fabricInfo.GetFabricLabel();
        if (labelSpan.size() > 0 && labelSpan.size() < sizeof(labelBuf)) {
            snprintf(labelBuf, sizeof(labelBuf), "%.*s",
                     static_cast<int>(labelSpan.size()), labelSpan.data());
        }

        if (offset < max_len) {
            offset += snprintf(buf + offset, max_len - offset,
                               "{\"node_id\": \"%016" PRIX64
                               "\", \"fabric_id\": \"%016" PRIX64
                               "\", \"vendor_id\": %u, \"label\": \"%s\"}",
                               nodeId, fabricId, vendorId, labelBuf);
        }
    }

    if (offset < max_len) {
        offset += snprintf(buf + offset, max_len - offset, "]");
    }

    if (fabric_count == 0) {
        bool commissioningOpen = chip::Server::GetInstance()
                                     .GetCommissioningWindowManager()
                                     .IsCommissioningWindowOpen();

        char qrCodeBuf[chip::QRCodeBasicSetupPayloadGenerator::
                           kMaxQRCodeBase38RepresentationLength +
                       1] = {0};
        chip::MutableCharSpan qrCode(qrCodeBuf, sizeof(qrCodeBuf) - 1);
        if (GetQRCode(qrCode,
                      chip::RendezvousInformationFlags(
                          chip::RendezvousInformationFlag::kOnNetwork)) ==
            CHIP_NO_ERROR) {
            qrCodeBuf[qrCode.size()] = '\0';
        }

        char manualPairingCodeBuf[64] = {0};
        chip::MutableCharSpan manualPairingCode(
            manualPairingCodeBuf, sizeof(manualPairingCodeBuf) - 1);
        if (GetManualPairingCode(
                manualPairingCode,
                chip::RendezvousInformationFlags(
                    chip::RendezvousInformationFlag::kOnNetwork)) ==
            CHIP_NO_ERROR) {
            manualPairingCodeBuf[manualPairingCode.size()] = '\0';
        }

        if (offset < max_len) {
            offset += snprintf(
                buf + offset, max_len - offset,
                ", \"setup\": {\"commissioning_open\": %s, "
                "\"manual_pairing_code\": \"%s\", \"qr_code\": \"%s\"}",
                commissioningOpen ? "true" : "false", manualPairingCodeBuf,
                qrCodeBuf);
        }
    }

    if (offset < max_len) {
        snprintf(buf + offset, max_len - offset, "}");
    }
}
