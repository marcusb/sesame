#include "matter_task.h"
#include <stdio.h>
#include <inttypes.h>
#include <app/server/Server.h>
#include <credentials/FabricTable.h>

extern "C" void matter_get_fabric_info_json(char *buf, size_t max_len) {
    if (!buf || max_len == 0) return;
    
    buf[0] = '[';
    size_t offset = 1;
    bool first = true;

    chip::FabricTable & fabricTable = chip::Server::GetInstance().GetFabricTable();
    for (auto it = fabricTable.cbegin(); it != fabricTable.cend(); ++it) {
        const chip::FabricInfo & fabricInfo = *it;
        if (!first) {
            if (offset + 2 < max_len) {
                buf[offset++] = ',';
                buf[offset++] = ' ';
            }
        }
        first = false;
        
        chip::NodeId nodeId = fabricInfo.GetNodeId();
        chip::FabricId fabricId = fabricInfo.GetFabricId();
        uint16_t vendorId = fabricInfo.GetVendorId();
        
        char labelBuf[33] = {0};
        auto labelSpan = fabricInfo.GetFabricLabel();
        if (labelSpan.size() > 0 && labelSpan.size() < sizeof(labelBuf)) {
            snprintf(labelBuf, sizeof(labelBuf), "%.*s", 
                static_cast<int>(labelSpan.size()),
                labelSpan.data());
        }

        offset += snprintf(buf + offset, max_len - offset,
            "{\"node_id\": \"%016" PRIX64 "\", \"fabric_id\": \"%016" PRIX64 "\", \"vendor_id\": %u, \"label\": \"%s\"}",
            nodeId, fabricId, vendorId, labelBuf);
    }
    
    snprintf(buf + offset, max_len - offset, "]");
}
