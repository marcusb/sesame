#include <DeviceInfoProviderImpl.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/clusters/window-covering-server/CodegenIntegration.h>
#include <app/clusters/window-covering-server/WindowCoveringCluster.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app/util/attribute-storage.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <data-model-providers/codegen/Instance.h>
#include <platform/CHIPDeviceLayer.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <zephyr/logging/log.h>

#include "SesameCommissionableDataProvider.h"
#include "controller.h"
#include "matter_endpoints.h"
#include "matter_task.h"

extern "C" {
#include "network.h"
}

LOG_MODULE_REGISTER(matter_task, LOG_LEVEL_INF);

extern "C" void matter_task_start(void) {
    LOG_INF("Initializing CHIP Stack");
    (void)chip::DeviceLayer::PlatformMgr().InitChipStack();
    static chip::DeviceLayer::DeviceInfoProviderImpl gExampleDeviceInfoProvider;
    static SesameCommissionableDataProvider sProvider;
    if (sProvider.Init() == CHIP_NO_ERROR) {
        chip::DeviceLayer::SetCommissionableDataProvider(&sProvider);
    }
    chip::Credentials::SetDeviceAttestationCredentialsProvider(
        chip::Credentials::Examples::GetExampleDACProvider());

    // Initialize the ZCL server
    LOG_INF("Initializing ZCL Server");
    static chip::CommonCaseDeviceServerInitParams initParams;
    (void)initParams.InitializeStaticResourcesBeforeServerInit();
    initParams.dataModelProvider = chip::app::CodegenDataModelProviderInstance(
        initParams.persistentStorageDelegate);

    gExampleDeviceInfoProvider.SetStorageDelegate(
        initParams.persistentStorageDelegate);
    chip::DeviceLayer::SetDeviceInfoProvider(&gExampleDeviceInfoProvider);

    (void)chip::Server::GetInstance().Init(initParams);

    InitOTARequestor();

    // Print setup info
    PrintOnboardingCodes(chip::RendezvousInformationFlag(
        chip::RendezvousInformationFlag::kOnNetwork));

    (void)chip::DeviceLayer::PlatformMgr().StartEventLoopTask();

    // Start a polling timer to open the commissioning window after network is
    // up
    static chip::System::TimerCompleteCallback sCommissioningPollTimer;
    sCommissioningPollTimer = [](chip::System::Layer*, void*) {
        if (network_is_up()) {
            LOG_INF("Network is UP. Opening commissioning window...");
            (void)chip::DeviceLayer::PlatformMgr().ScheduleWork(
                [](intptr_t) {
                    chip::Server::GetInstance()
                        .GetCommissioningWindowManager()
                        .OpenBasicCommissioningWindow(
                            chip::System::Clock::Seconds16(300),
                            chip::CommissioningWindowAdvertisement::kDnssdOnly);
                },
                0);
        } else {
            // Check again in 1 second
            chip::DeviceLayer::SystemLayer().StartTimer(
                chip::System::Clock::Seconds32(1), sCommissioningPollTimer,
                nullptr);
        }
    };
    chip::DeviceLayer::SystemLayer().StartTimer(
        chip::System::Clock::Seconds32(1), sCommissioningPollTimer, nullptr);
}

using namespace ::chip;
using namespace ::chip::app::Clusters::WindowCovering;

void MatterPostAttributeChangeCallback(
    const app::ConcreteAttributePath& attributePath, uint8_t mask, uint8_t type,
    uint16_t size, uint8_t* value) {
    // Implementation not needed for basic operation, just logging
}

void MatterWindowCoveringClusterServerAttributeChangedCallback(
    const app::ConcreteAttributePath& attributePath) {
    if (attributePath.mEndpointId == 1)  // Assuming Endpoint 1
    {
        if (attributePath.mAttributeId ==
            Attributes::TargetPositionLiftPercent100ths::Id) {
            app::DataModel::Nullable<chip::Percent100ths> targetPosition;
            auto wc =
                chip::app::Clusters::WindowCovering::FindClusterOnEndpoint(
                    attributePath.mEndpointId);
            if (wc) {
                targetPosition = wc->GetTargetPositionLiftPercent100ths();
            }
            if (!targetPosition.IsNull()) {
                ctrl_msg_t msg = {};
                msg.type = CTRL_MSG_DOOR_CONTROL;

                if (targetPosition.Value() == 0) {
                    msg.msg.door_control.command = DOOR_CMD_OPEN;
                    LOG_INF("Matter: Target=Open");
                } else {
                    msg.msg.door_control.command = DOOR_CMD_CLOSE;
                    LOG_INF("Matter: Target=Close");
                }

                // Enqueue to the main control queue (no block)
                k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
            }
        }
    }
}

void emberAfWindowCoveringClusterInitCallback(chip::EndpointId endpoint) {
    // Initialize attributes if needed
}

void matter_update_door_state(const door_state_msg_t* msg) {
    chip::app::DataModel::Nullable<chip::Percent100ths> pos;
    if (msg->state == DCM_DOOR_STATE_CLOSED) {
        pos.SetNonNull(10000);  // 100.00% closed
    } else if (msg->state == DCM_DOOR_STATE_OPEN) {
        pos.SetNonNull(0);  // 0.00% closed
    } else {
        pos.SetNonNull(5000);  // Partially closed
    }

    (void)chip::DeviceLayer::PlatformMgr().ScheduleWork(
        [](intptr_t arg) {
            auto wc =
                chip::app::Clusters::WindowCovering::FindClusterOnEndpoint(1);
            if (wc) {
                chip::app::DataModel::Nullable<chip::Percent100ths> p;
                p.SetNonNull((uint16_t)arg);
                wc->SetCurrentPositionLiftPercent100ths(p);
            }
        },
        pos.Value());
}

void matter_wipe_fabrics(void) {
    LOG_INF("Scheduling Matter factory reset!");
    chip::Server::GetInstance().ScheduleFactoryReset();
}

bool matter_commission_open(uint32_t timeout_s) {
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
        LOG_WRN("Cannot open basic commissioning on a commissioned device");
        return false;
    }
    CHIP_ERROR err =
        chip::Server::GetInstance()
            .GetCommissioningWindowManager()
            .OpenBasicCommissioningWindow(
                chip::System::Clock::Seconds16(timeout_s),
                chip::CommissioningWindowAdvertisement::kDnssdOnly);
    return err == CHIP_NO_ERROR;
}
