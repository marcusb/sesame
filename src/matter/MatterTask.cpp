#include <DeviceInfoProviderImpl.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/clusters/network-commissioning/CodegenInstance.h>
#include <app/clusters/window-covering-server/CodegenIntegration.h>
#include <app/clusters/window-covering-server/WindowCoveringCluster.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app/util/attribute-storage.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <data-model-providers/codegen/Instance.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/NetworkCommissioning.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <zephyr/logging/log.h>

#include "SesameCommissionableDataProvider.h"
#include "controller.h"
#include "matter_endpoints.h"
#include "matter_task.h"

extern "C" {
#include "network.h"
#if defined(CONFIG_SOC_88MW320) && defined(CONFIG_WATCHDOG)
#include "mw_watchdog.h"
#endif
}

LOG_MODULE_REGISTER(matter_task, LOG_LEVEL_INF);

namespace {

class SesameEthernetDriver final
    : public chip::DeviceLayer::NetworkCommissioning::EthernetDriver {
   public:
    class EthernetNetworkIterator final
        : public chip::DeviceLayer::NetworkCommissioning::NetworkIterator {
       public:
        EthernetNetworkIterator(SesameEthernetDriver* aDriver)
            : mDriver(aDriver) {}
        size_t Count() override { return 1; }
        bool Next(
            chip::DeviceLayer::NetworkCommissioning::Network& item) override {
            if (mExhausted) {
                return false;
            }
            mExhausted = true;
            static const char kIfaceName[] = "wlan0";
            memcpy(item.networkID, kIfaceName, sizeof(kIfaceName) - 1);
            item.networkIDLen = sizeof(kIfaceName) - 1;
            item.connected = true;
            return true;
        }
        void Release() override { delete this; }
        ~EthernetNetworkIterator() override = default;

       private:
        SesameEthernetDriver* mDriver;
        bool mExhausted = false;
    };

    uint8_t GetMaxNetworks() override { return 1; }
    chip::DeviceLayer::NetworkCommissioning::NetworkIterator* GetNetworks()
        override {
        return new EthernetNetworkIterator(this);
    }
    CHIP_ERROR Init(
        chip::DeviceLayer::NetworkCommissioning::Internal::BaseDriver::
            NetworkStatusChangeCallback* networkStatusChangeCallback) override {
        return CHIP_NO_ERROR;
    }
    void Shutdown() override {}

    static SesameEthernetDriver& Instance() {
        static SesameEthernetDriver instance;
        return instance;
    }
};

chip::app::Clusters::NetworkCommissioning::Instance
    sNetworkCommissioningInstance(0, &SesameEthernetDriver::Instance());

}  // namespace

static struct k_work_delayable sMatterCommWork;

static void MatterCommissioningWorkHandler(struct k_work* work) {
    // Wait for network to be up (including IPv6 for Matter)
    if (!network_is_up() || !network_has_ipv6()) {
#if defined(CONFIG_SOC_88MW320) && defined(CONFIG_WATCHDOG)
        feed_watchdog();
#endif
        k_work_schedule(&sMatterCommWork, K_MSEC(500));
        return;
    }
    // Wait for IPv4 DHCP if available (up to 10 seconds)
    static int ipv4_wait_count = 0;
    if (!network_has_ipv4() && ipv4_wait_count++ < 20) {
#if defined(CONFIG_SOC_88MW320) && defined(CONFIG_WATCHDOG)
        feed_watchdog();
#endif
        k_work_schedule(&sMatterCommWork, K_MSEC(500));
        return;
    }

    LOG_INF("Network is UP.");
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
        LOG_INF(
            "Device not commissioned. Opening basic commissioning window...");
        (void)chip::DeviceLayer::PlatformMgr().ScheduleWork(
            [](intptr_t) {
                CHIP_ERROR err =
                    chip::Server::GetInstance()
                        .GetCommissioningWindowManager()
                        .OpenBasicCommissioningWindow(
                            chip::System::Clock::Seconds16(300),
                            chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err == CHIP_NO_ERROR) {
                    LOG_INF("Commissioning window opened successfully");
                } else {
                    LOG_ERR("Failed to open commissioning window: %d",
                            (int)err.AsInteger());
                }
            },
            0);
    } else {
        LOG_INF("Device already commissioned (fabrics: %u)",
                (unsigned)chip::Server::GetInstance()
                    .GetFabricTable()
                    .FabricCount());
    }
}

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

    CHIP_ERROR netErr = sNetworkCommissioningInstance.Init();
    if (netErr != CHIP_NO_ERROR) {
        LOG_ERR("Failed to init NetworkCommissioning: %d",
                (int)netErr.AsInteger());
    } else {
        LOG_INF("NetworkCommissioning cluster initialized");
    }

    InitOTARequestor();

    // Print setup info
    PrintOnboardingCodes(chip::RendezvousInformationFlag(
        chip::RendezvousInformationFlag::kOnNetwork));

    (void)chip::DeviceLayer::PlatformMgr().StartEventLoopTask();

    k_work_init_delayable(&sMatterCommWork, MatterCommissioningWorkHandler);
    k_work_schedule(&sMatterCommWork, K_NO_WAIT);
}

using namespace ::chip;
using namespace ::chip::app::Clusters::WindowCovering;

class SesameWindowCoveringDelegate
    : public chip::app::Clusters::WindowCovering::WindowCoveringDelegate {
   public:
    SesameWindowCoveringDelegate() { mEndpoint = 1; }

    CHIP_ERROR HandleMovement(
        chip::app::Clusters::WindowCovering::WindowCoveringType type) override {
        if (type ==
            chip::app::Clusters::WindowCovering::WindowCoveringType::Lift) {
            ctrl_msg_t msg = {};
            msg.type = CTRL_MSG_DOOR_CONTROL;

            auto wc =
                chip::app::Clusters::WindowCovering::FindClusterOnEndpoint(1);
            if (wc) {
                auto target = wc->GetTargetPositionLiftPercent100ths();
                if (!target.IsNull()) {
                    // Target is percentage closed: 0 = fully open, 10000 =
                    // fully closed
                    if (target.Value() < 5000) {
                        msg.msg.door_control.command = DOOR_CMD_OPEN;
                        LOG_INF("Matter: Target=Open (%u)",
                                (unsigned)target.Value());
                    } else {
                        msg.msg.door_control.command = DOOR_CMD_CLOSE;
                        LOG_INF("Matter: Target=Close (%u)",
                                (unsigned)target.Value());
                    }
                    k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
                    return CHIP_NO_ERROR;
                }
            }

            auto op = chip::app::Clusters::WindowCovering::OperationalStateGet(
                1,
                chip::app::Clusters::WindowCovering::OperationalStatus::kLift);
            if (op == chip::app::Clusters::WindowCovering::OperationalState::
                          MovingUpOrOpen) {
                msg.msg.door_control.command = DOOR_CMD_OPEN;
                LOG_INF("Matter: Target=Open");
            } else if (op == chip::app::Clusters::WindowCovering::
                                 OperationalState::MovingDownOrClose) {
                msg.msg.door_control.command = DOOR_CMD_CLOSE;
                LOG_INF("Matter: Target=Close");
            } else {
                LOG_WRN(
                    "Matter: HandleMovement with unexpected operational state "
                    "%d",
                    (int)op);
                return CHIP_NO_ERROR;
            }
            k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        }
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR HandleStopMotion() override {
        ctrl_msg_t msg = {};
        msg.type = CTRL_MSG_DOOR_CONTROL;
        msg.msg.door_control.command = DOOR_CMD_STOP;
        LOG_INF("Matter: Target=Stop");
        k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        return CHIP_NO_ERROR;
    }
};

static SesameWindowCoveringDelegate sWindowCoveringDelegate;

void MatterPostAttributeChangeCallback(
    const app::ConcreteAttributePath& attributePath, uint8_t mask, uint8_t type,
    uint16_t size, uint8_t* value) {
    // Implementation not needed for basic operation, just logging
}

void MatterWindowCoveringClusterServerAttributeChangedCallback(
    const app::ConcreteAttributePath& attributePath) {}

void emberAfWindowCoveringClusterInitCallback(chip::EndpointId endpoint) {
    if (endpoint == 1) {
        chip::app::Clusters::WindowCovering::SetDefaultDelegate(
            1, &sWindowCoveringDelegate);
    }
}

void matter_update_door_state(const door_state_msg_t* msg) {
    // msg->pos is percentage open (0% = closed, 100% = open).
    // Matter CurrentPositionLiftPercent100ths is percentage closed (0 = open,
    // 10000 = closed).
    uint16_t closed_percent100ths = (100 - msg->pos) * 100;
    OperationalState opState = OperationalState::Stall;
    if (msg->direction == DCM_DOOR_DIR_UP) {
        opState = OperationalState::MovingUpOrOpen;
    } else if (msg->direction == DCM_DOOR_DIR_DOWN) {
        opState = OperationalState::MovingDownOrClose;
    } else {
        opState = OperationalState::Stall;
    }

    uint32_t packedArg =
        (static_cast<uint32_t>(opState) << 16) | closed_percent100ths;

    (void)chip::DeviceLayer::PlatformMgr().ScheduleWork(
        [](intptr_t arg) {
            OperationalState state =
                static_cast<OperationalState>((arg >> 16) & 0xFF);
            uint16_t posVal = static_cast<uint16_t>(arg & 0xFFFF);

            OperationalStateSet(1, OperationalStatus::kLift, state);

            auto wc =
                chip::app::Clusters::WindowCovering::FindClusterOnEndpoint(1);
            if (wc) {
                chip::app::DataModel::Nullable<chip::Percent100ths> p;
                p.SetNonNull(posVal);
                wc->SetCurrentPositionLiftPercent100ths(p);

                if (state == OperationalState::Stall) {
                    wc->SetTargetPositionLiftPercent100ths(p);
                }
            }
        },
        static_cast<intptr_t>(packedArg));
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
