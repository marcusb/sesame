/*
 * matter_app.cpp — CHIP stack entry point and public API for Sesame.
 *
 * Called from board_main.c via
 * matter_app_start() after Wi-Fi is up (network_manager has reached
 * kNetworkReady state).
 *
 * QEMU builds run the same CHIP stack but with chip_platform_qemu providing
 * the platform layer (no OTA flash drivers, stub WiFi).
 */

/* picolibc bare-metal doesn't provide a FILE* object for stderr.  Define a
 * weak NULL here so libstdc++ internals (e.g. vterminate) can reference it
 * without a link error.  Placed in this TU (not sesame_matter_stubs.c) so the
 * definition is guaranteed to be extracted from the archive. */
extern "C" {
#include <stdio.h>
#undef stderr
extern __attribute__((weak)) FILE * const stderr = nullptr;
}

#include "matter_app.h"
#include "matter_task.h"

#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Dnssd.h>
#include <app/server/Server.h>
#include <app/util/attribute-storage.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <data-model-providers/codegen/Instance.h>
#include <lib/support/CHIPMem.h>
#include <lib/support/logging/CHIPLogging.h>
#include <platform/CHIPDeviceLayer.h>
#include <setup_payload/OnboardingCodesUtil.h>

#include "app_logging.h"
#include "controller.h"

using namespace chip;
using namespace chip::app;
using namespace chip::DeviceLayer;

static constexpr EndpointId kWindowCoveringEndpoint = 1;

class SesameWindowCoveringDelegate : public Clusters::WindowCovering::Delegate
{
public:
    CHIP_ERROR HandleMovement(Clusters::WindowCovering::WindowCoveringType type) override
    {
        if (type != Clusters::WindowCovering::WindowCoveringType::Lift)
            return CHIP_NO_ERROR;

        Clusters::WindowCovering::NPercent100ths target;
        Clusters::WindowCovering::Attributes::TargetPositionLiftPercent100ths::
            Get(mEndpoint, target);

        ctrl_msg_t msg;
        msg.type = CTRL_MSG_DOOR_CONTROL;
        if (!target.IsNull() && target.Value() < 5000)
            msg.msg.door_control.command = DOOR_CMD_OPEN;
        else
            msg.msg.door_control.command = DOOR_CMD_CLOSE;

        xQueueSend(ctrl_queue, &msg, pdMS_TO_TICKS(500));
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR HandleStopMotion() override
    {
        ctrl_msg_t msg = {CTRL_MSG_DOOR_CONTROL, {DOOR_CMD_STOP}};
        xQueueSend(ctrl_queue, &msg, pdMS_TO_TICKS(500));
        return CHIP_NO_ERROR;
    }
};

static SesameWindowCoveringDelegate g_window_covering_delegate;

static bool s_started = false;

static void matter_app_task(void * /*param*/)
{
    ChipLogProgress(DeviceLayer, "CHIP stack init");

    CHIP_ERROR err = chip::Platform::MemoryInit();
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "MemoryInit failed: %" CHIP_ERROR_FORMAT, err.Format());
        vTaskDelete(nullptr);
        return;
    }

    err = PlatformMgr().InitChipStack();
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "InitChipStack failed: %" CHIP_ERROR_FORMAT, err.Format());
        vTaskDelete(nullptr);
        return;
    }

    chip::Credentials::SetDeviceAttestationCredentialsProvider(
        chip::Credentials::Examples::GetExampleDACProvider());

    static chip::CommonCaseDeviceServerInitParams init_params;
    (void) init_params.InitializeStaticResourcesBeforeServerInit();
    init_params.dataModelProvider =
        CodegenDataModelProviderInstance(init_params.persistentStorageDelegate);

    err = chip::Server::GetInstance().Init(init_params);
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "Server::Init failed: %" CHIP_ERROR_FORMAT, err.Format());
        vTaskDelete(nullptr);
        return;
    }

    PrintOnboardingCodes(chip::RendezvousInformationFlag::kOnNetwork);

    Clusters::WindowCovering::SetDefaultDelegate(kWindowCoveringEndpoint, &g_window_covering_delegate);

    PlatformMgr().RunEventLoop();
}

void matter_app_start(void)
{
    if (s_started)
        return;
    s_started = true;
    xTaskCreate(matter_app_task, "CHIP", 24 * 1024 / sizeof(StackType_t),
                nullptr, tskIDLE_PRIORITY + 2, nullptr);
}

void matter_init(void)
{
    matter_app_start();
}

void matter_schedule_network_up(void)
{
    if (!s_started)
        return;
    PlatformMgr().ScheduleWork(
        [](intptr_t) { chip::app::DnssdServer::Instance().StartServer(); }, 0);
}

void matter_report_door_state(const door_state_msg_t * msg)
{
    if (!msg || !s_started)
        return;

    auto to_percent100ths = [](door_open_state_t s) -> Clusters::WindowCovering::NPercent100ths {
        Clusters::WindowCovering::NPercent100ths val;
        switch (s)
        {
        case DCM_DOOR_STATE_CLOSED:
            val.SetNonNull(10000);
            break;
        case DCM_DOOR_STATE_OPEN:
            val.SetNonNull(0);
            break;
        default:
            val.SetNull();
            break;
        }
        return val;
    };

    auto to_op_state = [](door_direction_t d) -> Clusters::WindowCovering::OperationalState {
        switch (d)
        {
        case DCM_DOOR_DIR_UP:
            return Clusters::WindowCovering::OperationalState::MovingUpOrOpen;
        case DCM_DOOR_DIR_DOWN:
            return Clusters::WindowCovering::OperationalState::MovingDownOrClose;
        default:
            return Clusters::WindowCovering::OperationalState::Stall;
        }
    };

    Clusters::WindowCovering::LiftPositionSet(kWindowCoveringEndpoint, to_percent100ths(msg->state));
    Clusters::WindowCovering::OperationalStateSet(kWindowCoveringEndpoint,
        chip::BitMask<Clusters::WindowCovering::OperationalStatus>(
            Clusters::WindowCovering::OperationalStatus::kLift),
        to_op_state(msg->direction));
}

bool matter_commission_open(uint32_t timeout_s)
{
    if (timeout_s == 0 || timeout_s > 900)
        timeout_s = 900;

    CHIP_ERROR err =
        chip::Server::GetInstance().GetCommissioningWindowManager().OpenBasicCommissioningWindow(
            chip::System::Clock::Seconds32(static_cast<uint32_t>(timeout_s)));
    if (err != CHIP_NO_ERROR)
    {
        LogError(("[matter] commission_open failed: %" CHIP_ERROR_FORMAT, err.Format()));
        return false;
    }
    LogInfo(("[matter] commissioning window opened for %u s", (unsigned) timeout_s));
    return true;
}

void matter_wipe_fabrics(void)
{
    chip::Server::GetInstance().ScheduleFactoryReset();
    LogInfo(("[matter] factory reset scheduled"));
}
