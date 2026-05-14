/*
 *    Copyright (c) 2020 Project CHIP Authors
 *    Copyright (c) 2018 Nest Labs, Inc.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

/*
 * Sesame shadow override of GenericPlatformManagerImpl_FreeRTOS.ipp.
 *
 * Changes from upstream (pinned at 7521861ff3):
 *   - _RunEventLoop: replaced LayerImplFreeRTOS::HandlePlatformTimer() with a
 *     no-op for CHIP_SYSTEM_CONFIG_USE_SOCKETS=1.  In sockets mode,
 *     LayerImplSelect::StartTimer() never calls PlatformEventing::StartTimer(),
 *     so _StartChipTimer() is never invoked and mChipTimerActive is always
 *     false; the timer-dispatch branch is unreachable dead code.  The cast to
 *     LayerImplFreeRTOS is replaced to avoid a compile error on the undefined
 *     type.
 */

#ifndef GENERIC_PLATFORM_MANAGER_IMPL_FREERTOS_CPP
#define GENERIC_PLATFORM_MANAGER_IMPL_FREERTOS_CPP

#include <platform/PlatformManager.h>
#include <platform/internal/CHIPDeviceLayerInternal.h>
#include <platform/internal/GenericPlatformManagerImpl_FreeRTOS.h>

#include <lib/support/CodeUtils.h>

#include <platform/internal/GenericPlatformManagerImpl.ipp>

namespace chip {
namespace DeviceLayer {
namespace Internal {

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_InitChipStack(void)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    vTaskSetTimeOutState(&mNextTimerBaseTime);
    mNextTimerDurationTicks = 0;
    mChipTimerActive        = false;

    if (mChipStackLock == NULL)
    {
#if defined(CHIP_CONFIG_FREERTOS_USE_STATIC_SEMAPHORE) && CHIP_CONFIG_FREERTOS_USE_STATIC_SEMAPHORE
        mChipStackLock = xSemaphoreCreateMutexStatic(&mChipStackLockMutex);
#else
        mChipStackLock  = xSemaphoreCreateMutex();
#endif

        if (mChipStackLock == NULL)
        {
            ChipLogError(DeviceLayer, "Failed to create CHIP stack lock");
            ExitNow(err = CHIP_ERROR_NO_MEMORY);
        }
    }

    if (mChipEventQueue == NULL)
    {
#if defined(CHIP_CONFIG_FREERTOS_USE_STATIC_QUEUE) && CHIP_CONFIG_FREERTOS_USE_STATIC_QUEUE
        mChipEventQueue = xQueueCreateStatic(CHIP_DEVICE_CONFIG_MAX_EVENT_QUEUE_SIZE, sizeof(ChipDeviceEvent), mEventQueueBuffer,
                                             &mEventQueueStruct);
#else
        mChipEventQueue = xQueueCreate(CHIP_DEVICE_CONFIG_MAX_EVENT_QUEUE_SIZE, sizeof(ChipDeviceEvent));
#endif
        if (mChipEventQueue == NULL)
        {
            ChipLogError(DeviceLayer, "Failed to allocate CHIP main event queue");
            ExitNow(err = CHIP_ERROR_NO_MEMORY);
        }
    }
    else
    {
        xQueueReset(mChipEventQueue);
    }

    mShouldRunEventLoop.store(false);

#if defined(CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING) && CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING
    if (mBackgroundEventQueue == NULL)
    {
#if defined(CHIP_CONFIG_FREERTOS_USE_STATIC_QUEUE) && CHIP_CONFIG_FREERTOS_USE_STATIC_QUEUE
        mBackgroundEventQueue = xQueueCreateStatic(CHIP_DEVICE_CONFIG_BG_MAX_EVENT_QUEUE_SIZE, sizeof(ChipDeviceEvent),
                                                   mBackgroundQueueBuffer, &mBackgroundQueueStruct);
#else
        mBackgroundEventQueue = xQueueCreate(CHIP_DEVICE_CONFIG_BG_MAX_EVENT_QUEUE_SIZE, sizeof(ChipDeviceEvent));
#endif
        if (mBackgroundEventQueue == NULL)
        {
            ChipLogError(DeviceLayer, "Failed to allocate CHIP background event queue");
            ExitNow(err = CHIP_ERROR_NO_MEMORY);
        }
    }
    else
    {
        xQueueReset(mBackgroundEventQueue);
    }

    mShouldRunBackgroundEventLoop.store(false);
#endif

    err = GenericPlatformManagerImpl<ImplClass>::_InitChipStack();
    SuccessOrExit(err);

exit:
    return err;
}

template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_LockChipStack(void)
{
    xSemaphoreTake(mChipStackLock, portMAX_DELAY);
}

template <class ImplClass>
bool GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_TryLockChipStack(void)
{
    return xSemaphoreTake(mChipStackLock, 0) == pdTRUE;
}

template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_UnlockChipStack(void)
{
    xSemaphoreGive(mChipStackLock);
}

#if CHIP_STACK_LOCK_TRACKING_ENABLED
template <class ImplClass>
bool GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_IsChipStackLockedByCurrentThread() const
{
    return (mEventLoopTask == nullptr) || (mChipStackLock == nullptr) ||
        (xSemaphoreGetMutexHolder(mChipStackLock) == xTaskGetCurrentTaskHandle());
}
#endif // CHIP_STACK_LOCK_TRACKING_ENABLED

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_PostEvent(const ChipDeviceEvent * event)
{
    if (mChipEventQueue == NULL)
    {
        return CHIP_ERROR_INTERNAL;
    }
    BaseType_t status = xQueueSend(mChipEventQueue, event, 1);
    if (status != pdTRUE)
    {
        ChipLogError(DeviceLayer, "Failed to post event to CHIP Platform event queue");
        return CHIP_ERROR(chip::ChipError::Range::kOS, status);
    }
    return CHIP_NO_ERROR;
}

template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_RunEventLoop(void)
{
    CHIP_ERROR err;
    ChipDeviceEvent event;

    StackLock lock;

    bool oldShouldRunEventLoop = false;
    if (!mShouldRunEventLoop.compare_exchange_strong(oldShouldRunEventLoop /* expected */, true /* desired */))
    {
        ChipLogError(DeviceLayer, "Error trying to run the event loop while it is already running");
        return;
    }

    while (mShouldRunEventLoop.load())
    {
        TickType_t waitTime;

        if (mChipTimerActive)
        {
            if (xTaskCheckForTimeOut(&mNextTimerBaseTime, &mNextTimerDurationTicks) == pdTRUE)
            {
                mChipTimerActive = false;

                /* Sesame: when CHIP_SYSTEM_CONFIG_USE_SOCKETS=1, LayerImplSelect
                 * manages its own timer list via select() and never calls
                 * _StartChipTimer(), so mChipTimerActive is always false and
                 * this branch is unreachable dead code.  The original cast to
                 * LayerImplFreeRTOS (an LwIP-only type) is replaced with a
                 * no-op to keep the build clean. */
                err = CHIP_NO_ERROR;
                if (err != CHIP_NO_ERROR)
                {
                    ChipLogError(DeviceLayer, "Error handling CHIP timers: %" CHIP_ERROR_FORMAT, err.Format());
                }

                waitTime = 0;
            }
            else
            {
                waitTime = mNextTimerDurationTicks;
            }
        }
        else
        {
            waitTime = portMAX_DELAY;
        }

        BaseType_t eventReceived = pdFALSE;
        {
            StackUnlock unlock;
            eventReceived = xQueueReceive(mChipEventQueue, &event, waitTime);
        }

        while (eventReceived == pdTRUE)
        {
            Impl()->DispatchEvent(&event);
            {
                StackUnlock unlock;
                eventReceived = xQueueReceive(mChipEventQueue, &event, 0);
            }
        }
    }
}

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_StartEventLoopTask(void)
{
#if defined(CHIP_CONFIG_FREERTOS_USE_STATIC_TASK) && CHIP_CONFIG_FREERTOS_USE_STATIC_TASK
    mEventLoopTask = xTaskCreateStatic(EventLoopTaskMain, CHIP_DEVICE_CONFIG_CHIP_TASK_NAME, MATTER_ARRAY_SIZE(mEventLoopStack),
                                       this, CHIP_DEVICE_CONFIG_CHIP_TASK_PRIORITY, mEventLoopStack, &mEventLoopTaskStruct);
#else
    xTaskCreate(EventLoopTaskMain, CHIP_DEVICE_CONFIG_CHIP_TASK_NAME, CHIP_DEVICE_CONFIG_CHIP_TASK_STACK_SIZE / sizeof(StackType_t),
                this, CHIP_DEVICE_CONFIG_CHIP_TASK_PRIORITY, &mEventLoopTask);
#endif
    return (mEventLoopTask != NULL) ? CHIP_NO_ERROR : CHIP_ERROR_NO_MEMORY;
}

template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::EventLoopTaskMain(void * arg)
{
    ChipLogDetail(DeviceLayer, "CHIP event task running");
    GenericPlatformManagerImpl_FreeRTOS<ImplClass> * platformManager =
        static_cast<GenericPlatformManagerImpl_FreeRTOS<ImplClass> *>(arg);
    platformManager->Impl()->RunEventLoop();
    ChipLogDetail(DeviceLayer, "CHIP event task stopped");

    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    xTaskNotifyGive(currentTask);

    vTaskDelete(NULL);
}

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_PostBackgroundEvent(const ChipDeviceEvent * event)
{
#if defined(CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING) && CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING
    if (mBackgroundEventQueue == NULL)
    {
        return CHIP_ERROR_INTERNAL;
    }
    if (!(event->Type == DeviceEventType::kCallWorkFunct || event->Type == DeviceEventType::kNoOp))
    {
        return CHIP_ERROR_INVALID_ARGUMENT;
    }
    auto status = xQueueSendToBack(mBackgroundEventQueue, event, 1);
    if (status != pdTRUE)
    {
        ChipLogError(DeviceLayer, "Failed to post event to CHIP background event queue");
        return CHIP_ERROR_NO_MEMORY;
    }
    return CHIP_NO_ERROR;
#else
    return _PostEvent(event);
#endif
}

template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_RunBackgroundEventLoop(void)
{
#if defined(CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING) && CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING
    bool oldShouldRunBackgroundEventLoop = false;
    if (!mShouldRunBackgroundEventLoop.compare_exchange_strong(oldShouldRunBackgroundEventLoop /* expected */, true /* desired */))
    {
        ChipLogError(DeviceLayer, "Error trying to run the background event loop while it is already running");
        return;
    }

    while (mShouldRunBackgroundEventLoop.load())
    {
        ChipDeviceEvent event;
        auto eventReceived = xQueueReceive(mBackgroundEventQueue, &event, portMAX_DELAY) == pdTRUE;
        while (eventReceived)
        {
            Impl()->DispatchEvent(&event);
            eventReceived = xQueueReceive(mBackgroundEventQueue, &event, portMAX_DELAY) == pdTRUE;
        }
    }
#endif
}

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_StartBackgroundEventLoopTask(void)
{
#if defined(CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING) && CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING
#if defined(CHIP_CONFIG_FREERTOS_USE_STATIC_TASK) && CHIP_CONFIG_FREERTOS_USE_STATIC_TASK
    mBackgroundEventLoopTask = xTaskCreateStatic(
        BackgroundEventLoopTaskMain, CHIP_DEVICE_CONFIG_BG_TASK_NAME, MATTER_ARRAY_SIZE(mBackgroundEventLoopStack), this,
        CHIP_DEVICE_CONFIG_BG_TASK_PRIORITY, mBackgroundEventLoopStack, &mBackgroundEventLoopTaskStruct);
#else
    xTaskCreate(BackgroundEventLoopTaskMain, CHIP_DEVICE_CONFIG_BG_TASK_NAME,
                CHIP_DEVICE_CONFIG_BG_TASK_STACK_SIZE / sizeof(StackType_t), this, CHIP_DEVICE_CONFIG_BG_TASK_PRIORITY,
                &mBackgroundEventLoopTask);
#endif
    return (mBackgroundEventLoopTask != NULL) ? CHIP_NO_ERROR : CHIP_ERROR_NO_MEMORY;
#else
    return CHIP_NO_ERROR;
#endif
}

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_StopBackgroundEventLoopTask(void)
{
#if defined(CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING) && CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING
    bool oldShouldRunBackgroundEventLoop = true;
    if (mShouldRunBackgroundEventLoop.compare_exchange_strong(oldShouldRunBackgroundEventLoop /* expected */, false /* desired */))
    {
        ChipDeviceEvent noop{ .Type = DeviceEventType::kNoOp };
        xQueueSendToBack(mBackgroundEventQueue, &noop, 0);
    }
    return CHIP_NO_ERROR;
#else
    return CHIP_NO_ERROR;
#endif
}

#if defined(CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING) && CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING
template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::BackgroundEventLoopTaskMain(void * arg)
{
    ChipLogDetail(DeviceLayer, "CHIP background task running");
    GenericPlatformManagerImpl_FreeRTOS<ImplClass> * platformManager =
        static_cast<GenericPlatformManagerImpl_FreeRTOS<ImplClass> *>(arg);
    platformManager->Impl()->RunBackgroundEventLoop();
    vTaskDelete(NULL);
    platformManager->mBackgroundEventLoopTask = NULL;
}
#endif

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_StartChipTimer(System::Clock::Timeout delay)
{
    mChipTimerActive = true;
    vTaskSetTimeOutState(&mNextTimerBaseTime);
    mNextTimerDurationTicks = pdMS_TO_TICKS(System::Clock::Milliseconds64(delay).count());

    if (xTaskGetCurrentTaskHandle() != mEventLoopTask)
    {
        ChipDeviceEvent noop{ .Type = DeviceEventType::kNoOp };
        ReturnErrorOnFailure(Impl()->PostEvent(&noop));
    }

    return CHIP_NO_ERROR;
}

template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::PostEventFromISR(const ChipDeviceEvent * event, BaseType_t & yieldRequired)
{
    yieldRequired = pdFALSE;

    if (mChipEventQueue != NULL)
    {
        if (!xQueueSendFromISR(mChipEventQueue, event, &yieldRequired))
        {
            ChipLogError(DeviceLayer, "Failed to post event to CHIP Platform event queue");
        }
    }
}

template <class ImplClass>
void GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_Shutdown(void)
{
    if (mEventLoopTask != NULL)
    {
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS(2000);
        if (ulTaskNotifyTake(pdTRUE, xMaxBlockTime) == 0)
        {
            ChipLogError(DeviceLayer, "Event loop task failed to exit within timeout");
        }

        mEventLoopTask = NULL;
    }

    if (mChipEventQueue)
    {
        vQueueDelete(mChipEventQueue);
        mChipEventQueue = NULL;
    }
#if defined(CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING) && CHIP_DEVICE_CONFIG_ENABLE_BG_EVENT_PROCESSING
    if (mBackgroundEventQueue)
    {
        vQueueDelete(mBackgroundEventQueue);
        mBackgroundEventQueue = NULL;
    }
#endif
    if (mChipStackLock)
    {
        vSemaphoreDelete(mChipStackLock);
        mChipStackLock = NULL;
    }
    GenericPlatformManagerImpl<ImplClass>::_Shutdown();
}

template <class ImplClass>
CHIP_ERROR GenericPlatformManagerImpl_FreeRTOS<ImplClass>::_StopEventLoopTask(void)
{
    if (mEventLoopTask != NULL)
    {
        mShouldRunEventLoop.store(false);

        ChipDeviceEvent noop{ .Type = DeviceEventType::kNoOp };
        if (mChipEventQueue != NULL)
        {
            xQueueSend(mChipEventQueue, &noop, 0);
        }
    }
    return CHIP_NO_ERROR;
}

template class GenericPlatformManagerImpl_FreeRTOS<PlatformManagerImpl>;

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip

#endif // GENERIC_PLATFORM_MANAGER_IMPL_FREERTOS_CPP
