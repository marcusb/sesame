#ifndef _ZEPHYR_WM_OS_H_
#define _ZEPHYR_WM_OS_H_
#define _WM_OS_H_

#include <zephyr/kernel.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define hex2bin wm_hex2bin
#define bin2hex wm_bin2hex
#define wifi_scan_result mw320_wifi_scan_result

#include <zephyr/sys/util.h>

#define xQueueHandle os_queue_t
typedef void (*TimerCallbackFunction_t)(void *);

#include <wmerrno.h>
#include <wm_utils.h>

#define os_dprintf(...) // printk("[OS]" __VA_ARGS__)

#define is_isr_context() k_is_in_isr()

extern uint32_t SystemCoreClock;

#define CNTMAX                 ((SystemCoreClock / CONFIG_SYS_CLOCK_TICKS_PER_SEC) - 1UL)
#define CPU_CLOCK_TICKSPERUSEC (SystemCoreClock / 1000000U)
#define USECSPERTICK           (1000000U / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#define os_thread_relinquish() k_yield()

static inline unsigned os_ticks_get(void)
{
    return k_uptime_ticks();
}

static inline unsigned int os_get_timestamp(void)
{
    return k_uptime_get() * 1000;
}

typedef void *os_thread_arg_t;

typedef struct os_thread_stack {
    int size;
} os_thread_stack_t;

#define os_thread_stack_define(stackname, stacksize) \
    os_thread_stack_t stackname = { stacksize }

typedef struct k_thread * os_thread_t;

typedef uint32_t portSTACK_TYPE;

static inline const char *get_current_taskname(void)
{
    return k_thread_name_get(k_current_get());
}

int os_thread_create(os_thread_t *thandle,
                     const char *name,
                     void (*main_func)(os_thread_arg_t arg),
                     void *arg,
                     os_thread_stack_t *stack,
                     int prio);

int os_event_notify_get(unsigned long wait_time);
void os_event_notify_put(os_thread_t thread);

static inline os_thread_t os_get_current_task_handle(void)
{
    return k_current_get();
}

static inline int os_thread_delete(os_thread_t *thandle)
{
    if (thandle == NULL) {
        k_thread_abort(k_current_get());
    } else {
        k_thread_abort(*thandle);
    }
    if (thandle) *thandle = NULL;
    return WM_SUCCESS;
}

static inline void os_thread_sleep(int ticks)
{
    k_sleep(K_TICKS(ticks));
}

static inline unsigned long os_msec_to_ticks(unsigned long msecs)
{
    return k_ms_to_ticks_ceil32(msecs);
}

static inline unsigned long os_ticks_to_msec(unsigned long ticks)
{
    return k_ticks_to_ms_floor64(ticks);
}

static inline void os_thread_self_complete(os_thread_t *thandle)
{
    if (thandle != NULL) {
        k_thread_suspend(*thandle);
    } else {
        k_thread_suspend(k_current_get());
    }
    while (1) k_sleep(K_MSEC(60000));
}

#define OS_PRIO_0 K_PRIO_COOP(0)
#define OS_PRIO_1 K_PRIO_COOP(1)
#define OS_PRIO_2 K_PRIO_COOP(2)
#define OS_PRIO_3 K_PRIO_COOP(3)
#define OS_PRIO_4 K_PRIO_COOP(4)

typedef struct os_queue_pool {
    int size;
} os_queue_pool_t;

#define os_queue_pool_define(poolname, poolsize) os_queue_pool_t poolname = {poolsize};

typedef struct k_msgq * os_queue_t;

int os_queue_create(os_queue_t *qhandle, const char *name, int msgsize, os_queue_pool_t *poolname);

#define OS_WAIT_FOREVER 0xFFFFFFFFUL
#define OS_NO_WAIT      0

static inline k_timeout_t os_wait_to_timeout(unsigned long wait) {
    if (wait == OS_WAIT_FOREVER) return K_FOREVER;
    if (wait == OS_NO_WAIT) return K_NO_WAIT;
    return K_TICKS(wait);
}

static inline int os_queue_send(os_queue_t *qhandle, const void *msg, unsigned long wait)
{
    if (!qhandle || !(*qhandle)) return -WM_E_INVAL;
    int ret = k_msgq_put(*qhandle, msg, os_wait_to_timeout(wait));
    return ret == 0 ? WM_SUCCESS : -WM_FAIL;
}

static inline int os_queue_recv(os_queue_t *qhandle, void *msg, unsigned long wait)
{
    if (!qhandle || !(*qhandle)) return -WM_E_INVAL;
    int ret = k_msgq_get(*qhandle, msg, os_wait_to_timeout(wait));
    return ret == 0 ? WM_SUCCESS : -WM_FAIL;
}

int os_queue_delete(os_queue_t *qhandle);

static inline int os_queue_get_msgs_waiting(os_queue_t *qhandle)
{
    if (!qhandle || !(*qhandle)) return -WM_E_INVAL;
    return k_msgq_num_used_get(*qhandle);
}

static inline unsigned long os_enter_critical_section(void)
{
    return irq_lock();
}

static inline void os_exit_critical_section(unsigned long state)
{
    irq_unlock(state);
}

typedef struct k_mutex * os_mutex_t;

#define OS_MUTEX_INHERIT 1
#define OS_MUTEX_NO_INHERIT 0

int os_mutex_create(os_mutex_t *mhandle, const char *name, int flags);

static inline int os_mutex_get(os_mutex_t *mhandle, unsigned long wait)
{
    if (!mhandle || !(*mhandle)) return -WM_E_INVAL;
    int ret = k_mutex_lock(*mhandle, os_wait_to_timeout(wait));
    return ret == 0 ? WM_SUCCESS : -WM_FAIL;
}

static inline int os_mutex_put(os_mutex_t *mhandle)
{
    if (!mhandle || !(*mhandle)) return -WM_E_INVAL;
    k_mutex_unlock(*mhandle);
    return WM_SUCCESS;
}

int os_mutex_delete(os_mutex_t *mhandle);

typedef struct k_sem * os_semaphore_t;

int os_semaphore_create(os_semaphore_t *shandle, const char *name);
int os_semaphore_create_counting(os_semaphore_t *shandle, const char *name, int max, int init);

static inline int os_semaphore_get(os_semaphore_t *shandle, unsigned long wait)
{
    if (!shandle || !(*shandle)) return -WM_E_INVAL;
    int ret = k_sem_take(*shandle, os_wait_to_timeout(wait));
    return ret == 0 ? WM_SUCCESS : -WM_FAIL;
}

static inline int os_semaphore_put(os_semaphore_t *shandle)
{
    if (!shandle || !(*shandle)) return -WM_E_INVAL;
    k_sem_give(*shandle);
    return WM_SUCCESS;
}

int os_semaphore_delete(os_semaphore_t *shandle);
int os_semaphore_get_count(os_semaphore_t *shandle);

typedef struct k_timer * os_timer_t;
typedef void (*os_timer_cb_t)(void *);

enum os_timer_reload {
    OS_TIMER_ONE_SHOT,
    OS_TIMER_PERIODIC
};
typedef enum os_timer_reload os_timer_reload_t;

enum os_timer_activate {
    OS_TIMER_AUTO_ACTIVATE,
    OS_TIMER_NO_ACTIVATE
};
typedef enum os_timer_activate os_timer_activate_t;

typedef unsigned long os_timer_tick;
typedef void *os_timer_arg_t;

int os_timer_create(os_timer_t *timer_t, const char *name, os_timer_tick ticks,
                    void (*call_back)(os_timer_arg_t), void *cb_arg,
                    os_timer_reload_t reload, os_timer_activate_t activate);
int os_timer_activate(os_timer_t *timer_t);
int os_timer_change(os_timer_t *timer_t, os_timer_tick ticks, os_timer_tick block_time);
int os_timer_deactivate(os_timer_t *timer_t);
int os_timer_delete(os_timer_t *timer_t);
int os_timer_reset(os_timer_t *timer_t);

void *os_mem_alloc(size_t size);
void *os_mem_calloc(size_t size);
void *os_mem_realloc(void *ptr, size_t size);
void os_mem_free(void *ptr);

typedef struct os_rw_lock {
    os_mutex_t reader_mutex;
    os_semaphore_t rw_lock;
    int reader_count;
    int (*reader_cb)(struct os_rw_lock *, unsigned int);
} os_rw_lock_t;

typedef int (*cb_fn)(os_rw_lock_t *, unsigned int);

int os_rwlock_create(os_rw_lock_t *plock, const char *mutex_name, const char *lock_name);
int os_rwlock_create_with_cb(os_rw_lock_t *plock, const char *mutex_name, const char *lock_name, cb_fn r_fn);
int os_rwlock_read_lock(os_rw_lock_t *lock, unsigned int wait_time);
int os_rwlock_read_unlock(os_rw_lock_t *lock);
int os_rwlock_write_lock(os_rw_lock_t *lock, unsigned int wait_time);
void os_rwlock_write_unlock(os_rw_lock_t *lock);
void os_rwlock_delete(os_rw_lock_t *lock);

static inline void *os_timer_get_context(os_timer_t *timer_t)
{
    if (!timer_t || !(*timer_t))
        return (void *)-1;

    return k_timer_user_data_get(*timer_t);
}

#endif /* _ZEPHYR_WM_OS_H_ */
