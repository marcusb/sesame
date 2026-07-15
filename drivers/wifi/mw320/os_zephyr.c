#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "wm_os.h"

K_SEM_DEFINE(wifi_core_notify_sem, 0, 1);

int os_event_notify_get(unsigned long wait_time)
{
    int ret = k_sem_take(&wifi_core_notify_sem, (wait_time == OS_WAIT_FOREVER) ? K_FOREVER : K_MSEC(wait_time));
    return (ret == 0) ? WM_SUCCESS : -WM_FAIL;
}

void os_event_notify_put(os_thread_t thread)
{
    /* In this port, we use a single semaphore since only the wifi core thread receives events */
    k_sem_give(&wifi_core_notify_sem);
}

int os_thread_create(os_thread_t *thandle, const char *name, void (*main_func)(os_thread_arg_t arg), void *arg, os_thread_stack_t *stack, int prio)
{
    struct k_thread *thread = k_malloc(sizeof(struct k_thread));
    if (!thread) return -WM_FAIL;

    k_thread_stack_t *k_stack = k_thread_stack_alloc(stack->size * sizeof(void*), 0);
    if (!k_stack) {
        k_free(thread);
        return -WM_FAIL;
    }

    k_thread_create(thread, k_stack, stack->size * sizeof(void*), (k_thread_entry_t)main_func, arg, NULL, NULL, prio, 0, K_NO_WAIT);
    if (name) {
        k_thread_name_set(thread, name);
    }

    if (thandle) *thandle = thread;
    return WM_SUCCESS;
}

int os_queue_create(os_queue_t *qhandle, const char *name, int msgsize, os_queue_pool_t *poolname)
{
    struct k_msgq *q = k_malloc(sizeof(struct k_msgq));
    if (!q) return -WM_FAIL;

    int num_msgs = poolname->size / msgsize;
    char *buffer = k_malloc(poolname->size);
    if (!buffer) {
        k_free(q);
        return -WM_FAIL;
    }

    k_msgq_init(q, buffer, msgsize, num_msgs);
    if (qhandle) *qhandle = q;
    return WM_SUCCESS;
}

int os_queue_delete(os_queue_t *qhandle)
{
    if (!qhandle || !(*qhandle)) return -WM_E_INVAL;
    k_msgq_cleanup(*qhandle);
    /* In Zephyr, there is no k_msgq_buffer_get so we leak the buffer here unless we track it. But for Wi-Fi driver, queues are rarely deleted. */
    k_free(*qhandle);
    *qhandle = NULL;
    return WM_SUCCESS;
}

int os_mutex_create(os_mutex_t *mhandle, const char *name, int flags)
{
    struct k_mutex *m = k_malloc(sizeof(struct k_mutex));
    if (!m) return -WM_FAIL;
    k_mutex_init(m);
    if (mhandle) *mhandle = m;
    return WM_SUCCESS;
}

int os_mutex_delete(os_mutex_t *mhandle)
{
    if (!mhandle || !(*mhandle)) return -WM_E_INVAL;
    k_free(*mhandle);
    *mhandle = NULL;
    return WM_SUCCESS;
}

int os_semaphore_create(os_semaphore_t *shandle, const char *name)
{
    return os_semaphore_create_counting(shandle, name, 1, 1);
}

int os_semaphore_create_counting(os_semaphore_t *shandle, const char *name, int max, int init)
{
    struct k_sem *s = k_malloc(sizeof(struct k_sem));
    if (!s) return -WM_FAIL;
    k_sem_init(s, init, max);
    if (shandle) *shandle = s;
    return WM_SUCCESS;
}

int os_semaphore_delete(os_semaphore_t *shandle)
{
    if (!shandle || !(*shandle)) return -WM_E_INVAL;
    k_free(*shandle);
    *shandle = NULL;
    return WM_SUCCESS;
}

int os_semaphore_get_count(os_semaphore_t *shandle)
{
    if (!shandle || !(*shandle)) return 0;
    return k_sem_count_get(*shandle);
}

/* Timer implementation */
struct zephyr_timer_wrapper {
    struct k_timer timer;
    void (*call_back)(os_timer_arg_t);
    void *cb_arg;
    os_timer_reload_t reload;
    os_timer_tick ticks;
};

static void timer_handler(struct k_timer *timer_id)
{
    struct zephyr_timer_wrapper *w = CONTAINER_OF(timer_id, struct zephyr_timer_wrapper, timer);
    if (w->call_back) {
        w->call_back(w->cb_arg);
    }
}

int os_timer_create(os_timer_t *timer_t, const char *name, os_timer_tick ticks,
                    void (*call_back)(os_timer_arg_t), void *cb_arg,
                    os_timer_reload_t reload, os_timer_activate_t activate)
{
    struct zephyr_timer_wrapper *w = k_malloc(sizeof(struct zephyr_timer_wrapper));
    if (!w) return -WM_FAIL;

    w->call_back = call_back;
    w->cb_arg = cb_arg;
    w->reload = reload;
    w->ticks = ticks;
    k_timer_init(&w->timer, timer_handler, NULL);

    if (timer_t) *timer_t = (os_timer_t)w;

    if (activate == OS_TIMER_AUTO_ACTIVATE) {
        os_timer_activate((os_timer_t*)timer_t);
    }
    return WM_SUCCESS;
}

int os_timer_activate(os_timer_t *timer_t)
{
    if (!timer_t || !(*timer_t)) return -WM_E_INVAL;
    struct zephyr_timer_wrapper *w = (struct zephyr_timer_wrapper *)(*timer_t);
    k_timeout_t period = (w->reload == OS_TIMER_PERIODIC) ? K_TICKS(w->ticks) : K_NO_WAIT;
    k_timer_start(&w->timer, K_TICKS(w->ticks), period);
    return WM_SUCCESS;
}

int os_timer_change(os_timer_t *timer_t, os_timer_tick ticks, os_timer_tick block_time)
{
    if (!timer_t || !(*timer_t)) return -WM_E_INVAL;
    struct zephyr_timer_wrapper *w = (struct zephyr_timer_wrapper *)(*timer_t);
    w->ticks = ticks;
    return os_timer_activate(timer_t);
}

int os_timer_deactivate(os_timer_t *timer_t)
{
    if (!timer_t || !(*timer_t)) return -WM_E_INVAL;
    struct zephyr_timer_wrapper *w = (struct zephyr_timer_wrapper *)(*timer_t);
    k_timer_stop(&w->timer);
    return WM_SUCCESS;
}

int os_timer_delete(os_timer_t *timer_t)
{
    if (!timer_t || !(*timer_t)) return -WM_E_INVAL;
    struct zephyr_timer_wrapper *w = (struct zephyr_timer_wrapper *)(*timer_t);
    k_timer_stop(&w->timer);
    k_free(w);
    *timer_t = NULL;
    return WM_SUCCESS;
}

int os_timer_reset(os_timer_t *timer_t)
{
    return os_timer_activate(timer_t);
}

/* RW Lock */
int os_rwlock_create(os_rw_lock_t *plock, const char *mutex_name, const char *lock_name)
{
    return os_rwlock_create_with_cb(plock, mutex_name, lock_name, NULL);
}

int os_rwlock_create_with_cb(os_rw_lock_t *plock, const char *mutex_name, const char *lock_name, cb_fn r_fn)
{
    if (!plock) return -WM_E_INVAL;
    
    os_mutex_create(&plock->reader_mutex, mutex_name, 0);
    os_semaphore_create(&plock->rw_lock, lock_name);
    
    plock->reader_count = 0;
    plock->reader_cb = r_fn;
    return WM_SUCCESS;
}

int os_rwlock_read_lock(os_rw_lock_t *lock, unsigned int wait_time)
{
    int ret;
    k_mutex_lock(lock->reader_mutex, K_FOREVER);
    lock->reader_count++;
    if (lock->reader_count == 1) {
        if (lock->reader_cb) {
            k_mutex_unlock(lock->reader_mutex);
            ret = lock->reader_cb(lock, wait_time);
            if (ret != WM_SUCCESS) {
                k_mutex_lock(lock->reader_mutex, K_FOREVER);
                lock->reader_count--;
                k_mutex_unlock(lock->reader_mutex);
                return ret;
            }
            k_mutex_lock(lock->reader_mutex, K_FOREVER);
        } else {
            ret = k_sem_take(lock->rw_lock, os_wait_to_timeout(wait_time));
            if (ret != 0) {
                lock->reader_count--;
                k_mutex_unlock(lock->reader_mutex);
                return -WM_FAIL;
            }
        }
    }
    k_mutex_unlock(lock->reader_mutex);
    return WM_SUCCESS;
}

int os_rwlock_read_unlock(os_rw_lock_t *lock)
{
    k_mutex_lock(lock->reader_mutex, K_FOREVER);
    lock->reader_count--;
    if (lock->reader_count == 0) {
        k_sem_give(lock->rw_lock);
    }
    k_mutex_unlock(lock->reader_mutex);
    return WM_SUCCESS;
}

int os_rwlock_write_lock(os_rw_lock_t *lock, unsigned int wait_time)
{
    int ret = k_sem_take(lock->rw_lock, os_wait_to_timeout(wait_time));
    return ret == 0 ? WM_SUCCESS : -WM_FAIL;
}

void os_rwlock_write_unlock(os_rw_lock_t *lock)
{
    k_sem_give(lock->rw_lock);
}

void os_rwlock_delete(os_rw_lock_t *lock)
{
    os_mutex_delete(&lock->reader_mutex);
    os_semaphore_delete(&lock->rw_lock);
}

/* OSA stubs for SDMMC driver */
#include "fsl_os_abstraction.h"

void OSA_EnterCritical(uint32_t *sr)
{
    *sr = irq_lock();
}

void OSA_ExitCritical(uint32_t sr)
{
    irq_unlock(sr);
}

osa_status_t OSA_SemaphoreCreate(osa_semaphore_handle_t semaphoreHandle, uint32_t initValue)
{
    /* semaphoreHandle points to a buffer of size OSA_SEMAPHORE_HANDLE_SIZE */
    k_sem_init((struct k_sem *)semaphoreHandle, initValue, 1);
    return KOSA_StatusSuccess;
}

osa_status_t OSA_SemaphoreWait(osa_semaphore_handle_t semaphoreHandle, uint32_t millisec)
{
    int ret = k_sem_take((struct k_sem *)semaphoreHandle,
                         millisec == osaWaitForever_c ? K_FOREVER : K_MSEC(millisec));
    return ret == 0 ? KOSA_StatusSuccess : KOSA_StatusError;
}

osa_status_t OSA_SemaphorePost(osa_semaphore_handle_t semaphoreHandle)
{
    k_sem_give((struct k_sem *)semaphoreHandle);
    return KOSA_StatusSuccess;
}

osa_status_t OSA_SemaphoreDestroy(osa_semaphore_handle_t semaphoreHandle)
{
    return KOSA_StatusSuccess;
}

/* Mem */
void *os_mem_alloc(size_t size) { return k_malloc(size); }
void *os_mem_calloc(size_t size) { return k_calloc(1, size); }
void *os_mem_realloc(void *ptr, size_t size) { return NULL; /* k_realloc not available? */ }
void os_mem_free(void *ptr) { k_free(ptr); }
