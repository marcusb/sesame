#include "wm_os.h"

int os_rwlock_create(os_rw_lock_t* lock, const char* mutex_name,
                     const char* lock_name) {
    (void)lock;
    (void)mutex_name;
    (void)lock_name;
    return 0;
}
int os_rwlock_read_lock(os_rw_lock_t* lock, unsigned int wait_time) {
    (void)lock;
    (void)wait_time;
    return 0;
}
int os_rwlock_read_unlock(os_rw_lock_t* lock) {
    (void)lock;
    return 0;
}
int os_rwlock_write_lock(os_rw_lock_t* lock, unsigned int wait_time) {
    (void)lock;
    (void)wait_time;
    return 0;
}
void os_rwlock_write_unlock(os_rw_lock_t* lock) { (void)lock; }
void os_rwlock_delete(os_rw_lock_t* lock) { (void)lock; }
