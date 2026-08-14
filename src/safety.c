#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

#include "safety.h"
#include "global_objects.h"

#define SAFTEY_THREAD_SLEEP_TIME_MS 20
#define TIMEOUT_PERIOD 4 // Runs at 250 hz, 1000/250
#define ATTITUDE_TIMEOUT 2 // 500 hz
#define IMU_TIMEOUT 2 // 500 hz

#define MAX_SAFE_ROLL 10000 // deg * 100

LOG_MODULE_REGISTER(safety_task, LOG_LEVEL_DBG);


static flight_state_t flight_state;


static const struct device *wdt =
    DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int wdt_channel_id;


int init_watchdog() {
    
    if (!device_is_ready(wdt)) {
            return -1;
        }

    const struct wdt_timeout_cfg config = {
        .flags = WDT_FLAG_RESET_SOC,
        .window = {
            .min = 0,
            .max = 200,
        },
        .callback = NULL,
    };
    wdt_channel_id = wdt_install_timeout(wdt, &config);
    if (wdt_channel_id < 0) {
        LOG_ERR("Failed to install watchdog: %d", wdt_channel_id);
        return wdt_channel_id;
    }
    int ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (ret < 0) {
        LOG_ERR("Failed to setup watchdog: %d", ret);
        return ret;
    }
    return 0;
}

// Check for time delta
int time_delta(const atomic_t *val, int timeout) {
    uint32_t curr_time = k_uptime_get_32();
    uint32_t delta = curr_time - (uint32_t)atomic_get(val);

    if (delta > timeout) {
        return -1;
    }
    else {
        return 0;
    }
}



atomic_t pid_heartbeat = ATOMIC_INIT(0);
atomic_t attitude_heartbeat = ATOMIC_INIT(0);
atomic_t imu_heartbeat = ATOMIC_INIT(0);
atomic_t global_roll = ATOMIC_INIT(0);
    
void safety_thread(void* p1, void* p2, void* p3) {

    bool pid_ok = true, attitude_ok = true, imu_ok = true;

    while (1) {

        if (!time_delta(pid_heartbeat, TIMEOUT_PERIOD)) {
            pid_ok = false;
        }
        if (!time_delta(attitude_heartbeat, ATTITUDE_TIMEOUT)) {
            attitude_ok = false;
        }
        if (!time_delta(imu_heartbeat, IMU_TIMEOUT)) {
            imu_ok = false;
        }

        if (atomic_get(&global_roll) > MAX_SAFE_ROLL) {
            flight_state = FLIGHT_FAULT;
        }

        // Checks to see if other threads are OK, and if so then it feeds the watchdog
        if (!pid_ok || !attitude_ok || !imu_ok) {
            flight_state = FLIGHT_FAULT;
        }
        else {
            int ret = wdt_feed(wdt, wdt_channel_id);
            if (ret < 0) {
                LOG_ERR("Feeding watchdog issue: %d", ret);
            }
        }

        k_sleep(K_MSEC(SAFTEY_THREAD_SLEEP_TIME_MS));
    }

}



int check_flight_state() {
    if (flight_state == FLIGHT_FAULT) {
        return 1;
    }
    else if (flight_state == FLIGHT_DISARMED) {
        return 2;
    }
    else
    return 0;
}

