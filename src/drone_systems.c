#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

#include "global_objects.h"
#include "ble.h"

#define DRONE_SYSTEMS_NODE DT_PATH(zephyr_user)
#define DRONE_SYSTEMS_ADC_PERIOD_MS 200
#define MOVING_AVG_WINDOW 5
#define BATTERY_DIVIDER_NUMERATOR 28
#define BATTERY_DIVIDER_DENOMINATOR 10

struct batt_soc_point {
    int32_t battery_mv;
    uint16_t soc_pct_tenths;
};

static const struct batt_soc_point soc_table[] = {
    { 6400, 0 },
    { 7000, 25 },
    { 7200, 75 },
    { 7400, 175 },
    { 7600, 375 },
    { 7800, 575 },
    { 8000, 775 },
    { 8200, 900 },
    { 8400, 1000 },
};

static const struct adc_dt_spec systems_adc =
    ADC_DT_SPEC_GET_BY_NAME(DRONE_SYSTEMS_NODE, systems);

LOG_MODULE_REGISTER(drone_systems, LOG_LEVEL_DBG);

static uint16_t get_charge_percentage(int32_t battery_mv)
{
    if (battery_mv <= soc_table[0].battery_mv) {
        return soc_table[0].soc_pct_tenths;
    }

    size_t num_points = ARRAY_SIZE(soc_table);
    if (battery_mv >= soc_table[num_points - 1].battery_mv) {
        return soc_table[num_points - 1].soc_pct_tenths;
    }

    for (size_t i = 0; i < num_points - 1; i++) {
        if (battery_mv <= soc_table[i + 1].battery_mv) {
            int32_t voltage_range = soc_table[i + 1].battery_mv -
                                     soc_table[i].battery_mv;
            int32_t soc_range = soc_table[i + 1].soc_pct_tenths -
                                soc_table[i].soc_pct_tenths;
            int32_t voltage_from_lower = battery_mv - soc_table[i].battery_mv;

            return soc_table[i].soc_pct_tenths +
                   (uint16_t)((voltage_from_lower * soc_range +
                               voltage_range / 2) / voltage_range);
        }
    }

    return soc_table[0].soc_pct_tenths;
}

void drone_systems_thread(void *p1, void *p2, void *p3)
{

    if (!adc_is_ready_dt(&systems_adc)) {
        LOG_ERR("Drone systems ADC device is not ready");
        return;
    }

    int ret = adc_channel_setup_dt(&systems_adc);
    if (ret) {
        LOG_ERR("Drone systems ADC channel setup failed: %d", ret);
        return;
    }

    struct adc_sequence sequence = {
        .buffer = NULL,
        .buffer_size = 0,
    };

    ret = adc_sequence_init_dt(&systems_adc, &sequence);
    if (ret) {
        LOG_ERR("Drone systems ADC sequence setup failed: %d", ret);
        return;
    }

    int16_t sample;
    int32_t pin_mv;

    int32_t window[MOVING_AVG_WINDOW];
    size_t window_count = 0;
    size_t window_idx = 0;
    int32_t window_sum = 0;

    LOG_INF("Drone systems monitor started at 5 Hz on ADC P0.28 (AIN4)");

    while (1) {
        sample = 0;
        sequence.buffer = &sample;
        sequence.buffer_size = sizeof(sample);

        ret = adc_read_dt(&systems_adc, &sequence);
        if (ret) {
            LOG_ERR("Drone systems ADC read failed: %d", ret);
        } else {
            pin_mv = sample;
            ret = adc_raw_to_millivolts_dt(&systems_adc, &pin_mv);
            if (ret) {
                LOG_ERR("Drone systems ADC millivolt conversion failed: %d", ret);
            } else {
                if (window_count < MOVING_AVG_WINDOW) {
                    window[window_count] = pin_mv;
                    window_sum += pin_mv;
                    window_count++;
                } else {
                    window_sum -= window[window_idx];
                    window[window_idx] = pin_mv;
                    window_sum += pin_mv;
                    window_idx = (window_idx + 1) % MOVING_AVG_WINDOW;
                }

                int32_t avg_pin_mv = window_sum / (int32_t)window_count;
                int32_t battery_mv =
                    (avg_pin_mv * BATTERY_DIVIDER_NUMERATOR +
                     BATTERY_DIVIDER_DENOMINATOR / 2) /
                    BATTERY_DIVIDER_DENOMINATOR;
                uint16_t charge_pct_tenths = get_charge_percentage(battery_mv);

                LOG_DBG("Drone systems ADC P0.28: avg_adc=%d.%03d V, battery=%d.%03d V, charge=%u.%u%%",
                        avg_pin_mv / 1000, avg_pin_mv % 1000,
                        battery_mv / 1000, battery_mv % 1000,
                        charge_pct_tenths / 10, charge_pct_tenths % 10);

                ble_publish_system((uint16_t)battery_mv, 0);
            }
        }

        k_sleep(K_MSEC(DRONE_SYSTEMS_ADC_PERIOD_MS));
    }
}
