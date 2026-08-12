#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ble.h"
#include "pid.h"

#define TELEMETRY_PERIOD_MS 50
#define TELEMETRY_QUEUE_LEN 16

/* Version the notification packet so clients can reject incompatible layouts. */
#define TELEMETRY_VERSION 1U


#define PID_GAIN_MAX_X1000 32000U

LOG_MODULE_REGISTER(bluetooth_task, LOG_LEVEL_DBG);

#define DRONE_SERVICE_UUID_VAL \
    BT_UUID_128_ENCODE(0x4f4e0001, 0x9b3b, 0x4a7d, 0xbf07, 0x1a4e4bdfc001)
#define DRONE_TELEMETRY_UUID_VAL \
    BT_UUID_128_ENCODE(0x4f4e0002, 0x9b3b, 0x4a7d, 0xbf07, 0x1a4e4bdfc001)
#define DRONE_PID_CONFIG_UUID_VAL \
    BT_UUID_128_ENCODE(0x4f4e0003, 0x9b3b, 0x4a7d, 0xbf07, 0x1a4e4bdfc001)


struct __packed telemetry_packet {
    uint8_t version;
    uint8_t flags;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
    int16_t pid_output_x100;
    uint16_t throttle_x10;
    uint16_t battery_mv;
    uint32_t uptime_ms;
};

enum telemetry_update_type {
    TELEMETRY_UPDATE_ATTITUDE,
    TELEMETRY_UPDATE_PID,
    TELEMETRY_UPDATE_ACTUATOR,
    TELEMETRY_UPDATE_SYSTEM,
};

struct telemetry_update {
    enum telemetry_update_type type;
    float roll;
    float pitch;
    float yaw;
    float pid_output;
    float throttle;
    uint16_t battery_mv;
    uint32_t flags;
};

struct telemetry_state {
    float roll;
    float pitch;
    float yaw;
    float pid_output;
    float throttle;
    uint16_t battery_mv;
    uint32_t flags;
};

static const struct bt_uuid_128 service_uuid =
    BT_UUID_INIT_128(DRONE_SERVICE_UUID_VAL);
static const struct bt_uuid_128 telemetry_uuid =
    BT_UUID_INIT_128(DRONE_TELEMETRY_UUID_VAL);
static const struct bt_uuid_128 pid_config_uuid =
    BT_UUID_INIT_128(DRONE_PID_CONFIG_UUID_VAL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, DRONE_SERVICE_UUID_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

K_MSGQ_DEFINE(telemetry_queue, sizeof(struct telemetry_update), TELEMETRY_QUEUE_LEN,
              sizeof(void *));
K_MUTEX_DEFINE(connection_mutex);

static struct telemetry_state telemetry;
static struct bt_conn *current_conn;
static bool notify_enabled;
static const struct bt_gatt_attr *telemetry_attr;

static int16_t angle_to_centideg(float angle)
{
    return (int16_t)(CLAMP(angle, -327.68f, 327.67f) * 100.0f);
}

static int16_t int16_to_le(int16_t value)
{
    return (int16_t)sys_cpu_to_le16((uint16_t)value);
}

static void publish_update(const struct telemetry_update *update)
{
    /* Replace oldest before update */
    if (k_msgq_put(&telemetry_queue, update, K_NO_WAIT) != 0) {
        struct telemetry_update old;

        (void)k_msgq_get(&telemetry_queue, &old, K_NO_WAIT);
        if (k_msgq_put(&telemetry_queue, update, K_NO_WAIT) != 0) {
            LOG_DBG("BLE telemetry queue full");
        }
    }
}

void ble_publish_attitude(float roll, float pitch, float yaw)
{
    struct telemetry_update update = {
        .type = TELEMETRY_UPDATE_ATTITUDE,
        .roll = roll,
        .pitch = pitch,
        .yaw = yaw,
    };

    publish_update(&update);
}

void ble_publish_pid(float output)
{
    struct telemetry_update update = {
        .type = TELEMETRY_UPDATE_PID,
        .pid_output = output,
    };

    publish_update(&update);
}

void ble_publish_actuator(float throttle)
{
    struct telemetry_update update = {
        .type = TELEMETRY_UPDATE_ACTUATOR,
        .throttle = throttle,
    };

    publish_update(&update);
}

void ble_publish_system(uint16_t battery_mv, uint32_t flags)
{
    struct telemetry_update update = {
        .type = TELEMETRY_UPDATE_SYSTEM,
        .battery_mv = battery_mv,
        .flags = flags,
    };

    publish_update(&update);
}

static void handle_update(const struct telemetry_update *update)
{
    switch (update->type) {
    case TELEMETRY_UPDATE_ATTITUDE:
        telemetry.roll = update->roll;
        telemetry.pitch = update->pitch;
        telemetry.yaw = update->yaw;
        break;

    case TELEMETRY_UPDATE_PID:
        telemetry.pid_output = update->pid_output;
        break;

    case TELEMETRY_UPDATE_ACTUATOR:
        telemetry.throttle = update->throttle;
        break;

    case TELEMETRY_UPDATE_SYSTEM:
        telemetry.battery_mv = update->battery_mv;
        telemetry.flags = update->flags;
        break;
    }
}

static void build_packet(struct telemetry_packet *packet)
{
    packet->version = TELEMETRY_VERSION;
    packet->flags = (uint8_t)telemetry.flags;
    packet->roll_cdeg = int16_to_le(angle_to_centideg(telemetry.roll));
    packet->pitch_cdeg = int16_to_le(angle_to_centideg(telemetry.pitch));
    packet->yaw_cdeg = int16_to_le(angle_to_centideg(telemetry.yaw));
    packet->pid_output_x100 = int16_to_le((int16_t)(telemetry.pid_output * 100.0f));
    packet->throttle_x10 = sys_cpu_to_le16((uint16_t)(telemetry.throttle * 10.0f));
    packet->battery_mv = sys_cpu_to_le16(telemetry.battery_mv);
    packet->uptime_ms = sys_cpu_to_le32(k_uptime_get_32());
}

static void send_telemetry(void)
{
    struct telemetry_packet packet;
    struct bt_conn *conn = NULL;

    k_mutex_lock(&connection_mutex, K_FOREVER);
    if (current_conn != NULL && notify_enabled) {
        conn = bt_conn_ref(current_conn);
    }
    k_mutex_unlock(&connection_mutex);

    if (conn == NULL) {
        return;
    }

    build_packet(&packet);

    int ret = bt_gatt_notify(conn, telemetry_attr, &packet, sizeof(packet));
    if (ret != 0) {
        LOG_DBG("Telemetry notify failed: %d", ret);
    }

    bt_conn_unref(conn);
}

static void telemetry_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    k_mutex_lock(&connection_mutex, K_FOREVER);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    k_mutex_unlock(&connection_mutex);
}

static ssize_t write_pid_config(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    const uint8_t *data = buf;
    struct pid_command command;

    ARG_UNUSED(conn);
    ARG_UNUSED(attr);

    if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    /* PID values are scaled by 1000x*/
    if (len != 6) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint16_t kp = sys_get_le16(&data[0]);
    uint16_t ki = sys_get_le16(&data[2]);
    uint16_t kd = sys_get_le16(&data[4]);

    if (kp > PID_GAIN_MAX_X1000 ||
        ki > PID_GAIN_MAX_X1000 ||
        kd > PID_GAIN_MAX_X1000) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    command.type = PID_COMMAND_SET_GAINS;
    command.kp = (float)kp / 1000.0f;
    command.ki = (float)ki / 1000.0f;
    command.kd = (float)kd / 1000.0f;

    if (pid_submit_command(&command) != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
    }

    LOG_INF("PID gains updated over BLE");
    return len;
}

BT_GATT_SERVICE_DEFINE(drone_service, BT_GATT_PRIMARY_SERVICE(&service_uuid), BT_GATT_CHARACTERISTIC(&telemetry_uuid.uuid, BT_GATT_CHRC_NOTIFY, 0, NULL, NULL, NULL),
    BT_GATT_CCC(telemetry_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&pid_config_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_pid_config, NULL),
);

static int start_advertising(void)
{
    return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err != 0) {
        LOG_ERR("BLE connection failed: %u", err);
        return;
    }

    k_mutex_lock(&connection_mutex, K_FOREVER);
    if (current_conn != NULL) {
        bt_conn_unref(current_conn);
    }
    current_conn = bt_conn_ref(conn);
    notify_enabled = false;
    k_mutex_unlock(&connection_mutex);

    LOG_INF("BLE connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    k_mutex_lock(&connection_mutex, K_FOREVER);
    if (current_conn == conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    notify_enabled = false;
    k_mutex_unlock(&connection_mutex);

    LOG_INF("BLE disconnected: %u", reason);
}

static void recycled(void)
{
    int ret = start_advertising();
    if (ret != 0) {
        LOG_ERR("Could not restart advertising: %d", ret);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled,
};

int ble_init(void)
{
    telemetry_attr = &drone_service.attrs[2];

    int ret = bt_enable(NULL);
    if (ret != 0) {
        LOG_ERR("Bluetooth init failed: %d", ret);
        return ret;
    }

    ret = start_advertising();
    if (ret != 0) {
        LOG_ERR("Advertising failed: %d", ret);
        return ret;
    }

    LOG_INF("BLE advertising started");
    return 0;
}

void ble_thread(void *p1, void *p2, void *p3)
{
    struct telemetry_update update;

    while (1) {
        /* Processes old things to from queue before sending telementry*/
        while (k_msgq_get(&telemetry_queue, &update, K_NO_WAIT) == 0) {
            handle_update(&update);
        }

        send_telemetry();
        k_sleep(K_MSEC(TELEMETRY_PERIOD_MS));
    }
}
