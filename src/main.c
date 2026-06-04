#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "mpu6050.h"

#define I2C_NODE DT_NODELABEL(mysensor)

int main(void)
{
        static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
        struct accelrometer_data acc;
        struct gyro_data gyro;

        while (1) {
                int ret = init_sensor(&dev_i2c);
                if (!ret) {
                        return -1;
                }

                read_acclerometer(&dev_i2c, &acc);
                read_gyro(&dev_i2c, &gyro);

                printk("Accelerometer X: %d, Y: %d, Z: %d", acc.accel_x, acc.accel_y, acc.accel_z);

                k_msleep(100);
                printk("    |||     Gyro X: %d, Y: %d, Z: %d\n", gyro.accel_x, gyro.accel_y, gyro.accel_z);

                k_msleep(1000);
        }

        return 0;
}
