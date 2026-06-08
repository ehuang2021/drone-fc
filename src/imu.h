#ifndef IMU_H
#define IMU_H

void mpu_inturrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

#endif