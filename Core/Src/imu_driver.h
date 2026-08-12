#ifndef __IMU__DRIVER__H__
#define __IMU__DRIVER__H__

#include <stdbool.h>
#include "imu_app.h"   

/**
 * @brief Reset the BNO086 and read its power-on SHTP advertisement packet.
 */
void imu_init(void);

/**
 * @brief Read the I_INTN pin; the BNO086 drives it low when an SHTP packet is ready.
 * @retval true if a packet is waiting to be read.
 */
bool imu_data_ready(void);

void imu_enable_accel(void);              // sends the SH2 Set Feature command for accel    // reads one SHTP packet, fills accel_x/y/z if it's an accel report
void imu_enable_gyro(void);
void imu_enable_mag(void);
bool imu_read_report(imu_data_t *data);
#endif /* __IMU__DRIVER__H__ */