#include "imu_app.h"
#include "imu_driver.h"
static imu_data_t imu_data;

void imu_app_init(void)
{
    imu_init();
    imu_enable_accel();
}

void imu_app_task(void)
{
    if (imu_read_accel(&imu_data)) {
        // imu_data.accel_x / accel_y / accel_z now hold the latest reading in m/s^2
    }

}