#include "imu_app.h"
#include "imu_driver.h"
#include "main.h"
static imu_data_t imu_data;

void imu_app_init(void)
{
    imu_init();
    imu_enable_accel();
    imu_enable_gyro();
}

void imu_app_task(void)
{
    imu_read_report(&imu_data);
}