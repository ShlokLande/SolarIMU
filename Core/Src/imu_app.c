#include "imu_app.h"
#include "imu_driver.h"
#include "main.h"
static imu_data_t imu_data;

void imu_app_init(void)
{
    imu_init();
    imu_enable_accel();
}

void imu_app_task(void)
{
    if (imu_read_accel(&imu_data)) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   // toggles once per successful accel report
    }


}