#include "imu_driver.h"

#include "main.h"
#include "i2c.h"
#include "imu_app.h"

#define IMU_RESET_PULSE_MS      10
#define IMU_BOOT_WAIT_MS        200
#define IMU_SHTP_HEADER_LEN     4
#define IMU_SHTP_LENGTH_MASK    0x7FFF
#define IMU_RX_BUFFER_LEN       128
static uint8_t imu_control_seq = 0;   // per-channel sequence counter SHTP requires on writes
#define IMU_ACCEL_PAYLOAD_LEN   10    // reportID, seq, status, delay, X(2), Y(2), Z(2)


static uint8_t imu_rx_buffer[IMU_RX_BUFFER_LEN];
static uint16_t imu_rx_len;

bool imu_data_ready(void)
{
    return HAL_GPIO_ReadPin(I_INTN_GPIO_Port, I_INTN_Pin) == GPIO_PIN_RESET;
}

void imu_init(void)
{
    HAL_GPIO_WritePin(I_BOOTN_GPIO_Port, I_BOOTN_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(I_NRST_GPIO_Port, I_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(IMU_RESET_PULSE_MS);
    HAL_GPIO_WritePin(I_NRST_GPIO_Port, I_NRST_Pin, GPIO_PIN_SET);

    uint32_t start = HAL_GetTick();
    while (!imu_data_ready()) {
        if ((HAL_GetTick() - start) > IMU_BOOT_WAIT_MS) {
            return;
        }
    }

    uint8_t header[IMU_SHTP_HEADER_LEN];
    if (HAL_I2C_Master_Receive(&hi2c1, IMU_ADDRESS, header, IMU_SHTP_HEADER_LEN, HAL_MAX_DELAY) != HAL_OK) {
        return;
    }

    uint16_t packet_len = ((header[1] << 8) | header[0]) & IMU_SHTP_LENGTH_MASK;
    if (packet_len == 0) {
        return;
    }

    imu_rx_len = (packet_len > IMU_RX_BUFFER_LEN) ? IMU_RX_BUFFER_LEN : packet_len;
    HAL_I2C_Master_Receive(&hi2c1, IMU_ADDRESS, imu_rx_buffer, imu_rx_len, HAL_MAX_DELAY);
}

void imu_enable_accel(void)
{
    uint8_t packet[4 + 17];

    // --- SHTP header (channel 2 = control) ---
    uint16_t total_len = sizeof(packet);
    packet[0] = total_len & 0xFF;
    packet[1] = (total_len >> 8) & 0xFF;
    packet[2] = SHTP_CHANNEL_CONTROL;
    packet[3] = imu_control_seq++;

    // --- SH2 Set Feature Command payload ---
    packet[4]  = SH2_REPORT_SET_FEAATURE_CMD;
    packet[5]  = SH2_REPORT_ACCEL;
    packet[6]  = 0x00;                                   // feature flags
    packet[7]  = 0x00; packet[8]  = 0x00;                // change sensitivity
    packet[9]  = (IMU_ACCEL_REPORT_INTERVAL_US)       & 0xFF;
    packet[10] = (IMU_ACCEL_REPORT_INTERVAL_US >> 8)  & 0xFF;
    packet[11] = (IMU_ACCEL_REPORT_INTERVAL_US >> 16) & 0xFF;
    packet[12] = (IMU_ACCEL_REPORT_INTERVAL_US >> 24) & 0xFF;
    packet[13] = 0x00; packet[14] = 0x00; packet[15] = 0x00; packet[16] = 0x00; // batch interval
    packet[17] = 0x00; packet[18] = 0x00; packet[19] = 0x00; packet[20] = 0x00; // sensor-specific config

    HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDRESS, packet, sizeof(packet), HAL_MAX_DELAY);
}

static bool imu_read_packet(void)
{
    uint8_t header[IMU_SHTP_HEADER_LEN];
    if (HAL_I2C_Master_Receive(&hi2c1, IMU_ADDRESS, header, IMU_SHTP_HEADER_LEN, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }

    uint16_t packet_len = ((header[1] << 8) | header[0]) & IMU_SHTP_LENGTH_MASK;
    if (packet_len == 0) {
        return false;
    }

    imu_rx_len = (packet_len > IMU_RX_BUFFER_LEN) ? IMU_RX_BUFFER_LEN : packet_len;
    return HAL_I2C_Master_Receive(&hi2c1, IMU_ADDRESS, imu_rx_buffer, imu_rx_len, HAL_MAX_DELAY) == HAL_OK;
}

bool imu_read_accel(imu_data_t *data)
{
    if (!imu_data_ready() || !imu_read_packet()) {
        return false;
    }

    if (imu_rx_buffer[2] != SHTP_CHANNEL_REPORTS) {
        return false;
    }

    uint16_t offset = IMU_SHTP_HEADER_LEN;

    // Reports on channel 3 are often preceded by a 5-byte Base Timestamp
    if (offset < imu_rx_len && imu_rx_buffer[offset] == 0xFB) {
        offset += 5;
    }

    if (offset + IMU_ACCEL_PAYLOAD_LEN > imu_rx_len || imu_rx_buffer[offset] != SH2_REPORT_ACCEL) {
        return false;
    }

    /*
    offset+4: X low byte
    offset+5: X high byte
    offset+6: Y low byte
    offset+7: Y high byte
    offset+8: Z low byte
    offset+9: Z high byte
    */
    int16_t x = (int16_t)(imu_rx_buffer[offset + 5] << 8 | imu_rx_buffer[offset + 4]);
    int16_t y = (int16_t)(imu_rx_buffer[offset + 7] << 8 | imu_rx_buffer[offset + 6]);
    int16_t z = (int16_t)(imu_rx_buffer[offset + 9] << 8 | imu_rx_buffer[offset + 8]);

    data->accel_x = (float)x / 256.0f;   // Q8 fixed-point -> m/s^2 (BNO08x accel Q-point = 8)
    data->accel_y = (float)y / 256.0f;
    data->accel_z = (float)z / 256.0f;
    return true;
}