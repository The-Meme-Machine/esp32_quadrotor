#include <IMU.h>

static const char *TAG = "I2C";

// Flag for preventing i2c access conflicts
volatile bool i2c_busy_flag = false;

i2c_master_bus_config_t i2c_mst_config;
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t IMU_handle;
i2c_master_dev_handle_t mag_handle;

i2c_device_config_t IMU_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = IMU_ADDR,
    // .scl_speed_hz = 100000, // 100kHz standard mode
    // .scl_speed_hz = 400000, // 400kHz fast mode
    .scl_speed_hz = 1000000, // 1000kHz fast+ mode
    // Use fast+ if possible due to the enormous benefit to loop speed
};
i2c_device_config_t mag_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MAG_ADDR,
    // .scl_speed_hz = 100000, // 100kHz standard mode
    // .scl_speed_hz = 400000, // 400kHz fast mode
    .scl_speed_hz = 1000000, // 1000kHz fast+ mode

};

IMU_packet IMU_data;
mag_packet mag_data;

void setup_imu_mag(gpio_num_t sda, gpio_num_t slc)
{
    ESP_LOGI(TAG, "Setting up I2C devices...");

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = slc,
        .sda_io_num = sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &IMU_cfg, &IMU_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mag_cfg, &mag_handle));

    // Delay to avoid master bus not ready
    vTaskDelay(pdMS_TO_TICKS(300));

    // Check if the IMU and Mag is connected
    esp_err_t err = i2c_master_probe(bus_handle, IMU_ADDR, -1);
    if (err == 0)
    {
        // Check the WHOAMI registers
        uint8_t whoami_addr = 0x0F;
        uint8_t IMU_whoami_reg;
        uint8_t MAG_whoami_reg;
        i2c_master_transmit_receive(IMU_handle, &whoami_addr, 1, &IMU_whoami_reg, 1, -1);
        i2c_master_transmit_receive(mag_handle, &whoami_addr, 1, &MAG_whoami_reg, 1, -1);

        if (IMU_whoami_reg == IMU_WHOAMI && MAG_whoami_reg == MAG_WHOAMI)
        {
            ESP_LOGI(TAG, "IMU + Mag WHOAMI check success.");
        }
        else
        {
            ESP_LOGE(TAG, "IMU + Mag WHOAMI check failed. Recieved: IMU %x, MAG %x", IMU_whoami_reg, MAG_whoami_reg);
            // Perhaps throw hard error here
        }
    }
    else
    {
        ESP_LOGE(TAG, "IMU + Mag not connected.");
        ESP_ERROR_CHECK(err);
    }

    /* IMU SETUP */

    // COUNTER_BDR_REG1 ->
    // Enable pulsed data-ready mode
    write_to_buf(IMU_handle, 0x0b, 0b10000000);

    // INT1_CTRL
    // Enable interrupt on INT1 on gyro data ready
    // uint8_t int1_buf[2] = {0x0d, 0b00000010};
    // i2c_master_transmit(IMU_handle, &int1_buf, 2, -1);
    write_to_buf(IMU_handle, 0x0d, 0b00000010);

    // INT2_CTRL
    // Enable interrupt on INT2 on accel data ready

    // CTRL1_XL register ->
    // Set linear acceleration to 4g limit (FS_XL)
    // Set output rate to 1.66kHz (ODR_XL)
    // first stage digital filtering
    // write_to_buf(IMU_handle, 0x10, 0b10001010); // 0b10001000 for no filtering
    write_to_buf(IMU_handle, 0x10, 0b01111010); // for 833Hz

    // CTRL2_G register ->
    // Set output rate to 1.66kHz
    // Set limit to 500dps
    // write_to_buf(IMU_handle, 0x11, 0b10000100);
    write_to_buf(IMU_handle, 0x11, 0b01110100);

    // CTRL3_C register ->
    // Enable block data update
    // Interrupt pins active high
    write_to_buf(IMU_handle, 0x12, 0b01000100);

    // CTRL4_C register ->
    // Make all interrupts available on the INT1 pin
    // Mask DRDY on pin until filters settle
    write_to_buf(IMU_handle, 0x13, 0b00101010); // 0b00101010 for no filtering

    // TODO: CTRL7-G and CTRL8-XL may be necessary for filtering
    write_to_buf(IMU_handle, 0x15, 0b00000100); // low pass filter freq
    // write_to_buf(IMU_handle, 0x16, ) // high pass filter

    // TODO: Offset values for accel and gyro

    /* MAG SETUP */

    // TODO: Hard iron offset calibration here

    // CTRL_REG1
    // Enable temp sensor
    // Set output data rate to 5Hz
    write_to_buf(mag_handle, 0x20, 0b10001100);

    // INT_CFG
    // Enable interrupt on INTM pin
    write_to_buf(mag_handle, 0x30, 0b00001001);

    /* BAROMETER SETUP */
    // WIP
}

void write_to_buf(i2c_master_dev_handle_t dev, uint8_t addr, uint8_t val)
{
    // Write to buffer
    uint8_t buf[2] = {addr, val};
    i2c_master_transmit(dev, buf, 2, -1);

    // Readback from buffer
    uint8_t readback;
    i2c_master_transmit_receive(dev, &buf[0], 1, &readback, 1, -1);

    if (readback == val)
    {
        ESP_LOGI(TAG, "Buffer at addr 0x%x set correctly", addr);
    }
    else
    {
        ESP_LOGE(TAG, "Readback error: Buffer at addr 0x%x is incorrect.", addr);
        // Perhaps add a hard error here
    }
}

int16_t read_twos_comp_buf(i2c_master_dev_handle_t dev, uint8_t addr)
{
    uint8_t reg_addr = addr;
    uint8_t out_buf[2] = {0};
    i2c_master_transmit_receive(dev, &reg_addr, 1, out_buf, 2, -1);
    // Two's complement binary format
    int16_t out = (int16_t)((out_buf[1] << 8) | out_buf[0]);

    return out;
}

IMU_packet *read_IMU()
{
    if (!i2c_busy_flag)
    {
        // i2c_busy_flag = true;

        uint8_t out_buf[12];
        uint8_t addr = 0x22;
        i2c_master_transmit_receive(IMU_handle, &addr, 1, out_buf, 12, -1);

        // i2c_busy_flag = false;

        IMU_data.g_x = (int16_t)((out_buf[1] << 8) | out_buf[0]);
        IMU_data.g_y = (int16_t)((out_buf[3] << 8) | out_buf[2]);
        IMU_data.g_z = (int16_t)((out_buf[5] << 8) | out_buf[4]);
        IMU_data.xl_x = (int16_t)((out_buf[7] << 8) | out_buf[6]);
        IMU_data.xl_y = (int16_t)((out_buf[9] << 8) | out_buf[8]);
        IMU_data.xl_z = (int16_t)((out_buf[11] << 8) | out_buf[10]);

        return &IMU_data;
    }
    else
    {
        // if i2c is busy, return stale data
        return &IMU_data;
    }
}

mag_packet *read_mag()
{
    if (!i2c_busy_flag)
    {
        i2c_busy_flag = true;

        uint8_t out_buf[6];
        uint8_t addr = 0x28;
        i2c_master_transmit_receive(mag_handle, &addr, 1, out_buf, 6, -1);

        i2c_busy_flag = false;

        mag_data.m_x = (int16_t)((out_buf[1] << 8) | out_buf[0]);
        mag_data.m_y = (int16_t)((out_buf[3] << 8) | out_buf[2]);
        mag_data.m_z = (int16_t)((out_buf[5] << 8) | out_buf[4]);

        return &mag_data;
    }
    else
    {
        // if i2c is busy, return stale data
        return &mag_data;
    }
}

int16_t read_temp()
{
    static int16_t data;

    if (!i2c_busy_flag)
    {
        i2c_busy_flag = true;

        uint8_t out_buf[2];
        uint8_t addr = 0x2e;
        i2c_master_transmit_receive(mag_handle, &addr, 1, out_buf, 12, -1);

        i2c_busy_flag = false;

        data = (int16_t)((out_buf[1] << 8) | out_buf[0]);
        return data;
    }
    else
    {
        // if i2c is busy, return stale data
        return data;
    }
}