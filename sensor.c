
#include "sensor.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "rom/ets_sys.h"

#define BMA400_I2C_ADDRESS 0x14            /*!< slave address for BMA400 sensor */
#define I2C_MASTER_NUM I2C_NUM_0           /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO 30               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 33               /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000          /*!< I2C master clock frequency */

//reference to the sensor
struct bma400_dev sensor;

// Information about I2C interface (or SPI if implemented)
BMA400_InterfaceData interfaceData;

BMA400_INTF_RET_TYPE readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfaceData)
{
    esp_err_t ret = read_bma400(I2C_MASTER_NUM, regAddress, dataBuffer, numBytes);
    if (ret != ESP_OK) {
        return BMA400_E_COM_FAIL;
    }
    
    return BMA400_OK;
}

BMA400_INTF_RET_TYPE writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfaceData)
{
    esp_err_t ret = write_bma400(I2C_MASTER_NUM, regAddress, (uint8_t*)dataBuffer, numBytes);
    if (ret != ESP_OK) {
        return BMA400_E_COM_FAIL;
    }

    return BMA400_OK;
}

int8_t setMode(uint8_t mode)
{
    return bma400_set_power_mode(mode, &sensor);
}

void usDelay(uint32_t period, void* interfacePtr)
{
    ets_delay_us(period);
}

esp_err_t init_bma400(){
    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // Set up the interface configuration
    sensor.intf = BMA400_I2C_INTF;
    sensor.read = readRegisters;
    sensor.write = writeRegisters;
    sensor.delay_us = usDelay;
    sensor.intf_ptr = &interfaceData;
    
    //Reset the sensor
    err = bma400_soft_reset(&sensor);
    if(err != BMA400_OK) return err;

    // Initialize the sensor
    err = bma400_init(&sensor);
    if(err != BMA400_OK) return err;

    return ESP_OK;
}

esp_err_t get_accel_data(struct bma400_sensor_data *accel)
{
    int8_t err = BMA400_OK;
    err = bma400_get_accel_data(BMA400_DATA_ONLY, accel, &sensor);
    if (err != BMA400_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

#define BMA400_I2C_ADDRESS 0x14 // replace with your device's I2C address
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

esp_err_t write_bma400(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_wr, size_t size) {
    if (data_wr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMA400_I2C_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t read_bma400(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_rd, size_t size) {
    if (data_rd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMA400_I2C_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMA400_I2C_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
