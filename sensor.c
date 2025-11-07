
#include "sensor.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define BMA400_I2C_ADDRESS 0x14            /*!< slave address for BMA400 sensor */
#define I2C_MASTER_NUM I2C_NUM_0           /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO 30               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 33               /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000          /*!< I2C master clock frequency */

//reference to the sensor
struct bma400_dev sensor;

// Information about I2C interface (or SPI if implemented)
BMA400_InterfaceData interfaceData;

/// @brief Helper function to read sensor registers over I2C
/// @param regAddress Start address to read
/// @param dataBuffer Buffer to store register values
/// @param numBytes Number of bytes to read
/// @param interfaceData Pointer to interface data, see BMA400_InterfaceData
/// @return Error code. 0 means success, negative means failure
esp_err_t readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, BMA400_InterfaceData* interfaceData)
{
    // Jump to desired register address
    //all the folow code in coment is from wire.h so have to change to idf i2c
    /*
    interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);
    interfaceData->i2cPort->write(regAddress);
    if(interfaceData->i2cPort->endTransmission())
    {
        return BMA400_E_COM_FAIL;
    }

    // Read bytes from these registers
    interfaceData->i2cPort->requestFrom(interfaceData->i2cAddress, numBytes);

    // Store all requested bytes
    for(uint32_t i = 0; i < numBytes && interfaceData->i2cPort->available(); i++)
    {
        dataBuffer[i] = interfaceData->i2cPort->read();
    }
*/
    esp_err_t ret = read_bma400(I2C_MASTER_NUM, regAddress, dataBuffer, numBytes);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return BMA400_OK;
}

esp_err_t writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, BMA400_InterfaceData* interfaceData)
{
    // Begin transmission
    interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);

    // Write the address
    interfaceData->i2cPort->write(regAddress);
    
    // Write all the data
    for(uint32_t i = 0; i < numBytes; i++)
    {
        interfaceData->i2cPort->write(dataBuffer[i]);
    }

    // End transmission
    if(interfaceData->i2cPort->endTransmission())
    {
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
   // delay period in microseconds
    vTaskDelay(period / portTICK_PERIOD_MS);
}

esp_err_t init_bma400_i2c(i2c_port_t i2c_num) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMA400_I2C_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    // Add your initialization commands here
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t init_bma400_spi() {
    // Add your initialization commands here
    return ESP_FAIL;
}
//use bma400_soft_reset and bma400_init to initialize the sensor
esp_err_t init_bma400(){
    // Reference to the sensor
    struct bma400_dev sensor;
    BMA400_INTF_RET_TYPE registers;

    // Variable to track errors returned by API calls
    int8_t err = BMA400_OK;
    esp_err_t ret;
    ret = init_bma400_i2c(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set up the interface configuration
    sensor.read = readRegisters();
    sensor.write = writeRegisters();
    sensor.delay_us = usDelay;
    sensor.intf_ptr = &interfaceData;
    
    //Reset the sensor
    err = bma400_soft_reset();
    if(err != BMA400_OK) return err;

    // Initialize the sensor
    err = bma400_init(&sensor);
    if(err != BMA400_OK) return err;

    return setMode(mode);
}



#define BMA400_I2C_ADDRESS 0x14 // replace with your device's I2C address
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */



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
