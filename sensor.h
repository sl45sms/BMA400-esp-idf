#include "BMA400-API/bma400.h"

// default and secondary I2C addresses
#define BMA400_I2C_ADDRESS_DEFAULT BMA400_I2C_ADDRESS_SDO_LOW    // 0x14
#define BMA400_I2C_ADDRESS_SECONDARY BMA400_I2C_ADDRESS_SDO_HIGH // 0x15

// Struct to hold data about the I2C interface (or SPI if implemented)
struct BMA400_InterfaceData
{
    // Communication I2C interface (SPI not implemented)
    bma400_intf interface;

    // I2C settings
    uint8_t i2cAddress;

    // SPI settings (not implemented)
    uint8_t spiCSPin;
    uint32_t spiClockFrequency;
};

esp_err_t get_accel_data(struct bma400_sensor_data *accel);