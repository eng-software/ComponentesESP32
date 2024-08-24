#include "cbspI2C.h"
#include "freertos/FreeRTOS.h"

cbspI2C::cbspI2C()
{
}

cbspI2C::~cbspI2C()
{
}

void cbspI2C::init(i2c_port_t i2c_port, gpio_num_t i2c_gpio_sda, gpio_num_t i2c_gpio_scl)
{
	this->i2c_port		= i2c_port;
	this->i2c_gpio_sda	= i2c_gpio_sda;
	this->i2c_gpio_scl 	= i2c_gpio_scl;
}

void cbspI2C::lock()
{
}

void cbspI2C::unlock()
{
}

bool cbspI2C::openAsMaster(uint32_t baudrate)
{    
	this->i2c_frequency = baudrate;
    i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = this->i2c_gpio_sda,
        .scl_io_num = this->i2c_gpio_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags
        {
            .enable_internal_pullup = true,
        }
    };
    return i2c_new_master_bus(&bus_config, &i2c_bus) == ESP_OK;
}

bool cbspI2C::masterRead(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len )
{

    bool ret = false;
    i2c_master_dev_handle_t devhHandle = NULL;
    i2c_device_config_t devConfig
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = i2c_frequency,
        .scl_wait_us = 0,
        .flags = 0,
    };

    if( i2c_master_bus_add_device(i2c_bus, &devConfig, &devhHandle) == ESP_OK )
    {
        ret = i2c_master_transmit_receive(devhHandle, &dataAddress, 1, ptBuffer, len, 100) == ESP_OK;

        i2c_master_bus_rm_device(devhHandle);
    }

    return ret;
}

bool cbspI2C::masterRead(uint8_t address, uint8_t *ptBuffer, uint32_t len )
{
    bool ret = false;
    i2c_master_dev_handle_t devhHandle = NULL;
    i2c_device_config_t devConfig
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = i2c_frequency,
        .scl_wait_us = 0,
        .flags = 0,
    };

    if( i2c_master_bus_add_device(i2c_bus, &devConfig, &devhHandle) == ESP_OK )
    {
        ret = i2c_master_receive(devhHandle, ptBuffer, len, 100) == ESP_OK;
        i2c_master_bus_rm_device(devhHandle);
    }

    return ret;
}


bool cbspI2C::masterWrite(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len )
{
    bool ret = false;
    i2c_master_dev_handle_t devhHandle = NULL;
    i2c_device_config_t devConfig
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = i2c_frequency,
        .scl_wait_us = 0,
        .flags = 0,
    };

    if( i2c_master_bus_add_device(i2c_bus, &devConfig, &devhHandle) == ESP_OK )
    {
        ret  = i2c_master_transmit(devhHandle, &dataAddress, 1, 100) == ESP_OK;
        ret &= i2c_master_transmit(devhHandle, ptBuffer, len, 100) == ESP_OK;
        i2c_master_bus_rm_device(devhHandle);
    }

    return ret;
}

bool cbspI2C::masterWrite(uint8_t address, uint8_t *ptBuffer, uint32_t len )
{
    bool ret = false;
    i2c_master_dev_handle_t devhHandle = NULL;
    i2c_device_config_t devConfig
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = i2c_frequency,
        .scl_wait_us = 0,
        .flags = 0,
    };

    if( i2c_master_bus_add_device(i2c_bus, &devConfig, &devhHandle) == ESP_OK )
    {
        ret = i2c_master_transmit(devhHandle, ptBuffer, len, 100) == ESP_OK;
        i2c_master_bus_rm_device(devhHandle);
    }

    return ret;
}

