#include "../bme68x/bme68x_defs.h"
#include "../bsec/bsec_datatypes.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
/*I2C pinout*/
#define SDA_PIN 8
#define SCL_PIN 9

void blink();


/**
 * @brief function to read I2C 
 * 
 * @param reg register to write to
 * @param buf data buffer
 * @param nbytes number of bytes to write
 * @param intf_ptr pointer to store information from the callback
 * @return BME68X_INTF_RET_TYPE 
 * 
 * @retval 0 for Success
 * @retval -1 Failure
 */
BME68X_INTF_RET_TYPE bme_read(uint8_t reg, uint8_t *buf, uint32_t nbytes, void *intf_ptr);

/**
 * @brief function to write I2C 
 * 
 * @param reg register to write to
 * @param buf data buffer
 * @param nbytes number of bytes to write
 * @param intf_ptr pointer to store information from the callback
 * @return BME68X_INTF_RET_TYPE 
 * 
 * @retval 0 for Success
 * @retval -1 Failure
 */
BME68X_INTF_RET_TYPE bme_write(uint8_t reg, const uint8_t *buf, uint32_t nbytes, void *intf_ptr);

/**
 * @brief signals when there's an error
 * 
 * @param rslt code 
 * @param api_name message
 */
void check_rslt_api(int8_t rslt, const char api_name[]);

/**
 * @brief checks the result of the bsec operations
 * 
 * @param rslt code
 * @param api_name message
*/
void check_rslt_bsec(bsec_library_return_t rslt, const char api_name[]);

/**
 * @brief replaces the interface init by the Bosch API
 * 
 * @param bme sensor struct
 * @param intf bme68x_intf enum for SPI or I2C
 * @return int8_t 
 */
int8_t bme_interface_init(struct bme68x_dev *bme, uint8_t intf);

/**
 * @brief user defined function for the delay in sampling
 * 
 * @param period sampling interval in us
 * @param intf_ptr pointer to store information from the callback
 */
void delay_us(uint32_t period, void *intf_ptr);

/**
 * @brief returns the dev_addr
 * 
 * @return dev_addr
*/
uint8_t get_dev_addr();

/**
 * @brief returns the i2c
 * 
 * @return i2c
*/
i2c_inst_t *get_i2c();
