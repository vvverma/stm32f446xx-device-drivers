/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Aug 15, 2023
 *      Author: vvverma
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"



#define I2C_ACK_ENABLE ENABLE
#define I2C_ACK_DISABLE DISABLE


#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000



#define I2C_ACK_BIT 10
#define I2C_SPEED_MODE_BIT 15
#define I2C_PE_BIT 0


typedef struct {
  uint32_t speed_mode;
  uint8_t slave_addr;
  uint8_t duty;
  uint8_t ack_en;
}i2c_pinconfig_t;


typedef struct {
  i2cx_reg_def_t* pi2cx;
  i2c_pinconfig_t  i2c_pinconfig;
}i2c_handle_t;


enum status_register1{
  I2C_STATUS_SB_BIT,
  I2C_STATUS_ADDR_BIT,
  I2C_STATUS_BTF_BIT,
  I2C_STATUS_ADD10_BIT,
  I2C_STATUS_STOPF_BIT,
  I2C_STATUS_RESERVED,
  I2C_STATUS_RxNE_BIT,
  I2C_STATUS_TxE_BIT,
  I2C_STATUS_BERR_BIT,
  I2C_STATUS_ARLO_BIT,
  I2C_STATUS_AF_BIT,
  I2C_STATUS_OVR_BIT,
  I2C_STATUS_PEC_ERR_BIT,
  I2C_STATUS_RESERVED0,
  I2C_STATUS_TIMEOUT_BIT,
  I2C_STATUS_SMB_ALERT_BIT
};

enum status_register2{
  I2C_STATUS_MSL_BIT,
  I2C_STATUS_BUSY_BIT,
  I2C_STATUS_TRA_BIT,
  I2C_STATUS_RESERVED1,
  I2C_STATUS_GEN_CALL_BIT,
  I2C_STATUS_SMB_DEFAULT_BIT,
  I2C_STATUS_SMB_HOST_BIT,
  I2C_STATUS_DUALF_BIT,
  I2C_STATUS_PEC_7_BIT
};



#define I2C_START_BIT 8
#define I2C_STOP_BIT 9


uint8_t check_status_register1(i2cx_reg_def_t* pi2cx, uint8_t status_flag);
uint8_t check_status_register2(i2cx_reg_def_t* pi2cx, uint8_t status_flag);

void i2c_peripheral_control(i2cx_reg_def_t* i2cx, uint8_t status_flag);
void i2c_peripheral_clk(i2cx_reg_def_t *pi2cx, uint8_t enordi);
void i2c_writeByte(char saddr,char maddr,char data);
void i2c_init(i2c_handle_t *pi2c_handle);
void i2c_master_send( i2c_handle_t *pi2c_handle, uint8_t buffer,uint8_t slave_addr, uint8_t d);
//int i2c_master_receive();
//int i2c_slave_send();
// int i2c_slave_receive();


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
