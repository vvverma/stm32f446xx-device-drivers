/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Aug 15, 2023
 *      Author: vvverma
 */
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_rcc_driver.h"



void i2c_peripheral_clk(i2cx_reg_def_t *pi2cx, uint8_t enordi) {

    if (enordi == ENABLE) {
        if (pi2cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pi2cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pi2cx == I2C3) {
            I2C3_PCLK_EN();
        } else {
            //Report error or print something
        }
    } else if (enordi == DISABLE) {
        if (pi2cx == I2C1) {
            I2C1_PCLK_DI();
        } else if (pi2cx == I2C2) {
            I2C2_PCLK_DI();
        } else if (pi2cx == I2C3) {
            I2C3_PCLK_DI();
        } else {
            //Report error or print something
        }
    } else {
        //Report error or print something
    }
}

void i2c_peripheral_control(i2cx_reg_def_t* i2cx, uint8_t status_flag) {
	if(status_flag == ENABLE) {
		i2cx->cr1 |= 1<<I2C_PE_BIT;
	}
	else {
		i2cx->cr1 |= ~(1<<I2C_PE_BIT);
	}

}

void i2c_init(i2c_handle_t *pi2c_handle){

	uint32_t temp = 0;
	uint16_t ccr_calculation = 0;
	uint32_t clock_frequency = 0;
	uint8_t trise_time = 0;

//    5. Enable the peripheral clk
    i2c_peripheral_clk(pi2c_handle->pi2cx, ENABLE);

	pi2c_handle->pi2cx->cr1 |= (1<<15);
	while(check_status_register2(pi2c_handle->pi2cx, I2C_STATUS_BUSY_BIT)!=0);
	pi2c_handle->pi2cx->cr1 &= ~(1<<15);



	//2.Set the Frequency field Standard or fast mode and program in CR2 register
	temp = 0;
	clock_frequency = rcc_get_pclk1_value();

	pi2c_handle->pi2cx->cr2 = ((clock_frequency/1000000)&0x3F);

	//3. Set the Speed Mode Standard Mode or Fast Mode update the CCR register
	temp = 0;
	if(pi2c_handle->i2c_pinconfig.speed_mode <= I2C_SCL_SPEED_SM) {
	  	ccr_calculation = clock_frequency/(2*(pi2c_handle->i2c_pinconfig.speed_mode));
	  	temp |= ccr_calculation&0xFFF;
	  	//clear to standard mode bit
	  	temp&=~(1<<I2C_SPEED_MODE_BIT);
	}
	else {
		if (pi2c_handle->i2c_pinconfig.duty == ENABLE)
		{
		  	ccr_calculation = clock_frequency/(25*pi2c_handle->i2c_pinconfig.speed_mode);
		}else {

		  	ccr_calculation = clock_frequency/(3*pi2c_handle->i2c_pinconfig.speed_mode);
		}
	  	temp|=(ccr_calculation&0xfff);

	  	//set fastmode
	  	temp|=(1<<I2C_SPEED_MODE_BIT);
	}

    pi2c_handle->pi2cx->ccr = temp;

    //4.  Configure the TRISE bit in TRISE
      if(pi2c_handle->i2c_pinconfig.speed_mode <= I2C_SCL_SPEED_SM){
        trise_time = (clock_frequency/1000000)+1; //1000ns == 1000Khz max as per datasheet
      }
      else {
        trise_time = (clock_frequency/3333333)+1; //300ns == 3333333hz max as per datasheet

      }
      pi2c_handle->pi2cx->trise = trise_time;
  	//1. Check if ACK enabled in CR1 register, i think for slave
  	temp = pi2c_handle->pi2cx->cr1;
  	if(pi2c_handle->i2c_pinconfig.ack_en == I2C_ACK_ENABLE)
  	  temp |= 1<<I2C_ACK_BIT;
  	else
  	  temp &= ~(1<<I2C_ACK_BIT);

  	pi2c_handle->pi2cx->cr1 = temp;



}


void i2c_master_send(i2c_handle_t *pi2c_handle, uint8_t buffer, uint8_t slave_addr, uint8_t d) {

	uint32_t temp = 0;
	uint8_t data = 0;

	//while(check_status_register2(pi2c_handle->pi2cx, I2C_STATUS_BUSY_BIT)==1);
	while(pi2c_handle->pi2cx->sr2&2);

	//1. send start bit CR1
	temp = pi2c_handle->pi2cx->cr1;
	temp |= (1<<I2C_START_BIT);//start bit 8th
	//pi2c_handle->pi2cx->cr1 |= (1<<I2C_START_BIT);
	pi2c_handle->pi2cx->cr1 |= temp;

	//2. check SB bit if set then start bit is send use SR1->sb (0th bit)
	//while(!(pi2c_handle->pi2cx->sr1&1));
	while(check_status_register1(pi2c_handle->pi2cx, I2C_STATUS_SB_BIT)!=1);
	//3. if above is true then set slave address in DR register 0x3c + read and write bit
	data = 0;
	data = (slave_addr<<1); // 0 to write and 1 to read
	pi2c_handle->pi2cx->dr = data;

	//4. To clear ADDR read SR1 then SR2
	//while(!(pi2c_handle->pi2cx->sr1&2));
	while(check_status_register1(pi2c_handle->pi2cx, I2C_STATUS_ADDR_BIT)!=1);

	//Temp = pi2c_handle->pi2cx->sr1; // clear addr condition
	// volatile int Temp = pi2c_handle->pi2cx->sr2; // clear addr condition

	check_status_register2(pi2c_handle->pi2cx, I2C_STATUS_MSL_BIT); // check this

	//while(byte_written<len) {
	  //6. set data in DR register
		//5. Check TxE is 1, empty
	    //while(!(pi2c_handle->pi2cx->sr1&0x80));
		while(check_status_register1(pi2c_handle->pi2cx, I2C_STATUS_TxE_BIT)!=1);

			pi2c_handle->pi2cx->dr =  buffer;
		//while(!(pi2c_handle->pi2cx->sr1&0x80));
		while(check_status_register1(pi2c_handle->pi2cx, I2C_STATUS_TxE_BIT)!=1);

		pi2c_handle->pi2cx->dr =  d;
		//while(!(pi2c_handle->pi2cx->sr1&4));

	  //byte_written++;
	//}

	//7. if Txe IS not empty check BTF (this mean slave is processing the data and stretching the clock wait for BTF clear) both BTF and txe
	while(check_status_register1(pi2c_handle->pi2cx, I2C_STATUS_BTF_BIT)!=1);


	//8.Send Stop Bit
	  temp = 0;
	  temp = pi2c_handle->pi2cx->cr1;
	  temp = (1<<I2C_STOP_BIT);// stop bit 9th
	  pi2c_handle->pi2cx->cr1|=temp;

}


uint8_t check_status_register1(i2cx_reg_def_t* pi2cx, uint8_t status_flag) {
  volatile uint32_t temp = 0;
  uint8_t return_value = 0;
  temp = pi2cx->sr1;

  if (temp&(1<<status_flag)) {
    return_value = ENABLE;
  }
  else {
    return_value = DISABLE;
  }
  return return_value;
}


uint8_t check_status_register2(i2cx_reg_def_t* pi2cx, uint8_t status_flag) {
  uint32_t temp = 0;
  uint8_t return_value = 0;
  temp = pi2cx->sr2;

  if (temp&(1<<status_flag)) {
    return_value = ENABLE;
  }
  else{
    return_value = DISABLE;
  }

  return return_value;
}


