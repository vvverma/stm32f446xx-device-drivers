/*
 * stm32f446xx_dma_driver.h
 *
 *  Created on: Oct 17, 2020
 *      Author: vvverma
 */

#ifndef INC_STM32F446XX_DMA_DRIVER_H_
#define INC_STM32F446XX_DMA_DRIVER_H_
#include "stm32f446xx.h"

typedef struct {
	uint8_t stream_number;
	uint8_t channel_number;
	uint32_t source_addr;
	uint32_t destination_addr;
	uint8_t transfer_mode;
	uint8_t transfer_type;
	uint8_t fifo_threshold;
	uint8_t stream_priority;
	uint8_t mem_data_size;
	uint8_t per_data_size;
	uint16_t data_items;
	uint8_t  data_transfer_direction;
} dma_config_t;


typedef struct {
	dmax_reg_def_t* pdmax;
	dma_config_t dma_config;
} dma_handle_t;

#define DMA_STREAM_0    0
#define DMA_STREAM_1    1
#define DMA_STREAM_2    2
#define DMA_STREAM_3    3
#define DMA_STREAM_4    4
#define DMA_STREAM_5    5
#define DMA_STREAM_6    6
#define DMA_STREAM_7    7

#define DMA_STREAM_PRI_LOW        0
#define DMA_STREAM_PRI_MED        1
#define DMA_STREAM_PRI_HIGH       2
#define DMA_STREAM_PRI_VERY_HIGH  3


#define DMA_CHANNEL_0    0
#define DMA_CHANNEL_1    1
#define DMA_CHANNEL_2    2
#define DMA_CHANNEL_3    3
#define DMA_CHANNEL_4    4
#define DMA_CHANNEL_5    5
#define DMA_CHANNEL_6    6
#define DMA_CHANNEL_7    7

#define DMA_TRANSFER_P2M 0
#define DMA_TRANSFER_M2P 1
#define DMA_TRANSFER_M2M 2


#define DMA_TRANSFER_SINGLE 0
#define DMA_TRANSFER_BURST  1

#define DMA_DIRECT_MODE 0
#define DMA_FIFO_MODE   1

#define DMA_FIFO_THRES_1_4  0
#define DMA_FIFO_THRES_1_2  1
#define DMA_FIFO_THRES_3_4  3
#define DMA_FIFO_THRES_FULL 4


void dma_peripheral_clk(dmax_reg_def_t* pusartx, uint8_t enordi);
void dma_init(dma_handle_t* pusart_handle);

#endif /* INC_STM32F446XX_DMA_DRIVER_H_ */
