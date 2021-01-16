/*
 * stm32f446xx_dma_driver.c
 *
 *  Created on: Oct 17, 2020
 *      Author: vvverma
 */

#include "stm32f446xx_dma_driver.h"


/*typedef struct {
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
} dma_handle_t;*/


void dma_peripheral_clk(dmax_reg_def_t* pdmax, uint8_t enordi) {

    if (enordi == ENABLE) {

        if (pdmax == DMA1) {
            DMA1_PCLK_EN();
        } else if (pdmax == DMA2) {
            DMA2_PCLK_EN();
        } else {
            //Report error or print something
        }

    } else if (enordi == DISABLE) {
        if (pdmax == DMA1) {
            DMA1_PCLK_DI();
        } else if (pdmax == DMA2) {
            DMA2_PCLK_DI();
        } else {
            //Report error or print something
        }
    } else {
        //Report error or print something
    }
}


void dma_init(dma_handle_t* pdma_handle) {

	uint8_t stream_number = pdma_handle->dma_config.stream_number;

	//Step 1) Check if dma Channel is available to use
    while((pdma_handle->pdmax->streams[stream_number].dma_sxcr&(1<<0))) {
    	pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (1<<0);
    }
	//Step 2) Identify the STREAM needed as per peripheral

	//Step 3) Identify the Channel Number needed as per peripheral
    pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (pdma_handle->dma_config.channel_number<<25);

	//Step 4) Program the number of data items to transfer or send // reloads to old value if circular mode
    pdma_handle->pdmax->streams[stream_number].dma_sxndtr |= (pdma_handle->dma_config.data_items);

	//Step 5) Select/Set the direction of data transfer amongst Memory to Peripheral (m2p), Peripheral to Memory (p2m) or Memory to memory (m2m)
    pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (pdma_handle->dma_config.data_transfer_direction<<6);

	//Step 6) Program the Source Address
	//Step 7) Program the Destination Address
    pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (pdma_handle->dma_config.data_transfer_direction<<6);

    if(pdma_handle->dma_config.data_transfer_direction == DMA_TRANSFER_P2M){

    	pdma_handle->pdmax->streams[stream_number].dma_sxpar |= (pdma_handle->dma_config.source_addr);
    	pdma_handle->pdmax->streams[stream_number].dma_sxm0ar |= (pdma_handle->dma_config.destination_addr);

    }
    else if(pdma_handle->dma_config.data_transfer_direction == DMA_TRANSFER_M2P){
    	pdma_handle->pdmax->streams[stream_number].dma_sxpar |= (pdma_handle->dma_config.destination_addr);
    	pdma_handle->pdmax->streams[stream_number].dma_sxm0ar |= (pdma_handle->dma_config.source_addr);
    }
    else if(pdma_handle->dma_config.data_transfer_direction == DMA_TRANSFER_M2M){
    	pdma_handle->pdmax->streams[stream_number].dma_sxpar |= (pdma_handle->dma_config.source_addr);
    	pdma_handle->pdmax->streams[stream_number].dma_sxm0ar |= (pdma_handle->dma_config.destination_addr);
    }

	//Step 8) Program the Source and Destination data width
	pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (pdma_handle->dma_config.per_data_size<<11);
	pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (pdma_handle->dma_config.mem_data_size<<13);

	//Step 9) Select the mode of transfer Direct Mode or FIFO mode
	//Step 10) Select FIFO threshold if we have enabled FIFO mode

	//Step 11) Enable Circular Buffer mode if required

	//Step 12) Select between transfer mode: Single Transfer Mode or Burst Transfer Mode

	//Step 13) Configuire the STREAM Priority
    pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (pdma_handle->dma_config.stream_priority<<16);

	//Step 14) Enable the Stream
    while(!(pdma_handle->pdmax->streams[stream_number].dma_sxcr&(1<<0))) {
    	pdma_handle->pdmax->streams[stream_number].dma_sxcr |= (1<<1);
    }

}

