/*
 * w25q_spi.h
 *
 *  Created on: Mar 18, 2025
 *      Author: m.archipov
 */

#ifndef SRC_W25Q_SPI_H_
#define SRC_W25Q_SPI_H_
//-------------------------------------------------------------
#include "stm32f1xx_hal.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>
//-------------------------------------------------------------
void SPI1_Send (uint8_t *dt, uint16_t cnt);
void SPI1_Recv (uint8_t *dt, uint16_t cnt);
void W25_Reset (void);
void W25_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz);
uint32_t W25_Read_ID(void);
void W25_Ini(void);
//-------------------------------------------------------------


#endif /* SRC_W25Q_SPI_H_ */
