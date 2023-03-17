/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <ring_buffer.h>
#include <debug.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_FRAME_MAX 30
#define FLASH_USER  ADDR_FLASH_PAGE_127 //
#define START_ADDR  FLASH_USER
#define END_ADDR    FLASH_USER + 1024 // 1024 bytes
#define ADDR_FLASH_PAGE_0 ((uint32_t)0x08000000)
#define ADDR_FLASH_PAGE_127_END ((uint32_t )0x0801FFF0)
#define ADDR_FLASH_PAGE_127 ((uint32_t )0x0801FC00)

#define SectorSize			0x1000
#define PageSize			256
#define SectorCount			15 * 16
#define PageCount			(SectorCount * SectorSize) / PageSize
#define BlockSize			SectorSize * 16
#define CapacityInKiloByte	SectorCount * SectorSize / 1024;

#define AIC3204_I2C_ADDRESS (0x18 << 1) // I2C Slave Address

#define LDO_CR 0x02
#define ODPCR 0x09
#define HPL_DGSR 0x10
#define MICBIAS_CR 0x33
#define MIX_AMP_LVCR 0x18
#define HPL_RSR 0x0C
#define MICPGA_PTIRC 0x34
#define MICPGA_VCR 0x3B
#define MICPGA_NTIRC 0x36
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ring_buffer_t rbuf;
HAL_StatusTypeDef rst;

int cnt_num = 0;
int digit = 0;
int check = 0;
int blk_cnt;
int flag = 0;
int sstack = 0;

char rx_frame[RX_FRAME_MAX];
char *frame1, *frame2, *frame3, *frame4, *next_ptr;

uint32_t Temp = 0;
uint8_t mode = 0;
uint8_t mode_key1 = 0;
uint8_t num;
uint8_t indata;
uint8_t rcv_data;
uint8_t rx_buffer[RX_FRAME_MAX];
uint8_t rx_frame_cnt = 0;
uint8_t s_digit_10 = 0;
uint8_t s_digit_1 = 0;
uint8_t digit_10 = 0;
uint8_t digit_1 = 0;
uint8_t hex0 = 0;
uint32_t hex1 = 0;
uint32_t hex2 = 0;
uint8_t hex3 = 0;

uint8_t buff[4];
uint8_t uniqID[8];
uint8_t Stts_rgstr;
uint8_t Source_buf[4];
uint8_t mcubuf[8];
uint8_t flashwt_buf[4];
uint8_t Flash_buf[4096];
uint8_t rw_data[1];

uint32_t min = 1;
uint32_t count;
uint32_t use_console_start, use_console_end;
double use_console_run = 0;
uint32_t flash_add;
uint32_t flash_point;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void FND_CLR();
void printnum(uint8_t num);
static uint32_t StringToHexa(const char *frame2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C_Read(uint8_t id) {
	rst = HAL_I2C_Mem_Read(&hi2c1, AIC3204_I2C_ADDRESS , id, 1, rw_data, 1, 10000);
	if (rst == HAL_OK) PRINTF_DEBUG("AIC RD %x --> (%x)\n",id, *rw_data);
}

void I2C_Write(uint8_t id, uint8_t *dat) { // id : slave register address , 1 : address byte  , dat : buffer address
	rst = HAL_I2C_Mem_Write(&hi2c1, AIC3204_I2C_ADDRESS , id, 1, dat, sizeof(dat), 10000);
	if (rst == HAL_OK) PRINTF_DEBUG("AIC WT %x --> (%x) ", id, *dat);
}

void count_sub(void) {
	count++; // 1ms
}

void fnd_sub(void) {

	if (mode == 2) {
		cnt_num = 0;
		digit ^= 1;
		printnum(cnt_num);
		if (mode_key1 == 1) {
			blk_cnt++;
			if (blk_cnt > 50) {

				if (blk_cnt == 100) {
					blk_cnt = 0;
				}
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
			}
			if (blk_cnt < 50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
			}
		} else if (mode_key1 == 0) {
			FND_CLR();
		}

	}

	else if (mode == 3) {
		cnt_num++;
		if (cnt_num > 9999)
			cnt_num = 0;
		digit ^= 1;
		digit_10 = cnt_num / 1000;
		digit_1 = (cnt_num % 1000) / 100;

		if (mode_key1 == 1) {
			blk_cnt++;
			if (blk_cnt > 50) {

				if (blk_cnt == 100) {
					blk_cnt = 0;
				}

				switch (digit) {
				case 0:
					printnum(digit_10);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
					break;
				case 1:
					printnum(digit_1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
					break;
				}
			}
			if (blk_cnt < 50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
			}
		} else if (mode_key1 == 0) {
			switch (digit) {
			case 0:
				printnum(digit_10);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
				break;
			case 1:
				printnum(digit_1);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
				break;
			}
		}
	}

	else if (mode == 4) {
		digit_10 = cnt_num / 1000;
		digit_1 = (cnt_num % 1000) / 100;
		digit ^= 1;

		if (mode_key1 == 1) {
			blk_cnt++;
			if (blk_cnt > 50) {
				if (blk_cnt == 100) {
					blk_cnt = 0;
				}

				switch (digit) {
				case 0:
					printnum(digit_10);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
					break;
				case 1:
					printnum(digit_1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
					break;
				}
			}
			if (blk_cnt < 50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
			}
		} else if (mode_key1 == 0) {
			switch (digit) {
			case 0:
				printnum(digit_10);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
				break;
			case 1:
				printnum(digit_1);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
				break;
			}
		}

	}
}

void FND_CLR() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2

}

void printnum(uint8_t num) {
	switch (num) {
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 9:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	}
}


void use_console() {
	if (ring_buf_pop(&rbuf, &indata)) {
		if (indata == '\r' || indata == '\n') {
			use_console_start = (TIM3->CNT) + count * 1000; //start count

			rx_frame[rx_frame_cnt] = 0;
			frame1 = strtok_r(rx_frame, " ", &next_ptr);
			frame2 = strtok_r(NULL, " ", &next_ptr);
			frame3 = strtok_r(NULL, " ", &next_ptr);
			frame4 = strtok_r(NULL, " ", &next_ptr);

			if (!strcmp(frame1, "START") || !strcmp(frame1, "start")) {
				if (mode == 4) {
					mode = 3;
					PRINTF_DEBUG(">>>> START\r\n");
				} else if (mode == 3) {

				} else {
					mode = 3;
					cnt_num = 0;
					PRINTF_DEBUG(">>>> START\r\n");
				}

			} else if (!strcmp(frame1, "CLR") || !strcmp(frame1, "clr")) {
				mode = 2;
				cnt_num = 0;
				PRINTF_DEBUG(">>>> CLEAR\r\n");
			} else if (!strcmp(frame1, "STOP") || !strcmp(frame1, "stop")) {
				if (mode == 3) {
					mode = 4;
					PRINTF_DEBUG(">>>> STOP\r\n");
				} else if (mode == 4) {

				} else if (mode == 2) {

				}
			} else if (!strcmp(frame1, "MEMRD") || !strcmp(frame1, "memrd")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					if (hex1 >= ADDR_FLASH_PAGE_0 //&& hex1 <= ADDR_FLASH_PAGE_127_END
					) {
						PRINTF_DEBUG("read value : 0x%02x",
								(*(uint32_t*) hex1 & 0xFF000000) >> 24);
						PRINTF_DEBUG(" 0x%02x",
								(*(uint32_t*) hex1 & 0xFF0000) >> 16);
						PRINTF_DEBUG(" 0x%02x",
								(*(uint32_t*) hex1 & 0xFF00) >> 8);
						PRINTF_DEBUG(" 0x%02x\r\n", *(uint32_t*) hex1 & 0xFF);
					} else {
						PRINTF_DEBUG("Segmentation Fault\r\n\r\n");
					}
				}
			}
			/*
			else if (!strcmp(frame1, "MEMWT") || !strcmp(frame1, "memwt")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Hex-Value : %s", frame3);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					hex2 = StringToHexa(frame3);
					if (hex1 >= ADDR_FLASH_PAGE_0 //&& hex1 <= ADDR_FLASH_PAGE_127_END
					) {
						save_flash(hex1, hex2);
					}
				}
			}

			else if (!strcmp(frame1, "flashwt")
					|| !strcmp(frame1, "FLASHWT")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Value : %s", frame3);
					PRINTF_DEBUG("\r\n");
					hex2 = StringToHexa(frame2); // 32bit address
					flashwt_buf[0] = StringToHexa(frame3); // 8bit value

					flash_write_byte(hex2);
					flash_read(hex2);
				}
			} else if (!strcmp(frame1, "flashrd")
					|| !strcmp(frame1, "FLASHRD")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					hex2 = StringToHexa(frame2);
					flash_read(hex2);
				}
			} else if (!strcmp(frame1, "flash9fh")
					|| !strcmp(frame1, "FLASH9FH")) {
				PRINTF_DEBUG("Command : %s", frame1);
				PRINTF_DEBUG("\r\n");
				Temp = flash_read_ID();
				PRINTF_DEBUG("JEDEC ID : %X\r\n", Temp);

			} else if (!strcmp(frame1, "flash4bh")
					|| !strcmp(frame1, "FLASH4BH")) {
				PRINTF_DEBUG("Command : %s", frame1);
				PRINTF_DEBUG("\r\n");
				flash_read_uniqID();

			} else if (!strcmp(frame1, "flashrdrg")
					|| !strcmp(frame1, "FLASHRDRG")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Rgst : %s", frame2);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					hex2 = StringToHexa(frame3);
					if (!strcmp(frame2, "1") || !strcmp(frame2, "1")) {
						Stts_rgstr = flash_read_Status(1);
						PRINTF_DEBUG("sr1 : %X\r\n", Stts_rgstr);
					} else if (!strcmp(frame2, "2") || !strcmp(frame2, "2")) {
						Stts_rgstr = flash_read_Status(2);
						PRINTF_DEBUG("sr2 : %X\r\n", Stts_rgstr);
					} else if (!strcmp(frame2, "3") || !strcmp(frame2, "3")) {
						Stts_rgstr = flash_read_Status(3);
						PRINTF_DEBUG("sr3 : %X\r\n", Stts_rgstr);
					}
				}
			} else if (!strcmp(frame1, "flashcp")
					|| !strcmp(frame1, "FLASHCP")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Source Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Flash Address : %s", frame3);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					hex2 = StringToHexa(frame3);

					Source_buf[0] = ((*(uint32_t*) hex1 & 0xFF000000) >> 24);
					Source_buf[1] = ((*(uint32_t*) hex1 & 0xFF0000) >> 16);
					Source_buf[2] = ((*(uint32_t*) hex1 & 0xFF00) >> 8);
					Source_buf[3] = ((*(uint32_t*) hex1 & 0xFF));
					flash_point = (hex2 & 0x0000FFFF);

					flash_add = GetSector(hex2); //sector explorer
					sector_backup(flash_add); // sector backup

					for (int i = 0; i < 4; i++) {
						if (flash_point >= 4096) {
							flash_point = flash_point % 4096;
						}
						Flash_buf[flash_point + i] = Source_buf[i];
					}

					flash_erase_sector(flash_add);
					for (flag = 0; flag < 16; flag++) {
						sector_write_byte(flash_add + flag * 256);
					}

					//+ ram memory and flash memory Comparing
					compare_flash_read(hex2);
					for (int i = 0; i < 4; i++) {
						if (Source_buf[i] == buff[i]) {
							sstack++;
						}
					}
					if (sstack == 4) {
						PRINTF_DEBUG("\r\nRead MCU(%p) : 0x%02X", hex1,	(*(uint32_t*) hex1 & 0xFF000000) >> 24);
						PRINTF_DEBUG(" 0x%02X",	(*(uint32_t*) hex1 & 0xFF0000) >> 16);
						PRINTF_DEBUG(" 0x%02X",	(*(uint32_t*) hex1 & 0xFF00) >> 8);
						PRINTF_DEBUG(" 0x%02X", *(uint32_t*) hex1 & 0xFF);
						flash_read(hex2);
						PRINTF_DEBUG("Comparisons correct\r\n");
						sstack = 0;
					} else {
						PRINTF_DEBUG("Comparisons not correct\r\n");
						sstack = 0;
					}
				}
			} else if (!strcmp(frame1, "flashclr") || !strcmp(frame1, "FLASHCLR")) {
				flash_erase_chip();
				PRINTF_DEBUG("---Chip Erase---");
			} */

			else if (!strcmp(frame1, "aic") || !strcmp(frame1, "aic")) {
				if ((frame2 != NULL) && (!strcmp(frame2, "wt") || !strcmp(frame2,"WT"))) {
					hex0 = StringToHexa(frame3);
					hex1 = StringToHexa(frame4);
					I2C_Write(hex0, (uint8_t*)hex1);
				}
				else if ((frame2 != NULL) && (!strcmp(frame2, "rd") || !strcmp(frame2,"RD"))) {
					hex0 = StringToHexa(frame3);
					I2C_Read(hex0);
				}
			} else if (!strcmp(frame1, "micon") || !strcmp(frame1, "MICON")){
					mcubuf[0] = 0x00;
					I2C_Write(0x00,mcubuf);  // Select page 0
					I2C_Read(0x00);

					mcubuf[0] = 0x01;
					I2C_Write(0x01,mcubuf); // reset page 1
					I2C_Read(0x01);
					HAL_Delay(500);

					mcubuf[0] = 0x01;
					I2C_Write(0x00,mcubuf); // Select page 1
					I2C_Read(0x00);

					mcubuf[0] = 0x01;
					I2C_Write(LDO_CR,mcubuf); // Enable Analog Blocks, use LDO power , 0x02
					I2C_Read(LDO_CR);

					mcubuf[0] = 0x50;
					I2C_Write(MICBIAS_CR,mcubuf); // HPL, MAL powered up , 0x33
					I2C_Read(MICBIAS_CR);

					mcubuf[0] = 0x22;
					I2C_Write(ODPCR,mcubuf); // HPL, MAL powered up , 0x09
					I2C_Read(ODPCR);

					mcubuf[0] = 0x10;
					I2C_Write(MICPGA_PTIRC,mcubuf); // IN2L is routed to Left MICPGA , 0x34
					I2C_Read(MICPGA_PTIRC);

					mcubuf[0] = 0X20;
					I2C_Write(MICPGA_VCR,mcubuf);  // MIC_PGA_L unmute ( 10dB ) , 0x3B
					I2C_Read(MICPGA_VCR);

					mcubuf[0] = 0x00;
					I2C_Write(MIX_AMP_LVCR,mcubuf); // volume control register ( 0dB ) , 0x18
					I2C_Read(MIX_AMP_LVCR);

					mcubuf[0] = 0x02;
					I2C_Write(HPL_RSR,mcubuf); // IN1L is routed to HPL, 0x0C
					I2C_Read(HPL_RSR);

					mcubuf[0] = 0x00;
					I2C_Write(HPL_DGSR,mcubuf); // HPL driver gain setting register  (HPL is not mute, driver gain 0dB ), 0x10
					I2C_Read(HPL_DGSR);

					mcubuf[0] = 0x40;
					I2C_Write(MICPGA_NTIRC,mcubuf); // IN1L is routed to HPL , 0x36
					I2C_Read(MICPGA_NTIRC);

			} else if (!strcmp(frame1, "i2cwt") || !strcmp(frame1, "I2CWT")){
				if ((frame2 != NULL ) && (!strcmp(frame2, "HPL_RSR") ||(!strcmp(frame2, "hpl_rsr")))) {
					hex3 = StringToHexa(frame3);
				}
			}
			buf_clear(&rbuf);
			ring_buf_clr(&rbuf);
			memset(rx_frame, 0, RX_FRAME_MAX);
			rx_frame_cnt = 0;
			use_console_end = (TIM3->CNT) + count * 1000; //end count
			use_console_run = (double) (use_console_end - use_console_start);
			PRINTF_DEBUG("\r\nuse_console running time : %dus", use_console_run);
			PRINTF_DEBUG("\r\n\r\n");
			use_console_start = 0, use_console_end = 0, use_console_run = 0;
		} else {

			if (rx_frame_cnt == 0) {
			}
			rx_frame[rx_frame_cnt++] = indata;
			if (rx_frame_cnt >= RX_FRAME_MAX) {
				buf_clear(&rbuf);
				memset(rx_frame, 0, RX_FRAME_MAX);
				rx_frame_cnt = 0;
			}
		}
	}
}

uint8_t read_pin() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == '\0') {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) != '\0') {
			mode_key1 ^= 1;
			PRINTF_DEBUG(">>>> SELECT MODE 1\r\n");
		}
		return 0;
	} else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == '\0') {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) != '\0') {
			mode = 2;
			PRINTF_DEBUG(">>>> SELECT MODE 2\r\n");
		}
		return 0;
	} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == '\0' && check == 0) {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != '\0') {
			if (mode == 4) {
				mode = 3;
				PRINTF_DEBUG(">>>> SELECT MODE 3\r\n");
			} else if (mode == 3) {
				mode = 4;
				PRINTF_DEBUG(">>>> SELECT MODE 4\r\n");
			} else {
				mode = 3;
				cnt_num = 0;
				PRINTF_DEBUG(">>>> SELECT MODE 3\r\n");
			}
			return 0;
		}
	}
	return 1;
}

static uint32_t StringToHexa(const char *frame2) {
	uint32_t hex = 0;
	int count = strlen(frame2), i = 0;

	for (i = 0; i < count; i++) {
		if (*frame2 >= '0' && *frame2 <= '9')
			hex = hex * 16 + *frame2 - '0';
		else if (*frame2 >= 'A' && *frame2 <= 'F')
			hex = hex * 16 + *frame2 - 'A' + 10;
		else if (*frame2 >= 'a' && *frame2 <= 'f')
			hex = hex * 16 + *frame2 - 'a' + 10;
		frame2++;
	}

	return hex;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		ring_buf_push(&rbuf, rcv_data);
		HAL_UART_Transmit(&huart2, &rcv_data, 1, 100);
		HAL_UART_Receive_IT(&huart2, &rcv_data, 1);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_I2S3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  ring_buf_init(&rbuf, rx_buffer, RX_FRAME_MAX);
  ring_buf_clr(&rbuf);
  HAL_UART_Receive_IT(&huart2, &rcv_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		use_console();
		if (read_pin() == '\0') {
			read_pin();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_SLAVE_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 480-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FLASH_SS_Pin|FND_COM1_Pin|FND_COM2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_A_Pin|SEG_B_Pin|SEG_F_Pin|SEG_G_Pin
                          |SEG_DP_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BT_2_Pin */
  GPIO_InitStruct.Pin = BT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_1_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_SS_Pin FND_COM1_Pin FND_COM2_Pin */
  GPIO_InitStruct.Pin = FLASH_SS_Pin|FND_COM1_Pin|FND_COM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_F_Pin SEG_G_Pin
                           SEG_DP_Pin SEG_C_Pin SEG_D_Pin SEG_E_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_F_Pin|SEG_G_Pin
                          |SEG_DP_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_2_Pin */
  GPIO_InitStruct.Pin = KEY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
