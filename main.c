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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ring_buffer.h>
#include <debug.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//
//
//Uart define
#define RX_FRAME_MAX 30

//SPI define
#define SectorSize			0x1000
#define PageSize			256
#define SectorCount			15 * 16
#define PageCount			(SectorCount * SectorSize) / PageSize
#define BlockSize			SectorSize * 16
#define CapacityInKiloByte	SectorCount * SectorSize / 1024;
#define ADDR_FLASH_PAGE_0 ((uint32_t)0x08000000)
#define ADDR_FLASH_PAGE_127_END ((uint32_t )0x0801FFF0)
#define ADDR_FLASH_PAGE_127 ((uint32_t )0x0801FC00)
#define FLASH_USER  ADDR_FLASH_PAGE_127 //
#define START_ADDR  FLASH_USER
#define END_ADDR    FLASH_USER + 1024 // 1024 bytes

//I2C, I2S define
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
#define AUDIO_SAMPLE_RATE I2S_AUDIOFREQ_16K

//DMA define
#define AUDIO_BUFFER_SIZE DMA_BUFFER_SIZE*2
#define DMA_BUFFER_SIZE 1024

//Temp(sin) define
#define PI 3.14159f
#define F_SAMPLE		48000.0f
#define F_OUT				1500.0f

#define SAMPLE_FREQUENCY 32000.0f

typedef signed char int8;
typedef unsigned char uint8;

typedef signed short int16;
typedef unsigned short uint16;

typedef signed long int32;
typedef unsigned long uint32;

typedef signed long bool;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;

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

uint8_t p_num = 0xFF;
uint8 rx_flag = 0;
uint8 tx_flag = 0;
uint8 dma_flag = 0;
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
uint8_t hex3 = 0;
uint8_t buff[4];
uint8_t uniqID[8];
uint8_t Stts_rgstr;
uint8_t Source_buf[4];
uint8_t mcubuf[8];
uint8_t flashwt_buf[4];
uint8_t Flash_buf[4096];
uint8_t rw_data[1];
uint8_t wt_data[1];

int16_t audio_buffer[AUDIO_BUFFER_SIZE] = { 0, };
int16_t dma_transmit_buffer[DMA_BUFFER_SIZE] = { 0, };
int16_t dma_receive_buffer[DMA_BUFFER_SIZE] = { 0, };

uint32_t min = 1;
uint32_t count;
uint32_t use_console_start, use_console_end;
uint32_t use_console_run = 0;
uint32_t flash_add;
uint32_t flash_point;
uint32_t hex1 = 0;
uint32_t hex2 = 0;
uint32_t Temp = 0;

//Temp(sin) pv
float mySinVal;
float sample_dt = F_OUT / SAMPLE_FREQUENCY;
uint16 sample_N = SAMPLE_FREQUENCY / F_OUT;
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
void AIC_init(void);
void AIC_off(void);
void Analog_Bypass(void);
void FND_CLR(void);
void printnum(uint8_t num);
static uint32_t StringToHexa(const char *frame2);
void I2C_Read(uint8_t id);
void I2C_Write(uint8_t id, uint8_t dat);
void process_audio_transmit_data(int16_t *audio_buffer, uint16_t size);
void process_audio_receive_data(int16_t *dma_receive_buffer, uint16 size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef aic3204_write(uint8 addr, uint8 dat) {
	uint8_t tmp[2];
	HAL_StatusTypeDef res;

	tmp[0] = dat;
	res = HAL_I2C_Mem_Write(&hi2c1, AIC3204_I2C_ADDRESS, (uint16_t) addr, I2C_MEMADD_SIZE_8BIT, tmp, 1, 100);

	return (res);
}

HAL_StatusTypeDef aic3204_read(uint8 addr, uint8 *dat) {
	HAL_StatusTypeDef res;
	res = HAL_I2C_Mem_Read(&hi2c1, AIC3204_I2C_ADDRESS, (uint16_t) addr, I2C_MEMADD_SIZE_8BIT, dat, 1, 100);
	return (res);
}

void I2C_Read(uint8_t id) {

	rst = HAL_I2C_Mem_Read(&hi2c1, AIC3204_I2C_ADDRESS, id, 1, rw_data, 1, 10000);
	if (rst == HAL_OK)
		PRINTF_DEBUG("AIC RD %x --> (%x)\n", id, *rw_data);
}

void I2C_Write(uint8_t id, uint8_t dat) { // id : slave register address , 1 : address byte  , dat : buffer address
	uint8_t tmp[2];
	tmp[0] = dat;
	rst = HAL_I2C_Mem_Write(&hi2c1, AIC3204_I2C_ADDRESS, id, 1, tmp, 1, 10000);
	if (rst == HAL_OK)
		PRINTF_DEBUG("AIC WT %x --> (%x)\r\n", id, dat);
}

void AIC3204_rset(uint8_t id, uint8_t dat) {
	wt_data[0] = dat;
	rst = HAL_I2C_Mem_Write(&hi2c1, AIC3204_I2C_ADDRESS, id, 1, wt_data, sizeof(wt_data), 10000);
}

void process_audio_transmit_data(int16 *audio_buffer, uint16_t size) {
	if (tx_flag == 0) {
		if (size == DMA_BUFFER_SIZE / 2) { //RxHalfCpltCallback
			for (int i = 0; i < DMA_BUFFER_SIZE / 2; i++) {
				dma_transmit_buffer[i] = audio_buffer[i];
			}

		} else { //HAL_I2S_RxCpltCallback
			for (int i = DMA_BUFFER_SIZE / 2; i < DMA_BUFFER_SIZE; i++) {
				dma_transmit_buffer[i] = audio_buffer[i];
			}
			tx_flag ^= 1;
		}

	} else if (tx_flag != 0) {
		if (size == DMA_BUFFER_SIZE / 2) { //RxHalfCpltCallback , tx_flag = 1
			for (int i = 0; i < DMA_BUFFER_SIZE / 2; i++) {
				dma_transmit_buffer[i] = audio_buffer[i + tx_flag * DMA_BUFFER_SIZE];
			}

		} else { //HAL_I2S_RxCpltCallback , tx_flag = 1
			for (int i = DMA_BUFFER_SIZE / 2; i < DMA_BUFFER_SIZE; i++) {
				dma_transmit_buffer[i] = audio_buffer[i + tx_flag * DMA_BUFFER_SIZE];
			}
		}
		tx_flag ^= 1;
	}
}

void process_audio_receive_data(int16_t *dma_receive_buffer, uint16_t size) {

	if (rx_flag == 0) {
		if (size == DMA_BUFFER_SIZE / 2) { //RxHalfCpltCallback
			for (int i = 0; i < DMA_BUFFER_SIZE / 2; i++) {
				audio_buffer[i] = dma_receive_buffer[i];
			}

		} else { //HAL_I2S_RxCpltCallback
			for (int i = DMA_BUFFER_SIZE / 2; i < DMA_BUFFER_SIZE; i++) {
				audio_buffer[i] = dma_receive_buffer[i];
			}
			rx_flag ^= 1;
		}

	} else if (rx_flag != 0) {
		if (size == DMA_BUFFER_SIZE / 2) { //RxHalfCpltCallback , rx_flag = 1
			for (int i = 0; i < DMA_BUFFER_SIZE / 2; i++) {
				audio_buffer[i + rx_flag * DMA_BUFFER_SIZE] = dma_receive_buffer[i];
			}

		} else { //HAL_I2S_RxCpltCallback , rx_flag = 1
			for (int i = DMA_BUFFER_SIZE / 2; i < DMA_BUFFER_SIZE; i++) {
				audio_buffer[i + rx_flag * DMA_BUFFER_SIZE] = dma_receive_buffer[i];
			}
		}
		rx_flag ^= 1;
	}

	if (dma_flag == 1) {
		dma_flag++;
	}

}

void AIC3204_page_select(uint8_t page) {
	if (p_num != page) {
		p_num = page;
		AIC3204_rset(0, p_num);
	}
}

void aic3204_hw_init(void) {
	HAL_Delay(100);
//AIC32_RST_GPIO_Port->BSRR = (uint32_t)AIC32_RST_Pin<<16;
	GPIOB->BSRR = (uint32_t) GPIO_PIN_5 << 16;
	HAL_Delay(10);
	GPIOB->BSRR = GPIO_PIN_5;
	HAL_Delay(10);
}

void aic3204_sw_reset(void) {
	AIC3204_rset(0, 0);      // Select page 0
	AIC3204_rset(1, 1);      // Reset codec
	HAL_Delay(100);
}

//===============================================
// ADC Channel Volume Control
// [D6 ~ D0]: -12.0 ~ +20.0  (0.5 step)
// default: 0.0dB
//===============================================
void aic3204_ADC_volume(float vol) {
//	float tmp;
//	int8_t tmp8;
//	uint8_t dat;
//
//	if ((vol < -12.0) || (vol > 20.0))
//		return;
//
//	tmp = (vol * 2);
//	tmp8 = (int8_t) tmp;
//	dat = tmp8 & 0x7F;

//	PRINTF_DEBUG(" >> ADAVol:%d.%d[0x%02x]\r\n", (int) vol,
//			(int) (vol * 10) % 10, dat);
//DAC Channel Digital Volume Control
	AIC3204_page_select(0);      // Select page 0
	AIC3204_rset(0x53, 0x00);    // Left ADC Volume
	AIC3204_rset(0x54, 0x00);    //Right ADC Volume
//	AIC3204_rset(0x53, dat);    // Left ADC Volume
//	AIC3204_rset(0x54, dat);    //Right ADC Volume
}

//===============================================
// DAC Channel Digital Volume Control
// [D7 ~ D0]: -63.5 ~ +24.0  (0.5 step)
// default: 0.0dB
//===============================================
void aic3204_DAC_volume(float vol) {
//	float tmp;
//	int8_t dat;
//
//	if ((vol < -63.5) || (vol > 24.0))
//		return;
//
//	tmp = (vol * 2);
//	dat = (int8_t) tmp;

//	PRINTF_DEBUG(" >> DACVol:%d.%d[0x%2x]\r\n", (int) vol,
//			(int) (vol * 10) % 10, (uint8) dat);
//DAC Channel Digital Volume Control
	AIC3204_page_select(0);      // Select page 0
	AIC3204_rset(0x41, 0x00);    // Left DAC Volume
	AIC3204_rset(0x42, 0x00);    //Right DAC Volume
//	AIC3204_rset(0x41, (uint8_t) dat);    // Left DAC Volume
//	AIC3204_rset(0x42, (uint8_t) dat);    //Right DAC Volume
}

void aic3204_DAC_Enable(void) {
	AIC3204_page_select(0);          // Select page 0
	AIC3204_rset(0x40, 0); // Left and Right Channel have independent volume control
	AIC3204_rset(0x3F, 0xd4);  // Power up left,right data paths and set channel
}

void aic3204_ADC_Enable(void) {
	AIC3204_page_select(0);          // Select page 0
	AIC3204_rset(0x51, 0xC0);    // Powerup Left and Right ADC
	AIC3204_rset(0x52, 0);       // Unmute Left and Right ADC
}

void aic3204_micbias_set(void) {
	AIC3204_page_select(1);          // Select page 1 MICBIAS powered up
	AIC3204_rset(0x33, 0x50);     //
}

void aic3204_HeadphoneOut_volume(int vol_l, int vol_r) {
	int8 tmp;
	uint8 dat;

	if (vol_l > 29)
		return;

	AIC3204_page_select(1);      // Select page 1
	if (vol_l < -6) {
//		PRINTF_DEBUG(" >> HPL Amp: Mute\r\n");
		//Headphone Out Volume Control
		AIC3204_rset(0x10, 0x40);    // HPL Driver Gain
	} else {
		tmp = (int8) vol_l;
		dat = tmp & 0x3F;

//		PRINTF_DEBUG(" >> HPL Amp:%d[0x%2x]\r\n", vol_l, (uint8) dat);
		//Headphone Out Volume Control
		AIC3204_rset(0x10, (uint8) dat);    // HPL Driver Gain
	}
	if (vol_r > 29)
		return;
	if (vol_r < -6) {
//		PRINTF_D(" >> HPR Amp: Mute\r\n");
		//Headphone Out Volume Control
		AIC3204_rset(0x11, 0x40);    // HPR Driver Gain
	} else {
		tmp = (int8) vol_r;
		dat = tmp & 0x3F;

//		PRINTF_DEBUG(" >> HPR Amp:%d[0x%2x]\r\n", vol_l, (uint8) dat);
		//Headphone Out Volume Control
		AIC3204_rset(0x11, (uint8) dat);    // HPR Driver Gain
	}
}

void aic3204_OUTPUT_Amp_enable(uint8 dat) {
	uint8 tmp;
	AIC3204_page_select(1);          // Select page 1
	aic3204_read(0x09, &tmp);
	tmp |= (dat & 0x3F);
	AIC3204_rset(0x09, tmp);     // [0x09] Power up HPL,HPR
}

void aic3204_MICPGA_volume(float vol) {
	float tmp;
	int8 dat;

	if ((vol < 0) || (vol > 47.5))
		return;

	tmp = (vol * 2);
	dat = (int8) tmp;

//	PRINTF_DEBUG(" >> MICPGA:%d.%d[0x%2x]\r\n", (int) vol,
//			(int) (vol * 10) % 10, (uint8) dat);
//DAC Channel Digital Volume Control
	AIC3204_page_select(1);      // Select page 1
	AIC3204_rset(0x3B, (uint8) dat);    // Left MICPGA Volume
	AIC3204_rset(0x3C, (uint8) dat);    //Right MICPGA Volume
}

void audio_vol_initial(void) {
	aic3204_micbias_set();

	aic3204_HeadphoneOut_volume(11, 0); // 0x10
	aic3204_OUTPUT_Amp_enable(0x0F); // 0x09
	aic3204_MICPGA_volume(10); // 0x3B

	HAL_Delay(50);
//aic3204_ADC_volume(-6);        //(-12);
	aic3204_ADC_volume(-5);
	aic3204_ADC_Enable();
	aic3204_DAC_volume(-30);        //(-50);
	aic3204_DAC_Enable();
	HAL_Delay(50);
}

void audio_initial(void) {
	AIC3204_page_select(1);      // Select page 1
	AIC3204_rset(MICPGA_PTIRC, 0x10); // IN2L is routed to Left MICPGA , 0x34
	AIC3204_rset(MICPGA_NTIRC, 0x10); // IN1L is routed to HPL , 0x36
	AIC3204_rset(MICPGA_VCR, 0X00);  // MIC_PGA_L unmute ( 10dB ) , 0x3B
	AIC3204_rset(HPL_RSR, 0x08); // IN1L is routed to HPL, 0x0C
	AIC3204_rset(ODPCR, 0x30); // HPL, MAL powered up , 0x09
	AIC3204_rset(HPL_DGSR, 0x3A); // HPL driver gain setting register  (HPL is not mute, driver gain 0dB ), 0x10
	HAL_Delay(50);

}

void aic3204_init(void) {

	/* Configure Parallel Port */
//    SYS_EXBUSSEL = 0x1000;  // Configure Parallel Port mode = 1 for I2S2
	/* Configure AIC3204 */

	AIC3204_page_select(0);      // Select page 0
	AIC3204_rset(0x01, 0x01);      // Reset codec

	AIC3204_page_select(1);      // Point to page 1
	AIC3204_rset(0x01, 0x08);      // Disable crude AVDD generation from DVDD
	AIC3204_rset(0x02, 0x01);      // Enable Analog Blocks, use LDO power

	AIC3204_rset(0, 0);
	/* PLL and Clocks config and Power Up  */
	AIC3204_rset(0x1B, 0xcd); // BCLK and WCLK is set as o/p to AIC3204(Master)   // 16bit
							  // Audio interface Selection (D7-6)
							  // 00 = I2S / 01 = DSP / 10 = RJF / [11] = LJF
							  // Audio Data Word length (D5-4)
							  // [00]: Data Word length = 16 bits ]
							  // 01: Data Word length = 20 bits
							  // 10: Data Word length = 24 bits
							  // 11: Data Word length = 32 bits
							  // BCLK Direction Control (D3)
							  // 0: BCLK is input to the device
							  // [1]: BCLK is output from the device
							  // BCLK Direction Control (D2)
							  // 0: BCLK is input to the device
							  // [1]: BCLK is output from the device
							  // DOUT High Impendance Output Control (D0)
							  // 0: DOUT will not be high impedance while Audio Interface is active
							  // [1]: DOUT will be high impedance after data has been transferred

	AIC3204_rset(0x1C, 0x00);  // Data ofset = 0

	AIC3204_rset(0x04, 0x00);  // [EXT. 12.288M] PLL setting: CODEC_CLKIN <-MCLK
	AIC3204_rset(0x06, 0x07);      // PLL setting: J=7
	AIC3204_rset(0x07, 0x06);   // PLL setting: HI_BYTE(D)
	AIC3204_rset(0x08, 0x90);   // PLL setting: LO_BYTE(D)

#if (AUDIO_SAMPLE_RATE == I2S_AUDIOFREQ_48K)
    // DAC_CLK = 12.288MHz
    AIC3204_rset( 30, 0x84 );  //0x1E // For 32 bit clocks per frame in Master mode ONLY
                                // BCLK=DAC_CLK/N =(12288000/4) = 3.072MHz = 64*fs
#elif (AUDIO_SAMPLE_RATE == I2S_AUDIOFREQ_16K)
// DAC_CLK = 4.096Hz
	AIC3204_rset(0x1E, 0x84); //0x1E // For 32 bit clocks per frame in Master mode ONLY
							  // BCLK=DAC_CLK/N =(4.096/4) = 1.024MHz = 64*fs
#elif (AUDIO_SAMPLE_RATE == I2S_AUDIOFREQ_8K)
    // DAC_CLK = 4.096Hz
    AIC3204_rset( 30, 0x88 );  //0x1E // For 32 bit clocks per frame in Master mode ONLY
                                // BCLK=DAC_CLK/N =(0.512/8) = 0.512MHz = 64*fs
#else
    // DAC_CLK = 12.288MHz
    AIC3204_rset( 30, 0x86 );  //0x1E // For 32 bit clocks per frame in Master mode ONLY
                                // BCLK=DAC_CLK/N =(12288000/6) = 2.048MHz = 64*fs
#endif

//    AIC3204_rset( 5, 0x91 );   //PLL setting: Power up PLL, P=1 and R=1
	AIC3204_rset(0x05, 0x11);   //PLL setting: Power down, P=1 and R=1

#if (AUDIO_SAMPLE_RATE == I2S_AUDIOFREQ_48K)
    AIC3204_rset( 13, 0 );     // Hi_Byte(DOSR) for DOSR = 128 decimal or 0x0080 DAC oversamppling
    AIC3204_rset( 14, 0x80 );  // Lo_Byte(DOSR) for DOSR = 128 decimal or 0x0080
    AIC3204_rset( 20, 0x80 );  // AOSR for AOSR = 128 decimal or 0x0080 for decimation filters 1 to 6

    AIC3204_rset( 11, 0x81 );  // Power up NDAC and set NDAC value to 1
    AIC3204_rset( 12, 0x82 );  // Power up MDAC and set MDAC value to 2
    AIC3204_rset( 18, 0x81 );  // Power up NADC and set NADC value to 1
    AIC3204_rset( 19, 0x82 );  // Power up MADC and set MADC value to 2
#elif (AUDIO_SAMPLE_RATE == I2S_AUDIOFREQ_16K)
	AIC3204_rset(0x0D, 0x00); // Hi_Byte(DOSR) for DOSR = 128 decimal or 0x0080 DAC oversamppling
	AIC3204_rset(0x0E, 0x80);  // Lo_Byte(DOSR) for DOSR = 128 decimal or 0x0080
	AIC3204_rset(0x14, 0x80); // AOSR for AOSR = 128 decimal or 0x0080 for decimation filters 1 to 6

	AIC3204_rset(0x0B, 0x83);  // Power up NDAC and set NDAC value to 3
	AIC3204_rset(0x0C, 0x82);  // Power up MDAC and set MDAC value to 2
	AIC3204_rset(0x12, 0x83);  // Power up NADC and set NADC value to 3
	AIC3204_rset(0x13, 0x02);  // Power up MADC and set MADC value to 2
#elif (AUDIO_SAMPLE_RATE == I2S_AUDIOFREQ_8K)
    AIC3204_rset( 13, 0 );     // Hi_Byte(DOSR) for DOSR = 64 decimal or 0x0080 DAC oversamppling
    AIC3204_rset( 14, 0x80 );  // Lo_Byte(DOSR) for DOSR = 64 decimal or 0x0080
    AIC3204_rset( 20, 0x80 );  // AOSR for AOSR = 64 decimal or 0x0080 for decimation filters 1 to 6

    AIC3204_rset( 11, 0x83 );  // Power up NDAC and set NDAC value to 3
    AIC3204_rset( 12, 0x84 );  // Power up MDAC and set MDAC value to 4
    AIC3204_rset( 18, 0x83 );  // Power up NADC and set NADC value to 3
    AIC3204_rset( 19, 0x84 );  // Power up MADC and set MADC value to 4
#else // (AUDIO_SAMPLE_RATE == I2S_AUDIOFREQ_32K)
    AIC3204_rset( 13, 0 );     // Hi_Byte(DOSR) for DOSR = 128 decimal or 0x0080 DAC oversamppling
    AIC3204_rset( 14, 0x80 );  // Lo_Byte(DOSR) for DOSR = 128 decimal or 0x0080
    AIC3204_rset( 20, 0x80 );  // AOSR for AOSR = 128 decimal or 0x0080 for decimation filters 1 to 6

    AIC3204_rset( 11, 0x81 );  // Power up NDAC and set NDAC value to 1
    AIC3204_rset( 12, 0x83 );  // Power up MDAC and set MDAC value to 3
    AIC3204_rset( 18, 0x81 );  // Power up NADC and set NADC value to 1
    AIC3204_rset( 19, 0x83 );  // Power up MADC and set MADC value to 3
#endif
	HAL_Delay(50);

	return;

	/* DAC ROUTING and Power Up */
//    AIC3204_rset( 0, 1 );      // Select page 1
//    AIC3204_rset( 0x0c, 0x08 );   // LDAC AFIR routed to HPL
//    AIC3204_rset( 0x0d, 0x08 );   // RDAC AFIR routed to HPR
	AIC3204_page_select(0);      // Select page 0
	AIC3204_rset(0x40, 0x02);     // Left vol=right vol
	AIC3204_rset(0x41, 0x00);     // Left DAC gain to 0dB VOL; Right tracks Left
	AIC3204_rset(0x3f, 0xd4);  // Power up left,right data paths and set channel
	AIC3204_page_select(1);      // Select page 1
	AIC3204_rset(0x10, 0x0A);  // Unmute HPL , 10dB gain
	AIC3204_rset(0x11, 0x0A);  // Unmute HPR , 10dB gain
	AIC3204_rset(0x09, 0x30);   // Power up HPL,HPR
	AIC3204_page_select(0);      // Select page 0
//    USBSTK5505_wait( 100 );    // wait
	HAL_Delay(100);
	/* ADC ROUTING and Power Up */
	AIC3204_page_select(1);      // Select page 1
	AIC3204_rset(0x33, 0x48); // power up MICBIAS with AVDD (0x40)or LDOIN (0x48)	//MM - added micbias
	AIC3204_rset(0x34, 0xC0);  // STEREO 1 Jack
							   // IN1_L to LADC_P through 40 kohm
	AIC3204_rset(0x37, 0xC0);		         // IN1_R to RADC_P through 40 kohmm
	AIC3204_rset(0x36, 0x03);   // CM_1 (common mode) to LADC_M through 40 kohm
	AIC3204_rset(0x39, 0xc0);   // CM_1 (common mode) to RADC_M through 40 kohm
	AIC3204_rset(0x3b, 0x00);   // MIC_PGA_L unmute
	AIC3204_rset(0x3c, 0x00);   // MIC_PGA_R unmute
	AIC3204_page_select(0);      // Select page 0
	AIC3204_rset(0x51, 0xc0);      // Powerup Left and Right ADC
	AIC3204_rset(0x52, 0x00);       // Unmute Left and Right ADC
	AIC3204_page_select(0);      // Select page 0
//    USBSTK5505_wait( 100 );  // Wait
	HAL_Delay(100);
}

void Analog_Bypass(void) {

	AIC3204_page_select(0);	 	 // Select page 0
	AIC3204_rset(0x01, 0x01); // reset page 1

	HAL_Delay(500);

	AIC3204_page_select(1);	 	 // Select page 1
	AIC3204_rset(LDO_CR, 0x01); // Enable Analog Blocks, use LDO power , 0x02
	AIC3204_rset(MICBIAS_CR, 0x50); // HPL, MAL powered up , 0x33
	AIC3204_rset(ODPCR, 0x22); // HPL, MAL powered up , 0x09
	AIC3204_rset(MICPGA_PTIRC, 0x30); // IN2L is routed to Left MICPGA , 0x34
	AIC3204_rset(MICPGA_VCR, 0X00);  // MIC_PGA_L unmute ( 10dB ) , 0x3B
	AIC3204_rset(MIX_AMP_LVCR, 0x15); // volume control register ( 0dB ) , 0x18
	AIC3204_rset(HPL_RSR, 0x02); // IN1L is routed to HPL, 0x0C
	AIC3204_rset(HPL_DGSR, 0x00); // HPL driver gain setting register  (HPL is not mute, driver gain 0dB ), 0x10
	AIC3204_rset(MICPGA_NTIRC, 0xc0); // IN1L is routed to HPL , 0x36
}

uint8_t flash_spi(uint8_t data) {
	uint8_t ret;
	HAL_SPI_TransmitReceive(&hspi1, &data, &ret, 1, 100);
	return ret;
}

uint8_t buf_copy(uint32_t Bytes_Address) {
	uint8_t val;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x13);
	flash_spi((Bytes_Address & 0xFF000000) >> 24);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	val = flash_spi(0xA5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET); //pBuffer result value
	return val;
}

void flash_wait_end(void) { // Read Status Register-1
	uint8_t stat = 0;

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x05);
	do {
		stat = flash_spi(0xA5);
		HAL_Delay(1);
	} while ((stat & 0x01) == 0x01);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET); // CS to low
}

void flash_write_enable(void) { // Write Enable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x06);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_write_disable(void) { // Write Disable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x04);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_write_byte(uint32_t WriteAddr_inBytes) { //****
// Page Program
	flash_wait_end();
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x12);
	flash_spi((WriteAddr_inBytes & 0xFF000000) >> 24);
	flash_spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	flash_spi((WriteAddr_inBytes & 0xFF00) >> 8);
	flash_spi(WriteAddr_inBytes & 0xFF);
	flash_spi(flashwt_buf[0]);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
}

void sector_write_byte(uint32_t WriteAddr_inBytes) {
// Page Program
	flash_wait_end();
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x12);
	flash_spi((WriteAddr_inBytes & 0xFF000000) >> 24);
	flash_spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	flash_spi((WriteAddr_inBytes & 0xFF00) >> 8);
	flash_spi(WriteAddr_inBytes & 0xFF);
	for (int i = 256 * flag; i < 256 + 256 * flag; i++) {
		flash_spi(Flash_buf[i]);
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
}

uint32_t flash_SectorToPage(uint32_t SectorAddress) {
	return (SectorAddress * SectorSize) / PageSize;
}

uint32_t flash_BlockToPage(uint32_t BlockAddress) {
	return (BlockAddress * BlockSize) / PageSize;
}

void flash_write_page(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize) {
	if (((NumByteToWrite_up_to_PageSize + OffsetInByte) > PageSize) || (NumByteToWrite_up_to_PageSize == 0))
		NumByteToWrite_up_to_PageSize = PageSize - OffsetInByte;
	if ((OffsetInByte + NumByteToWrite_up_to_PageSize) > PageSize)
		NumByteToWrite_up_to_PageSize = PageSize - OffsetInByte;
	flash_wait_end();
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x02);
	Page_Address = (Page_Address * PageSize) + OffsetInByte;
	flash_spi((Page_Address & 0xFF0000) >> 16);
	flash_spi((Page_Address & 0xFF00) >> 8);
	flash_spi(Page_Address & 0xFF);
	HAL_SPI_Transmit(&hspi1, pBuffer, NumByteToWrite_up_to_PageSize, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(1);
}

void flash_write_sector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize) {
	if ((NumByteToWrite_up_to_SectorSize > SectorSize) || (NumByteToWrite_up_to_SectorSize == 0))
		NumByteToWrite_up_to_SectorSize = SectorSize;

	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > SectorSize)
		BytesToWrite = SectorSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_SectorSize;
	StartPage = flash_SectorToPage(Sector_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;

	do {
		flash_write_page(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
}

void flash_write_block(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize) {
	if ((NumByteToWrite_up_to_BlockSize > BlockSize) || (NumByteToWrite_up_to_BlockSize == 0))
		NumByteToWrite_up_to_BlockSize = BlockSize;

	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;

	if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > BlockSize)
		BytesToWrite = BlockSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_BlockSize;
	StartPage = flash_BlockToPage(Block_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;
	do {
		flash_write_page(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
}

void compare_flash_read(uint32_t Bytes_Address) { // Read(0Ch)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0C);
	flash_spi((Bytes_Address & 0xFF000000) >> 24);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	flash_spi(0); // DUMMY
	for (int i = 0; i < 4; i++) {
		buff[i] = flash_spi(0xA5);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
}

void flash_read(uint32_t Bytes_Address) { // Read(0Ch)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0C);
	flash_spi((Bytes_Address & 0xFF000000) >> 24);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	flash_spi(0); // DUMMY
	for (int i = 0; i < 4; i++) {
		buff[i] = flash_spi(0xA5);
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);

	PRINTF_DEBUG("\r\nRead Flash(%p) :", Bytes_Address);
	for (int i = 0; i < 4; i++) {
		PRINTF_DEBUG(" 0x%02X", buff[i]);
	}
	PRINTF_DEBUG("\r\n");
}

void sector_backup(uint32_t Bytes_Address) { // Read(03h)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0C);
	flash_spi((Bytes_Address & 0xFF000000) >> 24);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	flash_spi(0); // DUMMY
	for (int i = 0; i < 4096; i++) {
		Flash_buf[i] = flash_spi(0xA5);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
}

void flash_read_byte(uint8_t *pBuffer, uint32_t Bytes_Address) {
// Fast Read
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0B);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	flash_spi(0); // DUMMY
	*pBuffer = flash_spi(0xA5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);

	PRINTF_DEBUG("Fast_read(0Bh) : 0x%02X\n", *pBuffer);
}

void flash_read_bytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead) {
// Fast Read
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0B);
	flash_spi((ReadAddr & 0xFF0000) >> 16);
	flash_spi((ReadAddr & 0xFF00) >> 8);
	flash_spi(ReadAddr & 0xFF);
	flash_spi(0); // DUMMY
	HAL_SPI_Receive(&hspi1, pBuffer, NumByteToRead, 2000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_read_page(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize) {
	if ((NumByteToRead_up_to_PageSize > PageSize) || (NumByteToRead_up_to_PageSize == 0))
		NumByteToRead_up_to_PageSize = PageSize;
	if ((OffsetInByte + NumByteToRead_up_to_PageSize) > PageSize)
		NumByteToRead_up_to_PageSize = PageSize - OffsetInByte;

	Page_Address = Page_Address * PageSize + OffsetInByte;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0B);
	flash_spi((Page_Address & 0xFF0000) >> 16);
	flash_spi((Page_Address & 0xFF00) >> 8);
	flash_spi(Page_Address & 0xFF);
	flash_spi(0);
	HAL_SPI_Receive(&hspi1, pBuffer, NumByteToRead_up_to_PageSize, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_read_sector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize) {
	if ((NumByteToRead_up_to_SectorSize > SectorSize) || (NumByteToRead_up_to_SectorSize == 0))
		NumByteToRead_up_to_SectorSize = SectorSize;

	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > SectorSize)
		BytesToRead = SectorSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;
	StartPage = flash_SectorToPage(Sector_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;
	do {
		flash_read_page(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
}

void flash_read_block(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize) {
	if ((NumByteToRead_up_to_BlockSize > BlockSize) || (NumByteToRead_up_to_BlockSize == 0))
		NumByteToRead_up_to_BlockSize = BlockSize;

	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_BlockSize) > BlockSize)
		BytesToRead = BlockSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_BlockSize;
	StartPage = flash_BlockToPage(Block_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;
	do {
		flash_read_page(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
}

uint8_t flash_read_Status(uint8_t register_num) { // Read Status Register (05h - 1) (35h - 2)
	uint8_t status = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	if (register_num == 1) {
		flash_spi(0x05);
		status = flash_spi(0xA5);
	} else if (register_num == 2) {
		flash_spi(0x35);
		status = flash_spi(0xA5);
	} else if (register_num == 3) {
		flash_spi(0x15);
		status = flash_spi(0xA5);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	return status;
}

void flash_erase_chip(void) { //Chip Erase
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0xC7);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(10);
}

void flash_erase_sector(uint32_t SectorAddr) { //Sector Erase (4KB)
	flash_wait_end();
//	SectorAddr = SectorAddr * SectorSize;
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x21);
	flash_spi((SectorAddr & 0xFF000000) >> 24);
	flash_spi((SectorAddr & 0xFF0000) >> 16);
	flash_spi((SectorAddr & 0xFF00) >> 8);
	flash_spi(SectorAddr & 0xFF);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(1);
}

void flash_erase_block(uint32_t BlockAddr) {
	flash_wait_end();
	BlockAddr = BlockAddr * SectorSize * 16;
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0xD8);
	flash_spi((BlockAddr & 0xFF0000) >> 16);
	flash_spi((BlockAddr & 0xFF00) >> 8);
	flash_spi(BlockAddr & 0xFF);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(1);
}

uint32_t flash_read_ID(void) { //JEDEC ID
	uint32_t Temp0 = 0, Temp1 = 0, Temp2 = 0;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	flash_spi(0x9F);
	Temp0 = flash_spi(0xA5);
	Temp1 = flash_spi(0xA5);
	Temp2 = flash_spi(0xA5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	return Temp;
}

void flash_read_uniqID(void) { //Read Unique ID
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	flash_spi(0x4B);
	for (uint8_t i = 0; i < 4; i++)
		flash_spi(0x4B);
	for (uint8_t i = 0; i < 8; i++)
		uniqID[i] = flash_spi(0x4B);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

	for (uint8_t i = 0; i < 8; i++)
		PRINTF_DEBUG("%02X ", uniqID[i]);
	PRINTF_DEBUG("\n");
}

void save_flash(uint32_t hex1, uint32_t hex2) {
	uint32_t PAGEError = 0;
	static FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = START_ADDR;
	EraseInitStruct.NbSectors = (END_ADDR - START_ADDR) / 1024;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) hex1, (uint64_t) hex2); // 32bit
	HAL_FLASH_Lock();

	PRINTF_DEBUG("Address : 0x%08x --> Hex Value : %08x\r\n\r\n", (uint32_t*) hex1, (uint64_t*) hex2);
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

void FND_CLR(void) {
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
						PRINTF_DEBUG("read value : 0x%02x", (*(uint32_t*) hex1 & 0xFF000000) >> 24);
						PRINTF_DEBUG(" 0x%02x", (*(uint32_t*) hex1 & 0xFF0000) >> 16);
						PRINTF_DEBUG(" 0x%02x", (*(uint32_t*) hex1 & 0xFF00) >> 8);
						PRINTF_DEBUG(" 0x%02x\r\n", *(uint32_t*) hex1 & 0xFF);
					} else {
						PRINTF_DEBUG("Segmentation Fault\r\n\r\n");
					}
				}
			}

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

			else if (!strcmp(frame1, "flashwt") || !strcmp(frame1, "FLASHWT")) {
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
			} else if (!strcmp(frame1, "flashrd") || !strcmp(frame1, "FLASHRD")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					hex2 = StringToHexa(frame2);
					flash_read(hex2);
				}
			} else if (!strcmp(frame1, "flash9fh") || !strcmp(frame1, "FLASH9FH")) {
				PRINTF_DEBUG("Command : %s", frame1);
				PRINTF_DEBUG("\r\n");
				Temp = flash_read_ID();
				PRINTF_DEBUG("JEDEC ID : %X\r\n", Temp);

			} else if (!strcmp(frame1, "flash4bh") || !strcmp(frame1, "FLASH4BH")) {
				PRINTF_DEBUG("Command : %s", frame1);
				PRINTF_DEBUG("\r\n");
				flash_read_uniqID();

			} else if (!strcmp(frame1, "flashrdrg") || !strcmp(frame1, "FLASHRDRG")) {
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
			} else if (!strcmp(frame1, "flashcp") || !strcmp(frame1, "FLASHCP")) {
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
					flash_add = (hex2 & 0x01FFF000);
//					flash_add = GetSector(hex2); //sector explorer
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
						PRINTF_DEBUG("\r\nRead MCU(%p) : 0x%02X", hex1, (*(uint32_t*) hex1 & 0xFF000000) >> 24);
						PRINTF_DEBUG(" 0x%02X", (*(uint32_t*) hex1 & 0xFF0000) >> 16);
						PRINTF_DEBUG(" 0x%02X", (*(uint32_t*) hex1 & 0xFF00) >> 8);
						PRINTF_DEBUG(" 0x%02X", *(uint32_t*) hex1 & 0xFF);
						flash_read(hex2);
						PRINTF_DEBUG(">>Comparisons correct\r\n");
						sstack = 0;
					} else {
						PRINTF_DEBUG(">>Comparisons not correct\r\n");
						sstack = 0;
					}
				}
			} else if (!strcmp(frame1, "flashclr") || !strcmp(frame1, "FLASHCLR")) {
				PRINTF_DEBUG(">>Chip Erasing...\r\n");
				flash_erase_chip();
				PRINTF_DEBUG(">>Chip Erase end\r\n");
			}

			else if (!strcmp(frame1, "aic") || !strcmp(frame1, "AIC")) {
				if ((frame2 != NULL) && (!strcmp(frame2, "wt") || !strcmp(frame2, "WT"))) {
					hex0 = StringToHexa(frame3);
					hex1 = StringToHexa(frame4);
					AIC3204_rset(hex0, hex1);
					PRINTF_DEBUG("AIC WT %x --> (%x)\r\n", hex0, hex1);

					PRINTF_DEBUG(">>");
				} else if ((frame2 != NULL) && (!strcmp(frame2, "rd") || !strcmp(frame2, "RD"))) {
					hex0 = StringToHexa(frame3);
					I2C_Read(hex0);
					PRINTF_DEBUG(">>");
				} else if ((frame2 != NULL) && (!strcmp(frame2, "init") || !strcmp(frame2, "INIT"))) {
					if (dma_flag == 3) {
						dma_flag = 0;
						HAL_I2S_DMAPause(&hi2s2);
						HAL_I2S_DMAPause(&hi2s3);
						PRINTF_DEBUG("DMA PAUSE\r\n");
					} else {
						dma_flag = 1;
						HAL_I2S_DMAResume(&hi2s2);
						HAL_I2S_DMAResume(&hi2s3);
						PRINTF_DEBUG("DMA RESUME\r\n");
					}
				}

			} else if ((frame2 != NULL) && (!strcmp(frame2, "mset") || !strcmp(frame2, "MSET"))) {
				memset(dma_receive_buffer, 0, sizeof(int16) * DMA_BUFFER_SIZE);
				memset(audio_buffer, 0, sizeof(int16) * AUDIO_BUFFER_SIZE);
				PRINTF_DEBUG("audio buffer memset");

			} else if (!strcmp(frame2, "analog") || !strcmp(frame2, "ANALOG")) {
				Analog_Bypass();
			} else if (!strcmp(frame2, "dmastop") || !strcmp(frame2, "DMASTOP")) {
				HAL_I2S_DMAStop(&hi2s3);
			} else if (!strcmp(frame2, "micoff") || !strcmp(frame2, "MICOFF")) {
				AIC3204_rset(0x34, 0x00);
				PRINTF_DEBUG(">>");
			}
		} else if (!strcmp(frame1, "i2cwt") || !strcmp(frame1, "I2CWT")) {
			if ((frame2 != NULL) && (!strcmp(frame2, "HPL_RSR") || (!strcmp(frame2, "hpl_rsr")))) {
				hex3 = StringToHexa(frame3);
			}
		}
		buf_clear(&rbuf);
		ring_buf_clr(&rbuf);
		memset(rx_frame, 0, RX_FRAME_MAX);
		rx_frame_cnt = 0;

		use_console_end = (TIM3->CNT) + count * 1000; //end count
		use_console_run = (use_console_end - use_console_start);
		PRINTF_DEBUG("\r\nuse_console running time : %dus", use_console_run);
		PRINTF_DEBUG("\r\n\r\n>>");
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

uint8_t read_pin() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == '\0') {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) != '\0') {
			mode_key1 ^= 1;
			PRINTF_DEBUG("SELECT MODE 1\r\n");
			PRINTF_DEBUG(">>");
		}
		return 0;
	} else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == '\0') {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) != '\0') {
//			mode = 2;
//			PRINTF_DEBUG("SELECT MODE 2\r\n");
//			PRINTF_DEBUG(">>");
//		}

			if (dma_flag == 3) {
				dma_flag = 0;
				HAL_I2S_DMAPause(&hi2s2);
				HAL_I2S_DMAPause(&hi2s3);
				PRINTF_DEBUG("DMA PAUSE\r\n");
			} else {
				dma_flag = 1;
				HAL_I2S_DMAResume(&hi2s2);
				HAL_I2S_DMAResume(&hi2s3);
				PRINTF_DEBUG("DMA RESUME\r\n");
			}
		}
		return 0;
	} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == '\0' && check == 0) {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != '\0') {
			if (mode == 4) {
				mode = 3;
				PRINTF_DEBUG("SELECT MODE 3\r\n");
				PRINTF_DEBUG(">>");
			} else if (mode == 3) {
				mode = 4;
				PRINTF_DEBUG("SELECT MODE 4\r\n");
				PRINTF_DEBUG(">>");
			} else {
				mode = 3;
				cnt_num = 0;
				PRINTF_DEBUG("SELECT MODE 3\r\n");
				PRINTF_DEBUG(">>");
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

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	process_audio_receive_data(dma_receive_buffer, DMA_BUFFER_SIZE / 2);

//	/*Receive_Buf tx_rx*/
//	if (dma_flag == 1)
//	{
//		dma_flag++;
//	}
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	process_audio_receive_data(dma_receive_buffer, DMA_BUFFER_SIZE);

}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	process_audio_transmit_data(audio_buffer, DMA_BUFFER_SIZE / 2);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	process_audio_transmit_data(audio_buffer, DMA_BUFFER_SIZE);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	HAL_UART_Receive_IT(&huart2, &rcv_data, 1); 			// UART Interrupt
	aic3204_init();
	audio_vol_initial();
	audio_initial();
	dma_flag = 0;
	HAL_I2S_Receive_DMA(&hi2s3, (uint16*) dma_receive_buffer, DMA_BUFFER_SIZE); // I2S DMA Interrupt
//	memset((uint8*) dma_receive_buffer, 0xff, sizeof(dma_receive_buffer));
//	HAL_I2S_Receive_DMA(&hi2s3, (uint16*) dma_receive_buffer, 256); // I2S DMA Interrupt
	PRINTF_DEBUG(">>");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		use_console();

		if (read_pin() == '\0') {
			read_pin();
		}

//		/*Receive_Buf tx_rx*/
		if (dma_flag == 2) {
			HAL_I2S_Transmit_DMA(&hi2s2, (uint16*) dma_transmit_buffer, DMA_BUFFER_SIZE);
			dma_flag++;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

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
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_I2S2_Init(void) {

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_SLAVE_TX;
	hi2s2.Init.Standard = I2S_STANDARD_MSB;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
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
static void MX_I2S3_Init(void) {

	/* USER CODE BEGIN I2S3_Init 0 */

	/* USER CODE END I2S3_Init 0 */

	/* USER CODE BEGIN I2S3_Init 1 */

	/* USER CODE END I2S3_Init 1 */
	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;
	hi2s3.Init.Standard = I2S_STANDARD_MSB;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
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
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 900 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 90;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, FLASH_SS_Pin | FND_COM1_Pin | FND_COM2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	SEG_A_Pin | SEG_B_Pin | SEG_F_Pin | SEG_G_Pin | SEG_DP_Pin | GPIO_PIN_3 | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin, GPIO_PIN_RESET);

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
	GPIO_InitStruct.Pin = FLASH_SS_Pin | FND_COM1_Pin | FND_COM2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_F_Pin SEG_G_Pin
	 SEG_DP_Pin PB3 SEG_C_Pin SEG_D_Pin
	 SEG_E_Pin */
	GPIO_InitStruct.Pin = SEG_A_Pin | SEG_B_Pin | SEG_F_Pin | SEG_G_Pin | SEG_DP_Pin | GPIO_PIN_3 | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : KEY_2_Pin */
	GPIO_InitStruct.Pin = KEY_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(KEY_2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
