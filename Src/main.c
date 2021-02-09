
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_midi.h"
#include "usbd_midi_if.h"
#include "fonts.h"
#include "ssd1306.h"
uint8_t sndmsg[4] = {1, 2, 3,4};
uint8_t rxmsg[4];
uint8_t scan_note = 0;
uint8_t button_pressed = 0;
uint8_t need_update = 0;
uint8_t Buf[20]={0};
uint8_t vline = 0;

/* SPI transfer variables ---------------------------------------------------------*/
uint8_t Tbuffer_1[27] = {
	168,  119,  119,  0,  119,  119,  0,  0,  0,  1,  176,  0,  0,  0,  0,  0,  0,  0xC,  0,  0x20,  0,  0,  0,  88,  0,  0,  0};
uint8_t Tbuffer_2[27] = {
	168,  119,  119,  0,  119,  119,  0,  0,  0,  1,  176,  0,  0,  0,  0,  0,  0,  0xC,  0,  0x20,  0,  0,  0,  88,  0,  0,  0};
uint8_t Rbuffer_1[27]= {0};
uint8_t Rbuffer_2[27]= {0};
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

///VARIABLES DECK1
uint16_t flt_adr_1 = 1;
uint16_t PULSE_COUNT_1 = 0;
uint16_t flt_adr_CUE_1 = 1;
uint16_t PREV_COUNT_1 = 0;
uint8_t PITCH_MSB_1 = 0;
uint8_t PITCH_LSB_1 = 0;
uint8_t LOOP_IN_1_pressed = 0;
uint8_t LOOP_OUT_1_pressed = 0;
uint8_t LOOP_EXIT_1_pressed = 0;
uint8_t PLAY_BUTTON_1_pressed = 0;
uint8_t CUE_BUTTON_1_pressed = 0;
uint8_t JOG_1_pressed = 0;
uint8_t divider_1 = 0;
uint8_t MP3_1_pressed = 0;
uint8_t TEMPO_RESET_1_pressed = 0;
uint8_t MASTER_TEMPO_1_pressed = 0;
uint8_t TEMPO_1_pressed = 0;
uint8_t TRACK_BACK_BUTTON_1_pressed = 0;
uint8_t TRACK_FORWARD_BUTTON_1_pressed=0;
uint8_t SEARCH_BACK_BUTTON_1_pressed=0;
uint8_t SEARCH_FORWARD_BUTTON_1_pressed=0;
uint8_t MEMORY_BACK_BUTTON_1_pressed=0;
uint8_t MEMORY_FORWARD_BUTTON_1_pressed=0;
uint8_t MEMORY_SET_BUTTON_1_pressed=0;
uint8_t MEMORY_DEL_BUTTON_1_pressed=0;
uint8_t FOLDER_FORWARD_pressed_1=0;
uint8_t FOLDER_BACK_pressed_1=0;
uint8_t BROWSE_FORWARD_pressed_1=0;
uint8_t BROWSE_BACK_pressed_1=0;
uint8_t BROWSE_COUNTER_FORWARD_1=1;
uint8_t BROWSE_COUNTER_BACK_1=127;
uint8_t LOAD_1_pressed=0;
uint8_t REC_1_pressed=0;
uint8_t REC_1_set=0;
uint8_t HOT_CUE_A_1_set=0;
uint8_t HOT_CUE_B_1_set=0;
uint8_t HOT_CUE_C_1_set=0;
uint8_t HOTCUE_A_1_pressed=0;
uint8_t HOTCUE_B_1_pressed=0;
uint8_t HOTCUE_C_1_pressed=0;
uint8_t PLAY_1_state = 0;	
uint8_t  TOUCHBRAKE_1=0;
uint8_t  RELEASEBRAKE_1=0;
uint8_t TOUCH_1 = 0;	
uint8_t TOUCH_SET_1 = 0;	

uint8_t REVERSE_1_pressed = 0;	
uint8_t HALFLOOP_1_pressed = 0;	
uint8_t counter_1=0;
uint8_t DECK_1=0;
uint8_t DECK_SET_1=0;
uint8_t SLIP_1_pressed=0;
uint8_t MOVING_1=0;
uint8_t AUTOLOOP_press_1=0;
uint32_t AUTOLOOP_TIMER_1=0;
uint32_t TIMER_LOOP_COUNTER_1=0;
uint8_t AUTOLOOP_ACTIVE_1=0;
uint32_t PREV_TIMER_1=0;
uint32_t TIMER_1=0;

////DECK 2

uint16_t flt_adr_2 = 1;
uint16_t PULSE_COUNT_2 = 0;
uint16_t flt_adr_CUE_2 = 1;
uint16_t PREV_COUNT_2 = 0;
uint8_t PITCH_MSB_2 = 0;
uint8_t PITCH_LSB_2 = 0;
uint8_t LOOP_IN_2_pressed = 0;
uint8_t LOOP_OUT_2_pressed = 0;
uint8_t LOOP_EXIT_2_pressed = 0;
uint8_t PLAY_BUTTON_2_pressed = 0;
uint8_t CUE_BUTTON_2_pressed = 0;
uint8_t JOG_2_pressed = 0;
uint8_t divider_2 = 0;
uint8_t MP3_2_pressed = 0;
uint8_t TEMPO_RESET_2_pressed = 0;
uint8_t MASTER_TEMPO_2_pressed = 0;
uint8_t TEMPO_2_pressed = 0;
uint8_t TRACK_BACK_BUTTON_2_pressed = 0;
uint8_t TRACK_FORWARD_BUTTON_2_pressed=0;
uint8_t SEARCH_BACK_BUTTON_2_pressed=0;
uint8_t SEARCH_FORWARD_BUTTON_2_pressed=0;
uint8_t MEMORY_BACK_BUTTON_2_pressed=0;
uint8_t MEMORY_FORWARD_BUTTON_2_pressed=0;
uint8_t MEMORY_SET_BUTTON_2_pressed=0;
uint8_t MEMORY_DEL_BUTTON_2_pressed=0;
uint8_t FOLDER_FORWARD_pressed_2=0;
uint8_t FOLDER_BACK_pressed_2=0;
uint8_t BROWSE_FORWARD_pressed_2=0;
uint8_t BROWSE_BACK_pressed_2=0;
uint8_t BROWSE_COUNTER_FORWARD_2=1;
uint8_t BROWSE_COUNTER_BACK_2=127;
uint8_t LOAD_2_pressed=0;
uint8_t REC_2_pressed=0;
uint8_t REC_2_set=0;
uint8_t HOT_CUE_A_2_set=0;
uint8_t HOT_CUE_B_2_set=0;
uint8_t HOT_CUE_C_2_set=0;
uint8_t HOTCUE_A_2_pressed=0;
uint8_t HOTCUE_B_2_pressed=0;
uint8_t HOTCUE_C_2_pressed=0;
uint8_t PLAY_2_state = 0;	
uint8_t TOUCHBRAKE_2=0;
uint8_t RELEASEBRAKE_2=0;
uint8_t TOUCH_2 = 0;	
uint8_t TOUCH_SET_2 = 0;	
uint8_t SHIFT_1_pressed=0;
uint8_t REVERSE_2_pressed = 0;	
uint8_t HALFLOOP_2_pressed = 0;	
uint8_t counter_2=0;
uint8_t DECK_2=1;
uint8_t DECK_SET_2=0;
uint8_t SLIP_2_pressed=0;
uint8_t MOVING_2=0;
uint8_t AUTOLOOP_press_2=0;
uint32_t AUTOLOOP_TIMER_2=0;
uint32_t TIMER_LOOP_COUNTER_2=0;
uint8_t AUTOLOOP_ACTIVE_2=0;
uint32_t PREV_TIMER_2=0;
uint32_t TIMER_2=0;

//orgineel
//uint16_t flt_adr_2 = 1;
//uint16_t PULSE_COUNT_2 = 0;
//uint16_t PREV_COUNT_2 = 0;
//uint8_t PLAY_BUTTON_2_pressed = 0;
//uint8_t CUE_BUTTON_2_pressed = 0;
//uint8_t JOG_2_pressed = 0;
//uint8_t divider_2 = 0;

uint8_t need_send_message = 0;




/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void DrawJog(uint8_t pos);
void CheckTXCRC_1(void);
void CheckTXCRC_2(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	CheckTXCRC_1();
	CheckTXCRC_2();
	HAL_SPI_TransmitReceive_DMA(&hspi1, Tbuffer_1, Rbuffer_1, 27);
	HAL_SPI_TransmitReceive_DMA(&hspi2, Tbuffer_2, Rbuffer_2, 27);
	
	SSD1306_Init();
	SSD1306_UpdateLOGO();		
	HAL_Delay(300);
	SSD1306_DrawFilledRectangle(0, 0, 128, 64, SSD1306_COLOR_BLACK);	
	SSD1306_UpdateScreen();	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  if (USB_Tx_State == USB_TX_CONTINUE) 
			{	
      USBD_MIDI_SendPacket();
			}
	
	if(have_a_new_data>0)
		{
		if(have_a_new_data>4)
			{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);		
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);		
			}	
	/////////////////////		
	/// DECK 1	RECIEVE ON CDJ	
			
	/// JOG 1		
		if(USB_Rx_Buffer[2]==0 && (USB_Rx_Buffer[1]&0x6F)==0x2B && USB_Rx_Buffer[0]==0x0B)
			{
			flt_adr_1 = (135*USB_Rx_Buffer[3])/72;		//SX MIDI OUT CSV SAYS 72
			flt_adr_1+=116;
			flt_adr_1 = (flt_adr_1%134)+1;
			}	
				/// JOG 2
			else if(USB_Rx_Buffer[2]==1 && (USB_Rx_Buffer[1]&0x6F)==0x2B && USB_Rx_Buffer[0]==0x0B)
			{
			flt_adr_2 = (135*USB_Rx_Buffer[3])/72;		//SX MIDI OUT CSV SAYS 72
			flt_adr_2+=116;
			flt_adr_2 = (flt_adr_2%134)+1;
			}		
	/// CUE
		else if(USB_Rx_Buffer[2]==0x0C && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[17] &=0xFD;
				}
			else
				{
				Tbuffer_1[17] |=0x02;	
				}
			}
		/// PLAY
		else if(USB_Rx_Buffer[2]==0x0B && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[17] &=0xFE;
				}
			else
				{
				Tbuffer_1[17] |=0x01;	
				}
			}
			//MT 1 [7package][19 byte][bit3] =MT red diode on button 901a
		else if(USB_Rx_Buffer[2]==0x1A && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[19] &=0xF7; //LED
				//Tbuffer_1[12] &=0x7F; //DISPLAY
				}
			else
				{
				Tbuffer_1[19] |=0x08;	//LED
				//Tbuffer_1[12] &=0x80; //DISPLAY
				}
			}
		//REC MODE = DELETE (RED LEDS)
		else if (REC_1_pressed!=0 && REC_1_set ==0){
			Tbuffer_1[17] |=0x40;	//A RED ON
			Tbuffer_1[18] |=0x02;	//B	RED ON
			Tbuffer_1[18] |=0x16;	//C	RED ON
			Tbuffer_1[17] &=0x7F; //A GREEN LED OFF
			Tbuffer_1[18] &=0xFB; //B GREEN LED OFF
			Tbuffer_1[18] &=0xDF; //C GREEN LED OFF
			Tbuffer_1[18] &=0xFE; //A ORANGE LED OFF
			Tbuffer_1[18] &=0xF7; //B ORANGE LED OFF
			Tbuffer_1[18] &=0xBF; //C ORANGE LED OFF
			REC_1_set=1;
		}		
		else if (REC_1_pressed!=1 && REC_1_set==1){
			Tbuffer_1[17] &=0xBF;	//A RED OFF
			Tbuffer_1[18] &=0xFD;	//B	RED OFF
			Tbuffer_1[18] &=0xE9;	//C	RED OFF
			if (HOT_CUE_A_1_set==1){Tbuffer_1[17] |=0x80;} else {Tbuffer_1[18] |=0x01;}	//GREEN OR ORANGE LED ON}
			if (HOT_CUE_B_1_set==1){Tbuffer_1[18] |=0x04;} else {Tbuffer_1[18] |=0x08;}	//GREEN OR ORANGE LED ON}
			if (HOT_CUE_C_1_set==1){Tbuffer_1[18] |=0x20;} else {Tbuffer_1[18] |=0x40;}	//GREEN OR ORANGE LED ON}
			REC_1_set=0;
		}
		//HOT CUE A 9700 SET
		else if(USB_Rx_Buffer[2]==0x00 && (USB_Rx_Buffer[1]&0x6F)==7 && USB_Rx_Buffer[0]==0x09 )
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[17] &=0x7F; //GREEN LED OFF
				Tbuffer_1[18] |=0x01;	//ORANGE LED ON
				HOT_CUE_A_1_set=0;
				}
			else
				{
				Tbuffer_1[18] &=0xFE; //ORANGE LED OFF
				Tbuffer_1[17] |=0x80;	//GREEN LED ON
				HOT_CUE_A_1_set=1;
				}
			}
			//HOT CUE B 9701 SET
		else if(USB_Rx_Buffer[2]==0x01 && (USB_Rx_Buffer[1]&0x6F)==7 && USB_Rx_Buffer[0]==0x09 )
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[18] &=0xFB; //GREEN LED OFF
				Tbuffer_1[18] |=0x08;	//ORANGE LED ON
				HOT_CUE_B_1_set=0;
				}
			else
				{
				Tbuffer_1[18] &=0xF7; //ORANGE LED OFF
				Tbuffer_1[18] |=0x04;	//GREEN LED ON
				HOT_CUE_B_1_set=1;
				}
			}
			//HOT CUE C 9702 SET
		else if(USB_Rx_Buffer[2]==0x02 && (USB_Rx_Buffer[1]&0x6F)==7 && USB_Rx_Buffer[0]==0x09 )
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[18] &=0xDF; //GREEN LED OFF
				Tbuffer_1[18] |=0x40;	//ORANGE LED ON
				HOT_CUE_C_1_set=0;
				}
			else
				{
				Tbuffer_1[18] &=0xBF; //ORANGE LED OFF
				Tbuffer_1[18] |=0x20;	//GREEN LED ON
				HOT_CUE_C_1_set=1;
				}
			}

			//LOOP IN 9010
		else if(USB_Rx_Buffer[2]==0x10 && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[17] &=0xFB;
				}
			else
				{
				Tbuffer_1[17] |=0x04;	
				}
			}
			//LOOP IN 9011
		else if(USB_Rx_Buffer[2]==0x11 && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[17] &=0xF7;
				}
			else
				{
				Tbuffer_1[17] |=0x08;	
				}
			}
			//LOOP EXIT 904D
		else if(USB_Rx_Buffer[2]==0x4D && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[17] &=0xE9;
				}
			else
				{
				Tbuffer_1[17] |=0x16;	
				}
			}
		
			//REVERSE red diode on button 9038
		else if(USB_Rx_Buffer[2]==0x38 && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[17] &=0xDF;
				}
			else
				{
				Tbuffer_1[17] |=0x20;	
				}
			} 			
			//SLIP MODE ON (CDJ GREEN LED)
		else if(USB_Rx_Buffer[2]==0x40 && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[19] &=0xBF;
				}
			else
				{
				Tbuffer_1[19] |=0x40;	
				}
			}  			
			//TEMPO RESET 9061
		else if(USB_Rx_Buffer[2]==0x61 && (USB_Rx_Buffer[1]&0x6F)==0 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_1[19] &=0xE9;
				}
			else
				{
				Tbuffer_1[19] |=0x16;	
				}
			} 
			
			//TOUCH 
			else if((Rbuffer_1[12]&0x20)==0 && TOUCH_SET_1==0)
					{
					Tbuffer_1[23] &=0xDF;
					 TOUCH_SET_1=1;
					}
			else if((Rbuffer_1[12]&0x20)!=0  && TOUCH_SET_1==1)
					{
					Tbuffer_1[23] |=0x20;	
					 TOUCH_SET_1=0;
					}
				
	////////////////
	////DECK 2 RECIEVE ON CDJ
		
		else if(USB_Rx_Buffer[2]==0x0C && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[17] &=0xFD;
				}
			else
				{
				Tbuffer_2[17] |=0x02;	
				}
			}
		else if(USB_Rx_Buffer[2]==0x0B && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[17] &=0xFE;
				}
			else
				{
				Tbuffer_2[17] |=0x01;	
				}
			}
		//MT 1 [7package][19 byte][bit3] =MT red diode on button 901a
		else if(USB_Rx_Buffer[2]==0x1A && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[19] &=0xF7; //LED
				//Tbuffer_2[12] &=0x7F; //DISPLAY
				}
			else
				{
				Tbuffer_2[19] |=0x08;	//LED
				//Tbuffer_2[12] &=0x80; //DISPLAY
				}
			}
		//REC MODE = DELETE (RED LEDS)
		else if (REC_2_pressed!=0 && REC_2_set ==0){
			Tbuffer_2[17] |=0x40;	//A RED ON
			Tbuffer_2[18] |=0x02;	//B	RED ON
			Tbuffer_2[18] |=0x16;	//C	RED ON
			Tbuffer_2[17] &=0x7F; //A GREEN LED OFF
			Tbuffer_2[18] &=0xFB; //B GREEN LED OFF
			Tbuffer_2[18] &=0xDF; //C GREEN LED OFF
			Tbuffer_2[18] &=0xFE; //A ORANGE LED OFF
			Tbuffer_2[18] &=0xF7; //B ORANGE LED OFF
			Tbuffer_2[18] &=0xBF; //C ORANGE LED OFF
			REC_2_set=1;
		}		
		else if (REC_2_pressed!=1 && REC_2_set==1){
			Tbuffer_2[17] &=0xBF;	//A RED OFF
			Tbuffer_2[18] &=0xFD;	//B	RED OFF
			Tbuffer_2[18] &=0xE9;	//C	RED OFF
			if (HOT_CUE_A_2_set==1){Tbuffer_2[17] |=0x80;} else {Tbuffer_2[18] |=0x01;}	//GREEN OR ORANGE LED ON}
			if (HOT_CUE_B_2_set==1){Tbuffer_2[18] |=0x04;} else {Tbuffer_2[18] |=0x08;}	//GREEN OR ORANGE LED ON}
			if (HOT_CUE_C_2_set==1){Tbuffer_2[18] |=0x20;} else {Tbuffer_2[18] |=0x40;}	//GREEN OR ORANGE LED ON}
			REC_2_set=0;
		}
		//HOT CUE A 9700 SET
		else if(USB_Rx_Buffer[2]==0x00 && (USB_Rx_Buffer[1]&0x6F)==8 && USB_Rx_Buffer[0]==0x09 )
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[17] &=0x7F; //GREEN LED OFF
				Tbuffer_2[18] |=0x01;	//ORANGE LED ON
				HOT_CUE_A_2_set=0;
				}
			else
				{
				Tbuffer_2[18] &=0xFE; //ORANGE LED OFF
				Tbuffer_2[17] |=0x80;	//GREEN LED ON
				HOT_CUE_A_2_set=1;
				}
			}
			//HOT CUE B 9701 SET
		else if(USB_Rx_Buffer[2]==0x01 && (USB_Rx_Buffer[1]&0x6F)==8 && USB_Rx_Buffer[0]==0x09 )
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[18] &=0xFB; //GREEN LED OFF
				Tbuffer_2[18] |=0x08;	//ORANGE LED ON
				HOT_CUE_B_2_set=0;
				}
			else
				{
				Tbuffer_2[18] &=0xF7; //ORANGE LED OFF
				Tbuffer_2[18] |=0x04;	//GREEN LED ON
				HOT_CUE_B_2_set=1;
				}
			}
			//HOT CUE C 9702 SET
		else if(USB_Rx_Buffer[2]==0x02 && (USB_Rx_Buffer[1]&0x6F)==8 && USB_Rx_Buffer[0]==0x09 )
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[18] &=0xDF; //GREEN LED OFF
				Tbuffer_2[18] |=0x40;	//ORANGE LED ON
				HOT_CUE_C_2_set=0;
				}
			else
				{
				Tbuffer_2[18] &=0xBF; //ORANGE LED OFF
				Tbuffer_2[18] |=0x20;	//GREEN LED ON
				HOT_CUE_C_2_set=1;
				}
			}

			//LOOP IN 9010
		else if(USB_Rx_Buffer[2]==0x10 && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[17] &=0xFB;
				}
			else
				{
				Tbuffer_2[17] |=0x04;	
				}
			}
			//LOOP IN 9011
		else if(USB_Rx_Buffer[2]==0x11 && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[17] &=0xF7;
				}
			else
				{
				Tbuffer_2[17] |=0x08;	
				}
			}
			//LOOP EXIT 904D
		else if(USB_Rx_Buffer[2]==0x4D && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[17] &=0xE9;
				}
			else
				{
				Tbuffer_2[17] |=0x16;	
				}
			}
		
			//REVERSE red diode on button 9038
		else if(USB_Rx_Buffer[2]==0x38 && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[17] &=0xDF;
				}
			else
				{
				Tbuffer_2[17] |=0x20;	
				}
			} 			
			//SLIP MODE ON (CDJ GREEN LED)
		else if(USB_Rx_Buffer[2]==0x40 && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[19] &=0xBF;
				}
			else
				{
				Tbuffer_2[19] |=0x40;	
				}
			}  			
			//TEMPO RESET 9061
		else if(USB_Rx_Buffer[2]==0x61 && (USB_Rx_Buffer[1]&0x6F)==1 && USB_Rx_Buffer[0]==0x09)
			{	
			if(USB_Rx_Buffer[3]==0)
				{
				Tbuffer_2[19] &=0xE9;
				}
			else
				{
				Tbuffer_2[19] |=0x16;	
				}
			} 
			
			//TOUCH 
			else if((Rbuffer_2[12]&0x20)==0  && TOUCH_SET_2==0)
					{
					Tbuffer_2[23] &=0xDF;
						 TOUCH_SET_2=1;
					}
			else if((Rbuffer_2[12]&0x20)!=0 && TOUCH_SET_2==1)
					{
					Tbuffer_2[23] |=0x20;	
						 TOUCH_SET_2=0;
					}
		///END RECIEVE BYTES	///END RECIEVE BYTES	///END RECIEVE BYTES	///END RECIEVE BYTES	///END RECIEVE BYTES	///END RECIEVE BYTES	///END RECIEVE BYTES
		else
			{
					//					
			}
		have_a_new_data = 0;		
		}

			
	if(need_send_message)
		{
		USBD_SendMidiMessages();		
		need_send_message = 0;	
		}
			
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1 && button_pressed==0)
		{
		//USBD_AddNoteOn(0, 0x90+DECK_1, 0x36, 0x7F);
		USBD_AddNoteOn(0, 0x91, 0x0C, 0x7F);	
		USBD_SendMidiMessages();			
		button_pressed = 1;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);		
		}
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0 && button_pressed==1)
		{
		//USBD_AddNoteOn(0, 0x90+DECK_1, 0x36, 0x00);
		USBD_AddNoteOn(0, 0x91, 0x0C, 0x00);		
		USBD_SendMidiMessages();		
		button_pressed = 0;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);		
		}
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1 && button_pressed==1)
		{
		HAL_Delay(1);	
		USBD_AddNoteOn(0, 0xB0, 0x22, 69);
		USBD_SendMidiMessages();	
		HAL_Delay(1);	
		}
	scan_note++;

			
		
	if(need_update)
		{
		SSD1306_UpdateScreen();
		need_update = 0;	
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void DrawJog(uint8_t pos)
	{
	SSD1306_DrawFilledRectangle(0, 0, 18, 18, SSD1306_COLOR_BLACK);	
	uint8_t x0, y0, x1, y1;
	if(pos>72)
		{
		return;	
		}
	if(pos>63)
		{
		x0 = 8;
		y0 = 8;
		y1 = 0;
		x1 = pos-64;
		}
	else if(pos>54)
		{
		x0 = 8;
		y0 = 8;
		y1 = 63-pos;
		x1 = 0;
		}
	else if(pos>45)
		{
		x0 = 8;
		y0 = 9;
		y1 = 63-pos;
		x1 = 0;
		}
	else if(pos>36)
		{
		x0 = 8;
		y0 = 9;
		y1 = 17;
		x1 = 45-pos;
		}		
	else if(pos>27)
		{
		x0 = 9;
		y0 = 9;
		y1 = 17;
		x1 = 45-pos;
		}	
	else if(pos>18)
		{
		x0 = 9;
		y0 = 9;
		y1 = pos-10;
		x1 = 17;
		}	
	else if(pos>9)
		{
		x0 = 9;
		y0 = 8;
		y1 = pos-10;
		x1 = 17;
		}	
	else if(pos>=1)
		{
		x0 = 9;
		y0 = 8;
		y1 = 0;
		x1 = pos+8;
		}
		
	SSD1306_DrawLine(x0, y0, x1, y1, SSD1306_COLOR_WHITE);
	need_update = 1;	
	return;	
	}
	

//////////////////////////////////////
//
//		DECK 1
//
void DMA2_Stream3_IRQHandler(void)
	{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
	
	uint16_t SCRTCH;	
		
			
	if((Rbuffer_1[12]&0x1)==0 && DECK_SET_1==0){
		DECK_1=2;
		DECK_SET_1=1;
	}	
	else if((Rbuffer_1[12]&0x1)==1 && DECK_SET_1==1){
		DECK_1=0;
		DECK_SET_1=0;
	}		
	else if((Rbuffer_1[14]&0x1) && PLAY_BUTTON_1_pressed==0)								///////////PLAY button
		{
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x0B, 0x7F);
		need_send_message = 1;	
		PLAY_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[14]&0x1)==0 && PLAY_BUTTON_1_pressed==1)
		{
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x0B, 0x00);	
		need_send_message = 1;		
		PLAY_BUTTON_1_pressed = 0;	
		//PLAY_1_state++;		
		}
	else if((Rbuffer_1[14]&0x2) && CUE_BUTTON_1_pressed==0)									///////////CUE button
		{
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x0C, 0x7F);
		need_send_message = 1;		
		CUE_BUTTON_1_pressed = 1;	
			if(Tbuffer_1[17] != 0x01){
				flt_adr_CUE_1 = (flt_adr_1*0.62965);															//SET JOG CUE LIGHT ->CDJ has 85 positions 0 = no CUE		
			}
		}
	else if((Rbuffer_1[14]&0x2)==0 && CUE_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x0C, 0x00);
		need_send_message = 1;		
		CUE_BUTTON_1_pressed = 0;	
			
		}
////////////// 
else if((Rbuffer_1[14]&0x4) && LOOP_IN_1_pressed==0)																	///////////LOOP IN 
		{
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x10, 0x7F);
		need_send_message = 1;		
		LOOP_IN_1_pressed = 1;	
		}
	else if((Rbuffer_1[14]&0x4)==0 && LOOP_IN_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x10, 0x00);
		need_send_message = 1;		
		LOOP_IN_1_pressed = 0;	
		}
		
	else if((Rbuffer_1[14]&0x08) && LOOP_OUT_1_pressed==0)																///////////LOOP OUT
		{
			
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x11, 0x7F);
		need_send_message = 1;		
		LOOP_OUT_1_pressed = 1;	
		}
	else if((Rbuffer_1[14]&0x08)==0 && LOOP_OUT_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x11, 0x00);
		need_send_message = 1;		
		LOOP_OUT_1_pressed = 0;	
		}
		else if((Rbuffer_1[18]&0x20) && SHIFT_1_pressed==0){
			SHIFT_1_pressed=1;
		}
		else if((Rbuffer_1[18]&0x20) && SHIFT_1_pressed==1){
			SHIFT_1_pressed=0;
		}
		
		///LOOP EXIT AND AUTOLOOP NEED TO SET WHEN PRESSED 
		///CUE SENDS EXIT
		///LOOP IN SENDS EXIT
		else if((Rbuffer_1[14]&0x16) && LOOP_EXIT_1_pressed==0 && CUE_BUTTON_1_pressed != 1 && LOOP_IN_1_pressed != 1)						   								
																																									///////////  CDJ				 -> DDJ SX
		{																																							///////////  RELOOP/EXIT -> SHIFT LOOP OUT
			USBD_AddNoteOn(0, 0x90+DECK_1, 0x4D, 0x7F);
			need_send_message = 1;		
			LOOP_EXIT_1_pressed = 1;	
		}
	else if((Rbuffer_1[14]&0x16)==0 && LOOP_EXIT_1_pressed==1)
		{	
			USBD_AddNoteOn(0, 0x90+DECK_1, 0x4D, 0x00); 
			need_send_message = 1;		
			LOOP_EXIT_1_pressed = 0;
		}
	else if (	SHIFT_1_pressed==1 && LOOP_EXIT_1_pressed ==1)			///////////  TIME MODE = SHIFT + exit -> HALF LOOP
			{
				USBD_AddNoteOn(0, 0x90+DECK_1, 0x12, 0x7F);	
				need_send_message = 1;
				HALFLOOP_1_pressed=1;	
			}
	else if (SHIFT_1_pressed==0 && LOOP_EXIT_1_pressed ==0 && HALFLOOP_1_pressed==1){
		
				USBD_AddNoteOn(0, 0x90+DECK_1, 0x12, 0x00);	
				need_send_message = 1;
				HALFLOOP_1_pressed=0;	
	}
	
	///////////////////		AUTO LOOP
	else if((Rbuffer_1[18]&0x40)!=0 && AUTOLOOP_press_1==0)						   							///////////  CDJ				 -> DDJ SX
		{																																								///////////  TEXT MODE	 -> AUTOLOOP 4
			AUTOLOOP_TIMER_1=HAL_GetTick();						//timer for autoloop								///////////	 LONG PRESS	 -> AUTOLOOP 8
			USBD_AddNoteOn(0, 0x90+DECK_1, 0x14, 0x7F);				
			need_send_message = 1;		
			AUTOLOOP_press_1= 1;
		}	
	else if (HAL_GetTick()-AUTOLOOP_TIMER_1>=1000 && (Rbuffer_1[18]&0x40)!=0 && AUTOLOOP_ACTIVE_1==0)
				{
					USBD_AddNoteOn(0, 0x90+DECK_1, 0x13, 0x7F);	
					AUTOLOOP_ACTIVE_1=1;
					need_send_message = 1;		
					AUTOLOOP_TIMER_1=0;
				}
	else if((Rbuffer_1[18]&0x40)==0 && AUTOLOOP_press_1==1)
		{	
				if (AUTOLOOP_ACTIVE_1==1)
				{
					USBD_AddNoteOn(0, 0x90+DECK_1, 0x13, 0x00);	
					AUTOLOOP_ACTIVE_1=0;
				}
				else{
					USBD_AddNoteOn(0, 0x90+DECK_1, 0x14, 0x00);	
				}		
			need_send_message = 1;		
			AUTOLOOP_press_1=0;
		}
/////////////////////		AUTO LOOP

else if((Rbuffer_1[17]&0x20) && TEMPO_RESET_1_pressed==0)													///////////  TEMPO RESET
		{																																							///////////  SOLUTION FOR SX
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x60, 0x7F);
		need_send_message = 1;		
		TEMPO_RESET_1_pressed = 1;	
		}
	else if((Rbuffer_1[17]&0x20)==0 && TEMPO_RESET_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x60, 0x00);
		need_send_message = 1;		
		TEMPO_RESET_1_pressed = 0;	
		}
//SLIP MODE WITH JOG SELECT VINYL CDJ BUTTON
	else if((Rbuffer_1[17]&0x04) && SLIP_1_pressed==0)														///////////	SELECT = SLIP MODE
		{	
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x40, 0x7F);
		need_send_message = 1;		
		SLIP_1_pressed = 1;	
		}
	else if((Rbuffer_1[17]&0x04)==0 && SLIP_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x40, 0x00);
		need_send_message = 1;		
		SLIP_1_pressed = 0;	
		}
///////////  MASTER TEMPO																												/////////// MASTER TEMPO
	else if((Rbuffer_1[17]&0x16) && MASTER_TEMPO_1_pressed==0 && (Rbuffer_1[17]&0x2)==0 && FOLDER_FORWARD_pressed_1!=1 && SLIP_1_pressed != 1)		
		{																																						///////////  
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x1A, 0x7F);
		need_send_message = 1;		
		MASTER_TEMPO_1_pressed = 1;	
		}
	else if((Rbuffer_1[17]&0x16)==0 && MASTER_TEMPO_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x1A, 0x00);
		need_send_message = 1;		
		MASTER_TEMPO_1_pressed = 0;	
		}				
		
	else if((Rbuffer_1[17]&0x8) && TEMPO_1_pressed==0 && SLIP_1_pressed!=1)						   								///////////  TEMPO RANGE
		{																																							///////////  
		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x60, 0x7F);
		need_send_message = 1;		
		TEMPO_1_pressed = 1;	
		}
	else if((Rbuffer_1[17]&0x8)==0 && TEMPO_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x60, 0x00);
		need_send_message = 1;		
		TEMPO_1_pressed = 0;	
		}		

	else if((Rbuffer_1[12]&0x02)!=0 && REVERSE_1_pressed==0)						   				///////////  REVERSE
		{																																								  
			USBD_AddNoteOn(0, 0x90+DECK_1, 0x38, 0x7F);
			need_send_message = 1;
			USBD_AddNoteOn(0, 0x90+DECK_1, 0x38, 0x00);
			need_send_message = 1;		
		REVERSE_1_pressed = 1;	
		}
	else if((Rbuffer_1[12]&0x02)==0 && REVERSE_1_pressed==1)
		{		
			USBD_AddNoteOn(0, 0x90+DECK_1, 0x38, 0x7F);
			need_send_message = 1;
			USBD_AddNoteOn(0, 0x90+DECK_1, 0x38, 0x00);
			need_send_message = 1;		
		REVERSE_1_pressed = 0;	
		}		

	
	else if((Rbuffer_1[16]&0x2) && TRACK_BACK_BUTTON_1_pressed==0)														///////////			<< Track search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x71, 0x7F);
		need_send_message = 1;		
		TRACK_BACK_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[16]&0x2)==0 && TRACK_BACK_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x71, 0x00);
		need_send_message = 1;		
		TRACK_BACK_BUTTON_1_pressed = 0;	
		}

	else if((Rbuffer_1[16]&0x4) && TRACK_FORWARD_BUTTON_1_pressed==0)													///////////			>> Track search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x70, 0x7F);
		need_send_message = 1;		
		TRACK_FORWARD_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[16]&0x4)==0 && TRACK_FORWARD_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x70, 0x00);
		need_send_message = 1;		
		TRACK_FORWARD_BUTTON_1_pressed = 0;	
		}


	else if((Rbuffer_1[16]&0x08) && SEARCH_BACK_BUTTON_1_pressed==0)														///////////			<< search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x73, 0x7F);
		need_send_message = 1;		
		SEARCH_BACK_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[16]&0x08)==0 && SEARCH_BACK_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x73, 0x00);
		need_send_message = 1;		
		SEARCH_BACK_BUTTON_1_pressed = 0;	
		}
	else if((Rbuffer_1[16]&0x16) && SEARCH_FORWARD_BUTTON_1_pressed==0 && TRACK_BACK_BUTTON_1_pressed !=1 && TRACK_FORWARD_BUTTON_1_pressed != 1 )	
																																																///////////			>> search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x74, 0x7F);
		need_send_message = 1;		
		SEARCH_FORWARD_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[16]&0x16)==0 && SEARCH_FORWARD_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x74, 0x00);
		need_send_message = 1;		
		SEARCH_FORWARD_BUTTON_1_pressed = 0;	
		}
//HOTCUE RECMODE 901B//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE
//REC MODE = DELETE (RED LEDS)
		else if ((Rbuffer_1[16]&0x01)!=0 && REC_1_pressed==0){
			REC_1_pressed=1;
		}
		else if ((Rbuffer_1[16]&0x01)==0 && REC_1_pressed==1){
			REC_1_pressed=0;
		}
//HOTCUE A 9700
else if((Rbuffer_1[14]&0x20) && HOTCUE_A_1_pressed==0  )																																													
		{	
		if (REC_1_pressed==1){USBD_AddNoteOn(0, 0x97, 0x08, 0x7F);} else {USBD_AddNoteOn(0, 0x97, 0x00, 0x7F);}
		need_send_message = 1;		
		HOTCUE_A_1_pressed = 1;	
		}
else if((Rbuffer_1[14]&0x20)==0 && HOTCUE_A_1_pressed==1  )																																													
		{		
		if (REC_1_pressed==1){USBD_AddNoteOn(0, 0x97, 0x08, 0x00);} else {USBD_AddNoteOn(0, 0x97, 0x00, 0x00);}
		need_send_message = 1;		
		HOTCUE_A_1_pressed = 0;	
		}
//HOTCUE B 9701
else if((Rbuffer_1[14]&0x40) && HOTCUE_B_1_pressed==0  )																																												
		{	
		if (REC_1_pressed==1){USBD_AddNoteOn(0, 0x97, 0x09, 0x7F);} else {USBD_AddNoteOn(0, 0x97, 0x01, 0x7F);}
		need_send_message = 1;		
		HOTCUE_B_1_pressed = 1;	
		}
else if((Rbuffer_1[14]&0x40)==0 && HOTCUE_B_1_pressed==1  )																																												
		{		
		if (REC_1_pressed==1){USBD_AddNoteOn(0, 0x97, 0x09, 0x00);} else {USBD_AddNoteOn(0, 0x97, 0x01, 0x00);}
		need_send_message = 1;		
		HOTCUE_B_1_pressed = 0;	
		}
//HOTCUE C 9702
else if((Rbuffer_1[14]&0x80) && HOTCUE_C_1_pressed==0  )																																													
		{	
		if (REC_1_pressed==1){USBD_AddNoteOn(0, 0x97, 0x0A, 0x7F);} else {USBD_AddNoteOn(0, 0x97, 0x02, 0x7F);}
		need_send_message = 1;		
		HOTCUE_C_1_pressed = 1;	
		}
else if((Rbuffer_1[14]&0x80)==0 && HOTCUE_C_1_pressed==1  )																																												
		{		
		if (REC_1_pressed==1){USBD_AddNoteOn(0, 0x97, 0x0A, 0x00);} else {USBD_AddNoteOn(0, 0x97, 0x02, 0x00);}
		need_send_message = 1;		
		HOTCUE_C_1_pressed = 0;	
		}
/////////
	else if((Rbuffer_1[18]&0x02) && MEMORY_BACK_BUTTON_1_pressed==0)																///////////			<< CALL MEMORY
		{	
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x24, 0x7F);
		need_send_message = 1;		
		MEMORY_BACK_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[18]&0x02)==0 && MEMORY_BACK_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x24, 0x00);
		need_send_message = 1;		
		MEMORY_BACK_BUTTON_1_pressed = 0;	
		flt_adr_CUE_1 = (flt_adr_1*0.62965);								//SET JOG CUE LIGHT ->CDJ has 85 positions 0 = no CUE		
		}
	else if((Rbuffer_1[18]&0x01) && MEMORY_FORWARD_BUTTON_1_pressed==0)														///////////			>> CALL MEMORY
		{	
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x2C, 0x7F);
		need_send_message = 1;		
		MEMORY_FORWARD_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[18]&0x01)==0 && MEMORY_FORWARD_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x2C, 0x00);
		need_send_message = 1;		
		MEMORY_FORWARD_BUTTON_1_pressed = 0;	
		flt_adr_CUE_1 = (flt_adr_1*0.62965);								//SET JOG CUE LIGHT ->CDJ has 85 positions 0 = no CUE		
		}
	else if((Rbuffer_1[18]&0x04) && MEMORY_SET_BUTTON_1_pressed==0)																///////////			SET MEMORY
		{	
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x1C, 0x7F);
		need_send_message = 1;		
		MEMORY_SET_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[18]&0x04)==0 && MEMORY_SET_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x1C, 0x00);
		need_send_message = 1;		
		MEMORY_SET_BUTTON_1_pressed = 0;	
		}
	else if((Rbuffer_1[18]&0x08) && MEMORY_DEL_BUTTON_1_pressed==0)														///////////			DELETE MEMORY
		{	
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x1D, 0x7F);
		need_send_message = 1;		
		MEMORY_DEL_BUTTON_1_pressed = 1;	
		}
	else if((Rbuffer_1[18]&0x08)==0 && MEMORY_DEL_BUTTON_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x1D, 0x00);
		need_send_message = 1;		
		MEMORY_DEL_BUTTON_1_pressed = 0;	
		}
	
	else if((Rbuffer_1[4] != PITCH_MSB_1) && (Rbuffer_1[5] != PITCH_LSB_1))			///////////			PITCH FADER
	{
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x00, (Rbuffer_1[4]/2));
		PITCH_MSB_1=Rbuffer_1[4];
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x20, (Rbuffer_1[5]/2));
		PITCH_LSB_1=Rbuffer_1[5];
		need_send_message = 1;		
	}	
	else if((Rbuffer_1[2] <= TOUCHBRAKE_1-4) || (Rbuffer_1[2] >= TOUCHBRAKE_1+4))	///////////		TOUCH BRAKE
	{
		if (Rbuffer_1[2] != TOUCHBRAKE_1){
		USBD_AddNoteOn(0, 0xB4+DECK_1, 0x08, (Rbuffer_1[2]/2));
		need_send_message = 1;
		TOUCHBRAKE_1=Rbuffer_1[2];
		}
	}

///////////			LOAD WITH EJECT BUTTON
	else if((Rbuffer_1[18]&0x16) && LOAD_1_pressed==0 && MEMORY_BACK_BUTTON_1_pressed!=1 &&  MEMORY_SET_BUTTON_1_pressed!=1)																																																	
		{	
		USBD_AddNoteOn(0, 0x96, 0x46+DECK_1, 0x7F);
		need_send_message = 1;		
		LOAD_1_pressed = 1;	
		}
	else if((Rbuffer_1[18]&0x16)==0 && LOAD_1_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x96, 0x46+DECK_1, 0x00);
		need_send_message = 1;		
		LOAD_1_pressed = 0;	
		//Tbuffer_1[25] = 137;
		}	

//////////////////BROWSE BUTTONS////////////////////		
		///////SWITCH WINDOW 
	else if((Rbuffer_1[17]&0x1) && FOLDER_BACK_pressed_1==0)///////////	<< FOLDER UP 
		{
			if (SHIFT_1_pressed==1){
				USBD_AddNoteOn(0, 0x96, 0x65, 0x7F);
			}
			else{
				USBD_AddNoteOn(0, 0xB6, 0x40, 127);	//BROSWE Turn counterclockwise: 127~98(0x7F~0x62)
				BROWSE_BACK_pressed_1=1;
			}				
			need_send_message = 1;		
			FOLDER_BACK_pressed_1=1;
		}
	else if((Rbuffer_1[17]&0x1)==0 && FOLDER_BACK_pressed_1==1)
		{		
			if ( BROWSE_BACK_pressed_1!=1){
				USBD_AddNoteOn(0, 0x96, 0x65, 0x00);	
			}

			need_send_message = 1;	
			FOLDER_BACK_pressed_1 = 0;	
			BROWSE_BACK_pressed_1 = 0;
		}	
	
	else if((Rbuffer_1[17]&0x2) && FOLDER_FORWARD_pressed_1==0)///////////	>> FOLDER DOWN
		{
			if (SHIFT_1_pressed==1){
				USBD_AddNoteOn(0, 0x96, 0x41, 0x7F);	
			}
			else{
				USBD_AddNoteOn(0, 0xB6, 0x40, 1);	// BROWSE Turn clockwise: 1~30(0x01~0x1E)
				BROWSE_FORWARD_pressed_1 = 1;	
			}
			need_send_message = 1;		
			FOLDER_FORWARD_pressed_1 = 1;	
		}
	else if((Rbuffer_1[17]&0x2)==0 && FOLDER_FORWARD_pressed_1==1)
		{		
			if (BROWSE_FORWARD_pressed_1 !=1){
				USBD_AddNoteOn(0, 0x96, 0x41, 0x00);	
			}
			need_send_message = 1;	
			FOLDER_FORWARD_pressed_1 = 0;
			BROWSE_FORWARD_pressed_1 = 0;	
		}	
		
		
	/*	
	else if((Rbuffer_1[17]&0x01)!=0 && (Rbuffer_1[18]&0x40)!=0)																			///////////	<< FOLDER + TEXTMODE = NEEDLE SEARCH
		{
				if(NEEDLE>=1){
			NEEDLE--;
		}
		USBD_AddNoteOn(0, 0xB0, 0x03, NEEDLE);	
		need_send_message = 1;		
		}
	else if((Rbuffer_1[17]&0x02)!=0 && (Rbuffer_1[18]&0x40)!=0)																			///////////	>> FOLDER + TEXTMODE = NEEDLE SEARCH
			{
				if(NEEDLE<=126){
			NEEDLE++;
		}
		USBD_AddNoteOn(0, 0xB0, 0x03, NEEDLE);	
		need_send_message = 1;		
		}
	*/
	//////////////

	else if((Rbuffer_1[12]&0x20)!=0 && JOG_1_pressed==0)									// JOG PRESSED
		{
		USBD_AddNoteOn(0, 0x90+DECK_1, 0x36, 0x7F);
		need_send_message = 1;		
		JOG_1_pressed = 1;	
		TOUCH_1=1;
		}
	else if((Rbuffer_1[12]&0x20)==0 && JOG_1_pressed==1)									// JOG RELEASED
		{
				USBD_AddNoteOn(0, 0x90+DECK_1, 0x36, 0x00);
				need_send_message = 1;	
				JOG_1_pressed = 0;	
		}
	
	else if((Rbuffer_1[12]&0x80)!=0){	// ROTATION DETECTED
		MOVING_1=1;	
		PULSE_COUNT_1 = 256*Rbuffer_1[8] + Rbuffer_1[9];
		divider_1++;
		if (divider_1>=2){
			if(Rbuffer_1[12]&0x40)				  //foward
					{
					if(PULSE_COUNT_1>PREV_COUNT_1)
					{
						SCRTCH = (0xFFFF - PULSE_COUNT_1) +	PREV_COUNT_1;
					}
					else
					{
						SCRTCH = PREV_COUNT_1 - PULSE_COUNT_1;	
					}
	
		SCRTCH = SCRTCH>>2;	
		SCRTCH = SCRTCH*4;
		SCRTCH &= 0x3F;	
		SCRTCH = SCRTCH+1;
		if (127-(SCRTCH)>=65 && 127-(SCRTCH)<=125){
				if (TOUCH_1==1){
					USBD_AddNoteOn(0, 0xB0+DECK_1, 0x22, (127-SCRTCH)-1);	
				}
				else{
					USBD_AddNoteOn(0, 0xB0+DECK_1, 0x21, (127-SCRTCH)-1);	
				}
			}//>65 <127
		}//END ROTATION FORWARD
	else	//reverse rotation
	{
		if(PULSE_COUNT_1>PREV_COUNT_1)
			{
				SCRTCH = (0xFFFF - PULSE_COUNT_1) +	PREV_COUNT_1;
			}
			else
			{
				SCRTCH = PREV_COUNT_1 - PULSE_COUNT_1;	
			}
		SCRTCH = SCRTCH>>2;	
		SCRTCH = SCRTCH*4;
		SCRTCH &= 0x3F;	
		if (63-(SCRTCH)>=2 && 63-(SCRTCH)<=63){	
				if (TOUCH_1==1){
					USBD_AddNoteOn(0, 0xB0+DECK_1, 0x22, (64-SCRTCH)-1);		
				}
				else{	
					USBD_AddNoteOn(0, 0xB0+DECK_1, 0x21, (64-SCRTCH)-1);		
				}
			}//45 - 58
		}//END REVERSE ROTATION
		need_send_message = 1;			
		divider_1=0;					
	}//END DIVIDER		
PREV_COUNT_1 = PULSE_COUNT_1;
}

	else if(MOVING_1==1 && (Rbuffer_1[12]&0x80)==0)
		{
		USBD_AddNoteOn(0, 0xB0+DECK_1, 0x21, 64);		
		need_send_message = 1;
		MOVING_1=0;
		TOUCH_1=0;
		}
	
		//BPM 125 Test
				//		Tbuffer_1[6]|=18;
				//		Tbuffer_1[7]|=107;
				//		Tbuffer_1[8]|=91;
		
				//		Tbuffer_1[6]=0x5D;
				//		Tbuffer_1[7]=0x6B;
				//		Tbuffer_1[8]=0x12;
		
	Tbuffer_1[21] = flt_adr_CUE_1;		///JOG CUE LED
	Tbuffer_1[25] = flt_adr_1;				///JOG LED


	CheckTXCRC_1();		
	}

	
//////////////////////////////////////
//
//		DECK 2
//

void DMA1_Stream4_IRQHandler(void)
	{
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
	uint16_t SCRTCH_2;
	if((Rbuffer_2[12]&0x1)==0 && DECK_SET_2==0){
		DECK_2=3;
		DECK_SET_2=1;
	}	
	else if((Rbuffer_2[12]&0x1)==1 && DECK_SET_2==1){
		DECK_2=1;
		DECK_SET_2=0;
	}		
	else if((Rbuffer_2[14]&0x1) && PLAY_BUTTON_2_pressed==0)								///////////PLAY button
		{
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x0B, 0x7F);
		need_send_message = 1;	
		PLAY_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[14]&0x1)==0 && PLAY_BUTTON_2_pressed==1)
		{
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x0B, 0x00);	
		need_send_message = 1;		
		PLAY_BUTTON_2_pressed = 0;	
		//PLAY_2_state++;		
		}
	else if((Rbuffer_2[14]&0x2) && CUE_BUTTON_2_pressed==0)									///////////CUE button
		{
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x0C, 0x7F);
		need_send_message = 1;		
		CUE_BUTTON_2_pressed = 1;	
			if(Tbuffer_2[17] != 0x01){
				flt_adr_CUE_2 = (flt_adr_2*0.62965);															//SET JOG CUE LIGHT ->CDJ has 85 positions 0 = no CUE		
			}
		}
	else if((Rbuffer_2[14]&0x2)==0 && CUE_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x0C, 0x00);
		need_send_message = 1;		
		CUE_BUTTON_2_pressed = 0;	
			
		}
////////////// 
else if((Rbuffer_2[14]&0x4) && LOOP_IN_2_pressed==0)																	///////////LOOP IN 
		{
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x10, 0x7F);
		need_send_message = 1;		
		LOOP_IN_2_pressed = 1;	
		}
	else if((Rbuffer_2[14]&0x4)==0 && LOOP_IN_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x10, 0x00);
		need_send_message = 1;		
		LOOP_IN_2_pressed = 0;	
		}
		
	else if((Rbuffer_2[14]&0x08) && LOOP_OUT_2_pressed==0)																///////////LOOP OUT
		{
			
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x11, 0x7F);
		need_send_message = 1;		
		LOOP_OUT_2_pressed = 1;	
		}
	else if((Rbuffer_2[14]&0x08)==0 && LOOP_OUT_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x11, 0x00);
		need_send_message = 1;		
		LOOP_OUT_2_pressed = 0;	
		}
		
		///LOOP EXIT AND AUTOLOOP NEED TO SET WHEN PRESSED 
		///CUE SENDS EXIT
		///LOOP IN SENDS EXIT
		else if((Rbuffer_2[14]&0x16) && LOOP_EXIT_2_pressed==0 && CUE_BUTTON_2_pressed != 1 && LOOP_IN_2_pressed != 1)						   								
																																									///////////  CDJ				 -> DDJ SX
		{																																							///////////  RELOOP/EXIT -> SHIFT LOOP OUT
				if ((Rbuffer_2[18]&0x20)!=0){																							///////////  TIME MODE = SHIFT -> HALF LOOP
					USBD_AddNoteOn(0, 0x90+DECK_2, 0x12, 0x7F);			
					HALFLOOP_2_pressed=1;	
				}
				else{
					USBD_AddNoteOn(0, 0x90+DECK_2, 0x4D, 0x7F);
				}
			need_send_message = 1;		
			LOOP_EXIT_2_pressed = 1;	
		}
	else if((Rbuffer_2[14]&0x16)==0 && LOOP_EXIT_2_pressed==1)
		{	
			if (HALFLOOP_2_pressed==1){
					USBD_AddNoteOn(0, 0x90+DECK_2, 0x12, 0x00);			
					HALFLOOP_2_pressed=0;	
			}
			else{
				USBD_AddNoteOn(0, 0x90+DECK_2, 0x4D, 0x00); //EXIT LOOP
			}
			need_send_message = 1;		
			LOOP_EXIT_2_pressed = 0;
		}
		
/////////////////////		AUTO LOOP
	else if((Rbuffer_2[18]&0x40)!=0 && AUTOLOOP_press_2==0)						   							///////////  CDJ				 -> DDJ SX
		{																																								///////////  TEXT MODE	 -> AUTOLOOP 4
			AUTOLOOP_TIMER_2=HAL_GetTick();						//timer for autoloop								///////////	 LONG PRESS	 -> AUTOLOOP 8
			USBD_AddNoteOn(0, 0x90+DECK_2, 0x14, 0x7F);				
			need_send_message = 1;		
			AUTOLOOP_press_2= 1;
		}	
	else if (HAL_GetTick()-AUTOLOOP_TIMER_2>=1000 && (Rbuffer_2[18]&0x40)!=0 && AUTOLOOP_ACTIVE_2==0)
				{
					USBD_AddNoteOn(0, 0x90+DECK_2, 0x13, 0x7F);	
					AUTOLOOP_ACTIVE_2=1;
					need_send_message = 1;		
					AUTOLOOP_TIMER_2=0;
				}
	else if((Rbuffer_2[18]&0x40)==0 && AUTOLOOP_press_2==1)
		{	
				if (AUTOLOOP_ACTIVE_2==1)
				{
					USBD_AddNoteOn(0, 0x90+DECK_2, 0x13, 0x00);	
					AUTOLOOP_ACTIVE_2=0;
				}
				else{
					USBD_AddNoteOn(0, 0x90+DECK_2, 0x14, 0x00);	
				}		
			need_send_message = 1;		
			AUTOLOOP_press_2=0;
		}
/////////////////////		AUTO LOOP

else if((Rbuffer_2[17]&0x20) && TEMPO_RESET_2_pressed==0)													///////////  TEMPO RESET
		{																																							///////////  SOLUTION FOR SX
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x60, 0x7F);
		need_send_message = 1;		
		TEMPO_RESET_2_pressed = 1;	
		}
	else if((Rbuffer_2[17]&0x20)==0 && TEMPO_RESET_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x60, 0x00);
		need_send_message = 1;		
		TEMPO_RESET_2_pressed = 0;	
		}
//SLIP MODE WITH JOG SELECT VINYL CDJ BUTTON
	else if((Rbuffer_2[17]&0x04) && SLIP_2_pressed==0 && TEMPO_RESET_2_pressed!=1)	///////////	SELECT = SLIP MODE
		{	
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x40, 0x7F);
		need_send_message = 1;		
		SLIP_2_pressed = 1;	
		}
	else if((Rbuffer_2[17]&0x04)==0 && SLIP_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x40, 0x00);
		need_send_message = 1;		
		SLIP_2_pressed = 0;	
		}
///////////  MASTER TEMPO																												/////////// MASTER TEMPO
	else if((Rbuffer_2[17]&0x16) && MASTER_TEMPO_2_pressed==0 && (Rbuffer_2[17]&0x2)==0 && FOLDER_FORWARD_pressed_2!=1 && SLIP_2_pressed != 1)		
		{																																						///////////  
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x1A, 0x7F);
		need_send_message = 1;		
		MASTER_TEMPO_2_pressed = 1;	
		}
	else if((Rbuffer_2[17]&0x16)==0 && MASTER_TEMPO_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x1A, 0x00);
		need_send_message = 1;		
		MASTER_TEMPO_2_pressed = 0;	
		}				
		
	else if((Rbuffer_2[17]&0x8) && TEMPO_2_pressed==0 && SLIP_2_pressed!=1)						   								///////////  TEMPO RANGE
		{																																							///////////  
		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x60, 0x7F);
		need_send_message = 1;		
		TEMPO_2_pressed = 1;	
		}
	else if((Rbuffer_2[17]&0x8)==0 && TEMPO_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x60, 0x00);
		need_send_message = 1;		
		TEMPO_2_pressed = 0;	
		}		

	else if((Rbuffer_2[12]&0x02)!=0 && REVERSE_2_pressed==0)						   				///////////  REVERSE
		{																																								  
			USBD_AddNoteOn(0, 0x90+DECK_2, 0x38, 0x7F);
			need_send_message = 1;
			USBD_AddNoteOn(0, 0x90+DECK_2, 0x38, 0x00);
			need_send_message = 1;		
		REVERSE_2_pressed = 1;	
		}
	else if((Rbuffer_2[12]&0x02)==0 && REVERSE_2_pressed==1)
		{		
			USBD_AddNoteOn(0, 0x90+DECK_2, 0x38, 0x7F);
			need_send_message = 1;
			USBD_AddNoteOn(0, 0x90+DECK_2, 0x38, 0x00);
			need_send_message = 1;		
		REVERSE_2_pressed = 0;	
		}		

	
	else if((Rbuffer_2[16]&0x2) && TRACK_BACK_BUTTON_2_pressed==0)														///////////			<< Track search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x71, 0x7F);
		need_send_message = 1;		
		TRACK_BACK_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[16]&0x2)==0 && TRACK_BACK_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x71, 0x00);
		need_send_message = 1;		
		TRACK_BACK_BUTTON_2_pressed = 0;	
		}

	else if((Rbuffer_2[16]&0x4) && TRACK_FORWARD_BUTTON_2_pressed==0)													///////////			>> Track search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x70, 0x7F);
		need_send_message = 1;		
		TRACK_FORWARD_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[16]&0x4)==0 && TRACK_FORWARD_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x70, 0x00);
		need_send_message = 1;		
		TRACK_FORWARD_BUTTON_2_pressed = 0;	
		}


	else if((Rbuffer_2[16]&0x08) && SEARCH_BACK_BUTTON_2_pressed==0)														///////////			<< search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x73, 0x7F);
		need_send_message = 1;		
		SEARCH_BACK_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[16]&0x08)==0 && SEARCH_BACK_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x73, 0x00);
		need_send_message = 1;		
		SEARCH_BACK_BUTTON_2_pressed = 0;	
		}
	else if((Rbuffer_2[16]&0x16) && SEARCH_FORWARD_BUTTON_2_pressed==0 && TRACK_BACK_BUTTON_2_pressed !=1 && TRACK_FORWARD_BUTTON_2_pressed != 1 )	
																																																///////////			>> search
		{	
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x74, 0x7F);
		need_send_message = 1;		
		SEARCH_FORWARD_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[16]&0x16)==0 && SEARCH_FORWARD_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x74, 0x00);
		need_send_message = 1;		
		SEARCH_FORWARD_BUTTON_2_pressed = 0;	
		}
//HOTCUE RECMODE 901B//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE//HOTCUE
//REC MODE = DELETE (RED LEDS)
		else if ((Rbuffer_2[16]&0x01)!=0 && REC_2_pressed==0){
			REC_2_pressed=1;
		}
		else if ((Rbuffer_2[16]&0x01)==0 && REC_2_pressed==1){
			REC_2_pressed=0;
		}
//HOTCUE A 9700
else if((Rbuffer_2[14]&0x20) && HOTCUE_A_2_pressed==0  )																																													
		{	
		if (REC_2_pressed==1){USBD_AddNoteOn(0, 0x98+DECK_2, 0x08, 0x7F);} else {USBD_AddNoteOn(0, 0x97, 0x00, 0x7F);}
		need_send_message = 1;		
		HOTCUE_A_2_pressed = 1;	
		}
else if((Rbuffer_2[14]&0x20)==0 && HOTCUE_A_2_pressed==1  )																																													
		{		
		if (REC_2_pressed==1){USBD_AddNoteOn(0, 0x98+DECK_2, 0x08, 0x00);} else {USBD_AddNoteOn(0, 0x97, 0x00, 0x00);}
		need_send_message = 1;		
		HOTCUE_A_2_pressed = 0;	
		}
//HOTCUE B 9701
else if((Rbuffer_2[14]&0x40) && HOTCUE_B_2_pressed==0  )																																												
		{	
		if (REC_2_pressed==1){USBD_AddNoteOn(0,0x98+DECK_2, 0x09, 0x7F);} else {USBD_AddNoteOn(0, 0x97, 0x01, 0x7F);}
		need_send_message = 1;		
		HOTCUE_B_2_pressed = 1;	
		}
else if((Rbuffer_2[14]&0x40)==0 && HOTCUE_B_2_pressed==1  )																																												
		{		
		if (REC_2_pressed==1){USBD_AddNoteOn(0, 0x98+DECK_2, 0x09, 0x00);} else {USBD_AddNoteOn(0, 0x97, 0x01, 0x00);}
		need_send_message = 1;		
		HOTCUE_B_2_pressed = 0;	
		}
//HOTCUE C 9702
else if((Rbuffer_2[14]&0x80) && HOTCUE_C_2_pressed==0  )																																													
		{	
		if (REC_2_pressed==1){USBD_AddNoteOn(0, 0x98+DECK_2, 0x0A, 0x7F);} else {USBD_AddNoteOn(0, 0x97, 0x02, 0x7F);}
		need_send_message = 1;		
		HOTCUE_C_2_pressed = 1;	
		}
else if((Rbuffer_2[14]&0x80)==0 && HOTCUE_C_2_pressed==1  )																																												
		{		
		if (REC_2_pressed==1){USBD_AddNoteOn(0, 0x98+DECK_2, 0x0A, 0x00);} else {USBD_AddNoteOn(0, 0x97, 0x02, 0x00);}
		need_send_message = 1;		
		HOTCUE_C_2_pressed = 0;	
		}
/////////
	else if((Rbuffer_2[18]&0x02) && MEMORY_BACK_BUTTON_2_pressed==0)																///////////			<< CALL MEMORY
		{	
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x24, 0x7F);
		need_send_message = 1;		
		MEMORY_BACK_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[18]&0x02)==0 && MEMORY_BACK_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x24, 0x00);
		need_send_message = 1;		
		MEMORY_BACK_BUTTON_2_pressed = 0;	
		flt_adr_CUE_2 = (flt_adr_2*0.62965);								//SET JOG CUE LIGHT ->CDJ has 85 positions 0 = no CUE		
		}
	else if((Rbuffer_2[18]&0x01) && MEMORY_FORWARD_BUTTON_2_pressed==0)														///////////			>> CALL MEMORY
		{	
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x2C, 0x7F);
		need_send_message = 1;		
		MEMORY_FORWARD_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[18]&0x01)==0 && MEMORY_FORWARD_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x2C, 0x00);
		need_send_message = 1;		
		MEMORY_FORWARD_BUTTON_2_pressed = 0;	
		flt_adr_CUE_2 = (flt_adr_2*0.62965);								//SET JOG CUE LIGHT ->CDJ has 85 positions 0 = no CUE		
		}
	else if((Rbuffer_2[18]&0x04) && MEMORY_SET_BUTTON_2_pressed==0)																///////////			SET MEMORY
		{	
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x1C, 0x7F);
		need_send_message = 1;		
		MEMORY_SET_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[18]&0x04)==0 && MEMORY_SET_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x1C, 0x00);
		need_send_message = 1;		
		MEMORY_SET_BUTTON_2_pressed = 0;	
		}
	else if((Rbuffer_2[18]&0x08) && MEMORY_DEL_BUTTON_2_pressed==0)														///////////			DELETE MEMORY
		{	
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x1D, 0x7F);
		need_send_message = 1;		
		MEMORY_DEL_BUTTON_2_pressed = 1;	
		}
	else if((Rbuffer_2[18]&0x08)==0 && MEMORY_DEL_BUTTON_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x1D, 0x00);
		need_send_message = 1;		
		MEMORY_DEL_BUTTON_2_pressed = 0;	
		}
	
	else if((Rbuffer_2[4] != PITCH_MSB_2) && (Rbuffer_2[5] != PITCH_LSB_2))			///////////			PITCH FADER
	{
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x00, (Rbuffer_2[4]/2));
		PITCH_MSB_2=Rbuffer_2[4];
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x20, (Rbuffer_2[5]/2));
		PITCH_LSB_2=Rbuffer_2[5];
		need_send_message = 1;		
	}	
	else if((Rbuffer_2[2] <= TOUCHBRAKE_2-4) || (Rbuffer_2[2] >= TOUCHBRAKE_2+4))	///////////		TOUCH BRAKE
	{
		if (Rbuffer_2[2] != TOUCHBRAKE_2){
		USBD_AddNoteOn(0, 0xB5, 0x08, (Rbuffer_2[2]/2));
		need_send_message = 1;
		TOUCHBRAKE_2=Rbuffer_2[2];
		}
	}

///////////			LOAD WITH EJECT BUTTON
	else if((Rbuffer_2[18]&0x16) && LOAD_2_pressed==0 && MEMORY_BACK_BUTTON_2_pressed!=1 &&  MEMORY_SET_BUTTON_2_pressed!=1)																																																	
		{	
		USBD_AddNoteOn(0, 0x96, 0x46+DECK_2, 0x7F);
		need_send_message = 1;		
		LOAD_2_pressed = 1;	
		}
	else if((Rbuffer_2[18]&0x16)==0 && LOAD_2_pressed==1)
		{		
		USBD_AddNoteOn(0, 0x96, 0x46+DECK_2, 0x00);
		need_send_message = 1;		
		LOAD_2_pressed = 0;	
		//Tbuffer_2[25] = 137;
		}	

//////////////////BROWSE BUTTONS////////////////////		
	else if((Rbuffer_2[17]&0x1) && FOLDER_BACK_pressed_2==0)///////////	<< FOLDER
		{
			if ((Rbuffer_2[18]&0x20)!=0){
				USBD_AddNoteOn(0, 0x96, 0x65, 0x7F);
			}
			else{
				USBD_AddNoteOn(0, 0xB6, 0x40, 127);	//BROSWE Turn counterclockwise: 127~98(0x7F~0x62)
				BROWSE_BACK_pressed_2=1;
			}				
			need_send_message = 1;		
			FOLDER_BACK_pressed_2=1;
		}
	else if((Rbuffer_2[17]&0x1)==0 && FOLDER_BACK_pressed_2==1)
		{		
			if ( BROWSE_BACK_pressed_2!=1){
				USBD_AddNoteOn(0, 0x96, 0x65, 0x00);	
			}

			need_send_message = 1;	
			FOLDER_BACK_pressed_2 = 0;	
			BROWSE_BACK_pressed_2 = 0;
		}	
	
	else if((Rbuffer_2[17]&0x2) && FOLDER_FORWARD_pressed_2==0)///////////	>> FOLDER
		{
			if ((Rbuffer_2[18]&0x20)!=0){
				USBD_AddNoteOn(0, 0x96, 0x41, 0x7F);	
			}
			else{
				USBD_AddNoteOn(0, 0xB6, 0x40, 1);	// BROWSE Turn clockwise: 1~30(0x01~0x1E)
				BROWSE_FORWARD_pressed_2 = 1;	
			}
			need_send_message = 1;		
			FOLDER_FORWARD_pressed_2 = 1;	
		}
	else if((Rbuffer_2[17]&0x2)==0 && FOLDER_FORWARD_pressed_2==1)
		{		
			if (BROWSE_FORWARD_pressed_2 !=1){
				USBD_AddNoteOn(0, 0x96, 0x41, 0x00);	
			}

			need_send_message = 1;	
			FOLDER_FORWARD_pressed_2 = 0;
			BROWSE_FORWARD_pressed_2 = 0;	
		}	
		
		
	/*	
	else if((Rbuffer_2[17]&0x01)!=0 && (Rbuffer_2[18]&0x40)!=0)																			///////////	<< FOLDER + TEXTMODE = NEEDLE SEARCH
		{
				if(NEEDLE>=1){
			NEEDLE--;
		}
		USBD_AddNoteOn(0, 0xB0, 0x03, NEEDLE);	
		need_send_message = 1;		
		}
	else if((Rbuffer_2[17]&0x02)!=0 && (Rbuffer_2[18]&0x40)!=0)																			///////////	>> FOLDER + TEXTMODE = NEEDLE SEARCH
			{
				if(NEEDLE<=126){
			NEEDLE++;
		}
		USBD_AddNoteOn(0, 0xB0, 0x03, NEEDLE);	
		need_send_message = 1;		
		}
	*/
	//////////////

	else if((Rbuffer_2[12]&0x20)!=0 && JOG_2_pressed==0)									// JOG PRESSED
		{
		USBD_AddNoteOn(0, 0x90+DECK_2, 0x36, 0x7F);
		need_send_message = 1;		
		JOG_2_pressed = 1;	
		TOUCH_2=1;
		}
	else if((Rbuffer_2[12]&0x20)==0 && JOG_2_pressed==1)									// JOG RELEASED
		{
				USBD_AddNoteOn(0, 0x90+DECK_2, 0x36, 0x00);
				need_send_message = 1;	
				JOG_2_pressed = 0;	
		}
	
	else if((Rbuffer_2[12]&0x80)!=0){	// ROTATION DETECTED
		MOVING_2=1;	
		PULSE_COUNT_2 = 256*Rbuffer_2[8] + Rbuffer_2[9];
		divider_2++;
		if (divider_2>=2){
			if(Rbuffer_2[12]&0x40)				  //foward
					{
					if(PULSE_COUNT_2>PREV_COUNT_2)
					{
						SCRTCH_2 = (0xFFFF - PULSE_COUNT_2) +	PREV_COUNT_2;
					}
					else
					{
						SCRTCH_2 = PREV_COUNT_2 - PULSE_COUNT_2;	
					}
	
		SCRTCH_2 = SCRTCH_2>>2;	
		SCRTCH_2 = SCRTCH_2*4;
		SCRTCH_2 &= 0x3F;	
		SCRTCH_2 = SCRTCH_2+1;
		if (127-(SCRTCH_2)>=65 && 127-(SCRTCH_2)<=125){
				if (TOUCH_2==1){
					USBD_AddNoteOn(0, 0xB0+DECK_2, 0x22, (127-SCRTCH_2)-1);	
				}
				else{
					USBD_AddNoteOn(0, 0xB0+DECK_2, 0x21, (127-SCRTCH_2)-1);	
				}
			}//>65 <127
		}//END ROTATION FORWARD
	else	//reverse rotation
	{
		if(PULSE_COUNT_2>PREV_COUNT_2)
			{
				SCRTCH_2 = (0xFFFF - PULSE_COUNT_2) +	PREV_COUNT_2;
			}
			else
			{
				SCRTCH_2 = PREV_COUNT_2 - PULSE_COUNT_2;	
			}
		SCRTCH_2 = SCRTCH_2>>2;	
		SCRTCH_2 = SCRTCH_2*4;
		SCRTCH_2 &= 0x3F;	
		if (63-(SCRTCH_2)>=2 && 63-(SCRTCH_2)<=63){	
				if (TOUCH_2==1){
					USBD_AddNoteOn(0, 0xB0+DECK_2, 0x22, (64-SCRTCH_2)-1);		
				}
				else{	
					USBD_AddNoteOn(0, 0xB0+DECK_2, 0x21, (64-SCRTCH_2)-1);		
				}
			}//45 - 58
		}//END REVERSE ROTATION
		need_send_message = 1;			
		divider_2=0;					
	}//END DIVIDER		
PREV_COUNT_2 = PULSE_COUNT_2;
}

	else if(MOVING_2==1 && (Rbuffer_2[12]&0x80)==0)
		{
		USBD_AddNoteOn(0, 0xB0+DECK_2, 0x21, 64);		
		need_send_message = 1;
		MOVING_2=0;
		TOUCH_2=0;
		}
		
	Tbuffer_2[21] = flt_adr_CUE_2;	///JOG CUE LED
	Tbuffer_2[25] = flt_adr_2;			///JOG LED
	CheckTXCRC_2();		
	}
	
	
//////////////////////////////////////
//Function Checksum for TX package	
//
void CheckTXCRC_1()
	{
	uint8_t sdata = 141;
	uint8_t bt = 17;
	while(bt<26)
		{
		sdata+=Tbuffer_1[bt];	
		bt++;	
		}
	Tbuffer_1[26] = sdata;
	return;	
	}
	
//////////////////////////////////////
//Function Checksum for TX package	
//	
void CheckTXCRC_2()
	{
	uint8_t sdata = 141;
	uint8_t bt = 17;
	while(bt<26)
		{
		sdata+=Tbuffer_2[bt];	
		bt++;	
		}
	Tbuffer_2[26] = sdata;
	return;	
	}
	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
