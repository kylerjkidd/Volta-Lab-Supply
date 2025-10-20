/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "ctype.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define 	LED_DVR_IC1  			 (0x69 << 1) // IS31FL3193 PWM LED Driver I2C Address - U1
#define 	LED_DVR_IC2  			 (0x6A << 1) // IS31FL3193 PWM LED Driver I2C Address - U2

//General Registers
#define     SC16IS750_REG_RHR        (0x00)
#define     SC16IS750_REG_THR        (0x00)
#define     SC16IS750_REG_IER        (0x01)
#define     SC16IS750_REG_FCR        (0x02)
#define     SC16IS750_REG_IIR        (0x02)
#define     SC16IS750_REG_LCR        (0x03)
#define     SC16IS750_REG_MCR        (0x04)
#define     SC16IS750_REG_LSR        (0x05)
#define     SC16IS750_REG_MSR        (0x06)
#define     SC16IS750_REG_SPR        (0x07)
#define     SC16IS750_REG_TCR        (0x06)
#define     SC16IS750_REG_TLR        (0x07)
#define     SC16IS750_REG_TXLVL      (0x08)
#define     SC16IS750_REG_RXLVL      (0x09)
#define     SC16IS750_REG_IODIR      (0x0A)
#define     SC16IS750_REG_IOSTATE    (0x0B)
#define     SC16IS750_REG_IOINTENA   (0x0C)
#define     SC16IS750_REG_IOCONTROL  (0x0E)
#define     SC16IS750_REG_EFCR       (0x0F)

//Special Registers
#define     SC16IS750_REG_DLL        (0x00)
#define     SC16IS750_REG_DLH        (0x01)

//Enhanced Registers
#define     SC16IS750_REG_EFR        (0x02)
#define     SC16IS750_REG_XON1       (0x04)
#define     SC16IS750_REG_XON2       (0x05)
#define     SC16IS750_REG_XOFF1      (0x06)
#define     SC16IS750_REG_XOFF2      (0x07)

//
#define     SC16IS750_INT_CTS        (0x80)
#define     SC16IS750_INT_RTS        (0x40)
#define     SC16IS750_INT_XOFF       (0x20)
#define     SC16IS750_INT_SLEEP      (0x10)
#define     SC16IS750_INT_MODEM      (0x08)
#define     SC16IS750_INT_LINE       (0x04)
#define     SC16IS750_INT_THR        (0x02)
#define     SC16IS750_INT_RHR        (0x01)

#define     SC16IS750_ADDR           (0x90) // SC16IS750 UART Bridge/GPIO I2C Address
//#define     SC16IS750_OSC_FREQ       (10000000)
#define     SC16IS750_OSC_FREQ       (14745600)

// MCP47CMB22 configuration registers
#define DAC_CH0  					 (0x00 << 3) // DAC wiper 0 set register
#define DAC_CH1  					 (0x01 << 3) // DAC wiper 1 set register
#define DAC_REF 					 (0x08 << 3) // DAC Vref set register
#define DAC_POR 					 (0x09 << 3) // DAC Power Down register
#define DAC_SET					     (0x0A << 3) // DAC gain and status register

// MCP47CMB22 MTP memory regions (stores default values after POR)
#define MTP_DAC_CH0  				 (0x10 << 3) // MTP DAC wiper 0 set register
#define MTP_DAC_CH1					 (0x11 << 3) // MTP DAC wiper 1 set register
#define MTP_DAC_REF 				 (0x18 << 3) // MTP DAC Vref set register
#define MTP_DAC_POR					 (0x19 << 3) // MTP DAC Power Down register
#define MTP_DAC_SET					 (0x1A << 3) // MTP DAC gain and status register

#define ADC_ADDR  					 (0x68 << 1) // MCP3426 A/D converter address
#define DAC_ADDR  					 (0x60 << 1) // MCP47CVB21 12-bit D/A I2C Address
#define POT_ADDR  					 (0x2C << 1) // AD5245 10k digital pot I2C Address


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

// serial comms buffers
uint8_t tx_buf[64];
uint8_t rx_buf[64];

// sense offset
uint8_t vsense_offset = 0;
uint8_t isense_offset = 0;

// LED driver outputs states
uint8_t ledstate1,ledstate2;

// output switch state
int output_enabled;

// encoder
uint32_t counter;
uint8_t direction;
uint32_t position;
uint32_t position_prev;

// display values
uint16_t DisplayVoltage; // output voltage
uint16_t DisplayCurrent; // output current
uint16_t DisplayVoLimit; // compliance voltage
uint16_t DisplayIoLimit; // current limit
uint16_t DisplayOutput;
uint16_t DisplaySelection;

volatile int ButtonISR_A = 0;
volatile int ButtonISR_B = 0;
volatile int ButtonISR_C = 0;
volatile int ButtonISR_D = 0;
volatile int ButtonISR_E = 0;
volatile int FineAdjust = 0;
volatile int MenuState = 0;

volatile int SerialISR = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// -----------------------------------  Begin User Interface interrupt service routines  --------------------------------

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	position_prev = position;
	counter = __HAL_TIM_GET_COUNTER(htim);
	position = counter/4;

	if(position_prev > position){
		direction = 1;
	}
	if(position_prev < position){
		direction = 2;
	}

	return;
}

void ButtonPressed_A(){
	ButtonISR_A = 1; // enable switch
}

void ButtonPressed_B(){
	ButtonISR_B = 1; // current mode
}

void ButtonPressed_C(){
	ButtonISR_C = 1; // voltage mode
}

void ButtonPressed_D(){
	ButtonISR_D = 1; // trim adjust
}

void ButtonPressed_E(){
	ButtonISR_E = 1; // encoder switch
}

void Serial_Interrupt(){
	SerialISR = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim16){
		Serial_Interrupt();
	}
}

// -----------------------------------  End User Interface interrupt service routines  --------------------------------



// -----------------------------------  Begin SSD1306 OLED Display functions  --------------------------------

void VoltaDisplaySplash(){

	// display splash screen on power up

	ssd1306_SetCursor(22, 4);
	ssd1306_WriteString("Volta", Font_16x26, White);
	ssd1306_SetCursor(8, 30);
	ssd1306_WriteString("Lab Bench Supply", Font_7x10, White);
	ssd1306_SetCursor(8, 42);
	ssd1306_WriteString("Designed in 2022", Font_7x10, White);
	ssd1306_SetCursor(8, 54);
	ssd1306_WriteString("FW Rev: 0.9.3", Font_7x10, White);
	ssd1306_UpdateScreen();

	return;
}

void VoltaDisplayInit(){

	// set up the 128x64 OLED to display the framework for the user interface

	ssd1306_Fill(Black);

	ssd1306_SetCursor(2, 5);
	ssd1306_WriteString("Voltage:", Font_7x10, White);
	ssd1306_SetCursor(60, 1);
	ssd1306_WriteString("00.00", Font_11x18, White);
	ssd1306_SetCursor(114, 1);
	ssd1306_WriteString("V", Font_11x18, White);

	ssd1306_SetCursor(2, 20);
	ssd1306_WriteString("Current:", Font_7x10, White);
	ssd1306_SetCursor(60, 16);
	ssd1306_WriteString("0.000", Font_11x18, White);
	ssd1306_SetCursor(114, 16);
	ssd1306_WriteString("A", Font_11x18, White);

	ssd1306_SetCursor(10, 40);
	ssd1306_WriteString("VLIM:", Font_7x10, White);
	ssd1306_SetCursor(46, 40);
	ssd1306_WriteString("00.00", Font_7x10, White);
	ssd1306_SetCursor(82, 40);
	ssd1306_WriteString("V", Font_7x10, White);

	ssd1306_SetCursor(10, 52);
	ssd1306_WriteString("ILIM:", Font_7x10, White);
	ssd1306_SetCursor(46, 52);
	ssd1306_WriteString("0.000", Font_7x10, White);
	ssd1306_SetCursor(82, 52);
	ssd1306_WriteString("A", Font_7x10, White);

	ssd1306_SetCursor(95, 40);
	ssd1306_WriteString("COAR", Font_7x10, White);

	ssd1306_SetCursor(99, 52);
	ssd1306_WriteString("ADJ", Font_7x10, White);

	ssd1306_DrawRectangle(0, 0, 126, 36, White); // top box
	ssd1306_DrawRectangle(0, 36, 92, 62, White); // left box
	ssd1306_DrawRectangle(92, 36, 126, 62, White); // right box
	ssd1306_UpdateScreen();

	return;
}

void VoltaDisplayValues(uint16_t Voltage, uint16_t Current, uint16_t VoLimit, uint16_t IoLimit, uint8_t FineAdjustState, uint8_t SetSelect){
	uint8_t V1, V2, V3, V4;
	uint8_t I1, I2, I3, I4;
	uint8_t VL1, VL2, VL3, VL4;
	uint8_t IL1, IL2, IL3, IL4;
	char Vdisp[6];
	char Idisp[6];
	char VLdisp[6];
	char ILdisp[6];

	// extract digits from int
	V1 = Voltage / 1000 % 10;
	V2 = Voltage / 100  % 10;
	V3 = Voltage / 10   % 10;
	V4 = Voltage        % 10;

	I1 = Current / 1000 % 10;
	I2 = Current / 100  % 10;
	I3 = Current / 10   % 10;
	I4 = Current        % 10;

	VL1 = VoLimit / 1000 % 10;
	VL2 = VoLimit / 100  % 10;
	VL3 = VoLimit / 10   % 10;
	VL4 = VoLimit        % 10;

	IL1 = IoLimit / 1000 % 10;
	IL2 = IoLimit / 100  % 10;
	IL3 = IoLimit / 10   % 10;
	IL4 = IoLimit        % 10;

	// display voltage setting/sense
    sprintf(Vdisp, "%d%d.%d%d", V1,V2,V3,V4);
	ssd1306_SetCursor(60, 1);
	ssd1306_WriteString(Vdisp, Font_11x18, White);

	// display current setting/sense
    sprintf(Idisp, "%d.%d%d%d", I1,I2,I3,I4);
	ssd1306_SetCursor(60, 16);
	ssd1306_WriteString(Idisp, Font_11x18, White);

	// display voltage limit
    sprintf(VLdisp, "%d%d.%d%d", VL1,VL2,VL3,VL4);
	ssd1306_SetCursor(46, 40);
	ssd1306_WriteString(VLdisp, Font_7x10, White);

	// display current limit
    sprintf(ILdisp, "%d.%d%d%d", IL1,IL2,IL3,IL4);
	ssd1306_SetCursor(46, 52);
	ssd1306_WriteString(ILdisp, Font_7x10, White);

	// display state of adjustment resolution
	if (FineAdjustState == 0){
		ssd1306_SetCursor(95, 40);
		ssd1306_WriteString("COAR", Font_7x10, White);
	}
	else if (FineAdjustState == 1){
		ssd1306_SetCursor(95, 40);
		ssd1306_WriteString("FINE", Font_7x10, White);
	}
	else if (FineAdjustState == 2){
		ssd1306_SetCursor(95, 40);
		ssd1306_WriteString("TRIM", Font_7x10, White);
	}
	else;


	// display arrow for user choice
	if (SetSelect == 0){
		ssd1306_SetCursor(2, 40);
		ssd1306_WriteString(" ", Font_7x10, White);
		ssd1306_SetCursor(2, 52);
		ssd1306_WriteString(" ", Font_7x10, White);
	}
	else if (SetSelect == 1){
		ssd1306_SetCursor(2, 40);
		ssd1306_WriteString(">", Font_7x10, White);
		ssd1306_SetCursor(2, 52);
		ssd1306_WriteString(" ", Font_7x10, White);
	}
	else if (SetSelect == 2){
		ssd1306_SetCursor(2, 40);
		ssd1306_WriteString(" ", Font_7x10, White);
		ssd1306_SetCursor(2, 52);
		ssd1306_WriteString(">", Font_7x10, White);
	}
	else;

	ssd1306_UpdateScreen();

	return;
}

void AdjustVoltage(){
	uint8_t StepSize;
	ButtonISR_C = 0;
	DisplaySelection = 1;
	direction = 0;

	while(ButtonISR_E != 1){

		if(ButtonISR_B == 1){
			break;
		}

		if(ButtonISR_C == 1){
			break;
		}

		if(ButtonISR_D == 1){
			FineAdjust++;

			if (FineAdjust > 2){
				FineAdjust = 0;
			}
			ButtonISR_D = 0;
		}

		if(FineAdjust == 0){
			StepSize = 100;
		}
		else if(FineAdjust == 1){
			StepSize = 5;
		}
		else if(FineAdjust == 2){
			StepSize = 1;
		}
		else;

		if(direction == 1){

			if(DisplayVoLimit > 0){
				DisplayVoLimit -= StepSize;
				direction = 0;
			}
			else {
				direction = 0;
			}

		}
		if(direction == 2){

			if(DisplayVoLimit < 1200){
				DisplayVoLimit += StepSize;
				direction = 0;
			}
			else {
				direction = 0;
			}

		}

		VoltaDisplayValues(DisplayVoltage, DisplayCurrent, DisplayVoLimit, DisplayIoLimit, FineAdjust, DisplaySelection);

		//uint16_t DisplayVoltage; // output voltage
		//uint16_t DisplayCurrent; // output current
		//uint16_t DisplayVoLimit; // compliance voltage
		//uint16_t DisplayIoLimit; // current limit

	}

	DisplaySelection = 0;

	return;
}

void AdjustCurrent(){
	uint8_t StepSize;
	ButtonISR_B = 0;
	DisplaySelection = 2;
	direction = 0;

	while(ButtonISR_E != 1){

		if(ButtonISR_B == 1){
			break;
		}

		if(ButtonISR_C == 1){
			break;
		}

		if(ButtonISR_D == 1){
			FineAdjust++;

			if (FineAdjust > 2){
				FineAdjust = 0;
			}
			ButtonISR_D = 0;
		}

		if(FineAdjust == 0){
			StepSize = 20;
		}
		else if(FineAdjust == 1){
			StepSize = 5;
		}
		else if(FineAdjust == 2){
			StepSize = 1;
		}
		else;

		if(direction == 1){

			if(DisplayIoLimit > 0){
				DisplayIoLimit -= StepSize;
				direction = 0;
			}
			else {
				direction = 0;
			}

		}
		if(direction == 2){

			if(DisplayIoLimit < 1000){
				DisplayIoLimit += StepSize;
				direction = 0;
			}
			else {
				direction = 0;
			}

		}

		VoltaDisplayValues(DisplayVoltage, DisplayCurrent, DisplayVoLimit, DisplayIoLimit, FineAdjust, DisplaySelection);

		//uint16_t DisplayVoltage; // output voltage
		//uint16_t DisplayCurrent; // output current
		//uint16_t DisplayVoLimit; // compliance voltage
		//uint16_t DisplayIoLimit; // current limit

	}

	DisplaySelection = 0;

	return;
}

// -----------------------------------  End SSD1306 OLED Display functions  --------------------------------



// -----------------------------------  Begin SC16IS750 UART Bridge/GPIO functions  --------------------------------

uint8_t SC16IS750_ReadRegister(uint8_t reg_addr){
	uint8_t i2c_buf[8];
	uint8_t value;

	HAL_I2C_Mem_Read(&hi2c2, SC16IS750_ADDR, reg_addr << 3, I2C_MEMADD_SIZE_8BIT, i2c_buf, 1, 10);

	value = i2c_buf[0];

	return value;
}

void SC16IS750_WriteRegister(uint8_t reg_addr, uint8_t value){
	uint8_t i2c_buf[8];
	i2c_buf[0] = value;

	HAL_I2C_Mem_Write(&hi2c2, SC16IS750_ADDR, reg_addr << 3, I2C_MEMADD_SIZE_8BIT, i2c_buf, 1, 10);

	return;
}

void SC16IS750_GPIOPinMode(uint8_t pin_number, uint8_t i_o){
	uint8_t temp_iodir;

	temp_iodir = SC16IS750_ReadRegister(SC16IS750_REG_IODIR);
	if ( i_o == 1 ){	// if output
		temp_iodir |= (0x01 << pin_number);
	}
	else {
		temp_iodir &= (uint8_t)~(0x01 << pin_number);
	}

	SC16IS750_WriteRegister(SC16IS750_REG_IODIR, temp_iodir);

    return;
}

void SC16IS750_GPIOSetPinState(uint8_t pin_number, uint8_t pin_state){
	uint8_t temp_iostate;

	temp_iostate = SC16IS750_ReadRegister(SC16IS750_REG_IOSTATE);
	if ( pin_state == 1 ) {
		temp_iostate |= (0x01 << pin_number);
    }
	else {
		temp_iostate &= (uint8_t)~(0x01 << pin_number);
    }

	SC16IS750_WriteRegister(SC16IS750_REG_IOSTATE, temp_iostate);

    return;
}

uint8_t SC16IS750_GPIOGetPinState(uint8_t pin_number){
    uint8_t temp_iostate;

    temp_iostate = SC16IS750_ReadRegister(SC16IS750_REG_IOSTATE);
    if ( (temp_iostate & (0x01 << pin_number)) == 0 ) {

    	return 0;
    }

    return 1;
}

void SC16IS750_SetBaudrate(uint32_t baudrate){
    uint16_t divisor;
    uint8_t prescaler;
    uint8_t temp_lcr;
    if ( (SC16IS750_ReadRegister(SC16IS750_REG_MCR)&0x80) == 0) { //if prescaler==1
        prescaler = 1;
    } else {
        prescaler = 4;
    }

    divisor = (SC16IS750_OSC_FREQ/prescaler)/(baudrate*16);

    temp_lcr = SC16IS750_ReadRegister(SC16IS750_REG_LCR);
    temp_lcr |= 0x80;
    SC16IS750_WriteRegister(SC16IS750_REG_LCR,temp_lcr);

    //write to DLL
    SC16IS750_WriteRegister(SC16IS750_REG_DLL,(uint8_t)divisor);

    //write to DLH
    SC16IS750_WriteRegister(SC16IS750_REG_DLH,(uint8_t)(divisor>>8));

    temp_lcr &= 0x7F;
    SC16IS750_WriteRegister(SC16IS750_REG_LCR,temp_lcr);

    return;
}

void SC16IS750_FlushFIFO(uint8_t rx_tx){
   uint8_t temp_fcr;

   temp_fcr = SC16IS750_ReadRegister(SC16IS750_REG_FCR);

   if (rx_tx == 0){
       temp_fcr |= 0x04; // reset Tx FIFO
   } else {
       temp_fcr |= 0x02; // reset Rx FIFO
   }
   SC16IS750_WriteRegister(SC16IS750_REG_FCR,temp_fcr);

   return;
}

void CheckDuplicates(void){
    int duplicate_char;

    for (int i=1; i<64; i++){
        if (rx_buf[0] != rx_buf[i]){
            duplicate_char = 0;
        }
        else duplicate_char = 1;
    }

    if(duplicate_char==1){
    	rx_buf[0] = 0x00;
    }

    return;
}

void FlushTxBuffer(void){

	for(int i=0; i<64; i++){
		tx_buf[i] = 0x20;
	}

    return;
}

void FlushRxBuffer(void){

	for(int i=0; i<64; i++){
		rx_buf[i] = 0x20;
	}

    return;
}

void SerialPrint(uint8_t* str){

    for (uint8_t *p = str; *p != '\0'; p++) {

		if(*p == 0x0D){ // 0x2F = / ; 0x0D = CR (enter)
			//rx_buf[i] = 0x0D;
			break;
		}

    	SC16IS750_WriteRegister(SC16IS750_REG_THR, *p);
    }
    SC16IS750_WriteRegister(SC16IS750_REG_THR, 0x0A); // new line
    SC16IS750_WriteRegister(SC16IS750_REG_THR, 0x0D); // carriage return

    FlushRxBuffer();

    return;
}

void SerialRead(){
	uint8_t rx_char;

	for(int i=0; i<64; i++){
		rx_char = SC16IS750_ReadRegister(SC16IS750_REG_RHR);

		if(rx_char == 0x0D){ // 0x2F = / ; 0x0D = CR (enter)
			rx_buf[i] = 0x0D;
			break;
		}

		rx_buf[i] = rx_char;
	}

	CheckDuplicates();

	return;
}

void SC16IS750_Init(){

	SC16IS750_WriteRegister(SC16IS750_REG_IOCONTROL,0x08); // software reset
	SC16IS750_WriteRegister(SC16IS750_REG_FCR,0x01); // enable fifo
	SC16IS750_WriteRegister(SC16IS750_REG_LCR,0x03); // no parity, 8 bits, 1 stop bit

	//SC16IS750_WriteRegister(SC16IS750_REG_IER,0x01); // RHR Interrupt enable

	SC16IS750_SetBaudrate(115200);

	SC16IS750_GPIOPinMode(0,1); // set power on enable pin to output mode
	SC16IS750_GPIOSetPinState(0,0); // initialize power on enable low

	SC16IS750_GPIOPinMode(1,0); // set current limit sense pin to input mode

	SC16IS750_GPIOPinMode(2,1); // set output enable pin to output mode
	SC16IS750_GPIOSetPinState(2,0); // initialize output enable low

	return;
}

// -----------------------------------  End SC16IS750 UART Bridge/GPIO functions  --------------------------------



// -----------------------------------  Begin MCP47CMB22 D/A converter functions  --------------------------------

void MCP47CMB22_Init(){
	uint8_t i2c_buf[2];

    i2c_buf[0] = 0x00;
    i2c_buf[1] = 0x00;

	HAL_I2C_Mem_Write(&hi2c2, DAC_ADDR, DAC_CH0, I2C_MEMADD_SIZE_8BIT, i2c_buf, 2, 10); // send output decode to DAC

    i2c_buf[0] = 0x00;
    i2c_buf[1] = 0x00;

	HAL_I2C_Mem_Write(&hi2c2, DAC_ADDR, DAC_CH1, I2C_MEMADD_SIZE_8BIT, i2c_buf, 2, 10); // send output decode to DAC

	return;
}

void MCP47CMB22_MTP_Program(uint8_t address, uint8_t byte_1, uint8_t byte_2){
	uint8_t i2c_buf[2];

	i2c_buf[0] = byte_1;
	i2c_buf[1] = byte_2;

	HAL_Delay(10);

	HAL_I2C_Mem_Write(&hi2c2, DAC_ADDR, address, I2C_MEMADD_SIZE_8BIT, i2c_buf, 2, 10); // send command to DAC

	return;
}

void MCP47CMB22_Voltage_Set(uint16_t set_point){
	uint8_t i2c_buf[2];
	uint16_t output_code;
    int ref = 2048;
    int gain1 = 1;
    int gain2 = 6;

    output_code = ((set_point*10*4096) + (ref*gain1*gain2) - 1 ) / (ref*gain1*gain2); // 1 LSB = 3mV

    i2c_buf[0] = (output_code >> 8) & 0xFF;
    i2c_buf[1] = output_code & 0xFF;

	HAL_I2C_Mem_Write(&hi2c2, DAC_ADDR, DAC_CH0, I2C_MEMADD_SIZE_8BIT, i2c_buf, 2, 10); // send output decode to DAC

	return;
}

void MCP47CMB22_Current_Set(uint16_t set_point){
	uint8_t i2c_buf[2];
	uint16_t output_code;

    output_code = (set_point*4); // 4 LSB = 1mA

    i2c_buf[0] = (output_code >> 8) & 0xFF;
    i2c_buf[1] = output_code & 0xFF;

	HAL_I2C_Mem_Write(&hi2c2, DAC_ADDR, DAC_CH1, I2C_MEMADD_SIZE_8BIT, i2c_buf, 2, 10); // send output decode to DAC

	return;
}

// -----------------------------------  End MCP47CMB22 D/A converter functions  --------------------------------



// -----------------------------------  Begin MCP3426 A/D converter functions  --------------------------------

void MCP3426_Init(){
	uint8_t i2c_buf[2];
	uint8_t config_byte = 0x00;

	config_byte = 0x08; // one shot conversion mode, channel 1, 16 bit resolution

    i2c_buf[0] = config_byte;

	HAL_I2C_Master_Transmit(&hi2c2, ADC_ADDR, i2c_buf, 1, 10);

	config_byte = 0x28; // one shot conversion mode, channel 1, 16 bit resolution

    i2c_buf[0] = config_byte;

	HAL_I2C_Master_Transmit(&hi2c2, ADC_ADDR, i2c_buf, 1, 10);

	return;
}

uint16_t MCP3426_Voltage_Sense(){
	uint8_t i2c_buf[3];
	uint16_t adc_value;
	uint16_t sense_code;

	i2c_buf[0] = 0x88; // trigger conversion, one shot conversion mode, channel 1, 16 bit resolution
	i2c_buf[2] = 0xFF; // initialize buffer to ensure loop starts

    HAL_I2C_Master_Transmit(&hi2c2, ADC_ADDR, i2c_buf, 1, 10); // ADC trigger

    while( (i2c_buf[2] &= ~(0x7F)) != 0x00 ){
    	HAL_I2C_Master_Receive(&hi2c2, ADC_ADDR, i2c_buf, 3, 10); // wait for output register ready bit to be cleared
    }

	adc_value = ((i2c_buf[0] << 8) | (i2c_buf[1])); // combine bytes after readout

	adc_value = 0x8000 ^ adc_value; // complement MSB

    sense_code = (((adc_value*4096)/65535)*1000 + 3333 - 1 )/3333; // scale to match UX range

    return sense_code;
}

uint16_t MCP3426_Current_Sense(){
	uint8_t i2c_buf[3];
	uint16_t adc_value;
	uint16_t sense_code;

	i2c_buf[0] = 0xA8; // trigger conversion, one shot conversion mode, channel 2, 16 bit resolution
	i2c_buf[2] = 0xFF; // initialize buffer to ensure loop starts

    HAL_I2C_Master_Transmit(&hi2c2, ADC_ADDR, i2c_buf, 1, 10); // ADC trigger

    while( (i2c_buf[2] &= ~(0x7F)) != 0x00 ){
    	HAL_I2C_Master_Receive(&hi2c2, ADC_ADDR, i2c_buf, 3, 10); // wait for output register ready bit to be cleared
    }

	adc_value = ((i2c_buf[0] << 8) | (i2c_buf[1])); // combine bytes  after readout

	adc_value = 0x8000 ^ adc_value; // complement MSB

    sense_code = (((adc_value*4096)/65535))/2; // scale to match UX range

    return sense_code;
}

// -----------------------------------  End MCP3426 A/D converter functions  --------------------------------



// -----------------------------------  Begin AD5245 Digital Potentiometer functions  --------------------------------

void AD5245_Wiper_Command(uint8_t shutdown, uint8_t reset, uint8_t wiper_value){
	uint8_t i2c_buf[2];
	uint8_t instruction_byte = 0x00;

	if(shutdown == 1){
		instruction_byte |= (0x20);
	}

	if(shutdown == 1){
		instruction_byte |= (0x40);
	}

    i2c_buf[0] = instruction_byte;
    i2c_buf[1] = wiper_value;

	HAL_I2C_Master_Transmit(&hi2c2, POT_ADDR, i2c_buf, 2, 10);

	return;
}

// -----------------------------------  End AD5245 Digital Potentiometer functions  --------------------------------



// -----------------------------------  Begin IS31FL3193 PWM LED driver interface functions  --------------------------------

void LED_DVR_Init(void){
	uint8_t i2c_buf[12]  = {0x20, 0x00 , 0x00, 0x08 , 0x20 , 0x20 , 0x20 , 0x00}; // set initial values
	// shutdown disable ; enable outputs ; normal operation ; disable LED breathing ; PWM control ; Imax 5mA ; duty cycle 12.5% ; update PWM registers

	HAL_I2C_Mem_Write(&hi2c1, LED_DVR_IC1, 0x00, I2C_MEMADD_SIZE_8BIT, i2c_buf, 8, 10); // send initialization buffer
	HAL_I2C_Mem_Write(&hi2c1, LED_DVR_IC2, 0x00, I2C_MEMADD_SIZE_8BIT, i2c_buf, 8, 10);

	i2c_buf[0] = 0x07;

	HAL_I2C_Mem_Write(&hi2c1, LED_DVR_IC1, 0x1D, I2C_MEMADD_SIZE_8BIT, i2c_buf, 1, 10); // enable all outputs
	HAL_I2C_Mem_Write(&hi2c1, LED_DVR_IC2, 0x1D, I2C_MEMADD_SIZE_8BIT, i2c_buf, 1, 10);

	i2c_buf[0] = 0x00;
	HAL_Delay(500);

	HAL_I2C_Mem_Write(&hi2c1, LED_DVR_IC1, 0x1D, I2C_MEMADD_SIZE_8BIT, i2c_buf, 1, 10); // disable all outputs
	HAL_I2C_Mem_Write(&hi2c1, LED_DVR_IC2, 0x1D, I2C_MEMADD_SIZE_8BIT, i2c_buf, 1, 10);

	return;
}

void LED_DVR_CTRL(uint8_t driver_number, uint8_t led_number, uint8_t led_state){
	uint8_t i2c_buf[4];
	uint8_t i2c_address;
	uint8_t temp_ledstate;

	if (driver_number == 0) {
		i2c_address = LED_DVR_IC1;
		temp_ledstate = ledstate1;
	}
	if(driver_number == 1){
		i2c_address = LED_DVR_IC2;
		temp_ledstate = ledstate2;
	}


	if ( led_state == 1 ) {
		temp_ledstate |= (0x01 << led_number);
	}
	else {
		temp_ledstate &= (uint8_t)~(0x01 << led_number);
	}

	if (driver_number == 0) {
		ledstate1 = temp_ledstate;
	}
	if(driver_number == 1){
		ledstate2 = temp_ledstate;
	}

	i2c_buf[0] = temp_ledstate;

	HAL_I2C_Mem_Write(&hi2c1, i2c_address, 0x1D, I2C_MEMADD_SIZE_8BIT, i2c_buf, 1, 10);

	return;
}

// -----------------------------------  End IS31FL3193 PWM LED driver interface functions  --------------------------------



// -----------------------------------  Begin Regulator Loop functions  --------------------------------

void ConstantVoltage(uint16_t voltage_set_point){

	// set preregulator output voltage
	if(voltage_set_point < 300){
		AD5245_Wiper_Command(0,0,255); // set VREG to 2.8V
	}
	else if(voltage_set_point <= 400){
		AD5245_Wiper_Command(0,0,50); // set VREG to 5.0V
	}
	else if(voltage_set_point <= 500){
		AD5245_Wiper_Command(0,0,32); // set VREG to 6.0V
	}
	else if(voltage_set_point <= 600){
		AD5245_Wiper_Command(0,0,21); // set VREG to 7.0V
	}
	else if(voltage_set_point <= 700){
		AD5245_Wiper_Command(0,0,14); // set VREG to 8.0V
	}
	else if(voltage_set_point <= 800){
		AD5245_Wiper_Command(0,0,9); // set VREG to 9.0V
	}
	else if(voltage_set_point <= 900){
		AD5245_Wiper_Command(0,0,6); // set VREG to 10.0V
	}
	else if(voltage_set_point <= 1000){
		AD5245_Wiper_Command(0,0,3); // set VREG to 11.0V
	}
	else if(voltage_set_point <= 1100){
		AD5245_Wiper_Command(0,0,1); // set VREG to 11.8V
	}
	else if(voltage_set_point <= 1200){
		AD5245_Wiper_Command(0,0,0); // set VREG to 12.8V
	}

	//HAL_Delay(10);

	MCP47CMB22_Voltage_Set(DisplayVoLimit);

	return;
}

void CC_CV_Detection(){

	int temp_cc_cv;

	if(DisplayVoltage > DisplayVoLimit){
		temp_cc_cv = 0;
	}
	else temp_cc_cv = 1;

	//DisplayVoltage > DisplayCurrent

	if( temp_cc_cv ){
		 LED_DVR_CTRL(0,1,1); // enable constant current LED
		 LED_DVR_CTRL(0,2,0); // disable constant voltage LED
	}
	else {
		 LED_DVR_CTRL(0,1,0); // disable constant current LED
		 LED_DVR_CTRL(0,2,1); // enable constant voltage LED
	}


}

void Current_Limit(){

	if( SC16IS750_GPIOGetPinState(1) ){
		 LED_DVR_CTRL(1,1,1); // enable limit/clamp LED
	}
	else LED_DVR_CTRL(1,1,0); // disable limit/clamp LED

}

void Toggle_Output_Switch(){

	if(output_enabled){
		output_enabled = 0;
		LED_DVR_CTRL(1,2,0); // disable output LED
		SC16IS750_GPIOSetPinState(2,0); // disable output switch
	}
	else{
		output_enabled = 1;
		LED_DVR_CTRL(1,2,1); // enable output LED
	    SC16IS750_GPIOSetPinState(2,1); // enable output switch
	}

}

// -----------------------------------  End Regulator Loop functions  --------------------------------



// ----------------------------------- Begin Serial Comms functions  --------------------------------

void Serial_Voltage_Handler(uint8_t nread_write, uint8_t VL1, uint8_t VL2, uint8_t VL3, uint8_t VL4){

	uint8_t V1, V2, V3, V4;

    if(nread_write == 1){

        DisplayVoLimit = (VL1 * 1000) + (VL2 * 100) + (VL3 * 10) + (VL4 * 1);

        if(DisplayVoLimit > 1200){
        	DisplayVoLimit = 1200;
        }

        V1 = DisplayVoLimit / 1000 % 10;
        V2 = DisplayVoLimit / 100  % 10;
        V3 = DisplayVoLimit / 10   % 10;
        V4 = DisplayVoLimit        % 10;

        sprintf(tx_buf, "VLIM: %d%d.%d%d V",V1,V2,V3,V4);
		SerialPrint(tx_buf);

    }
    else{

    	// extract digits from int
    	V1 = DisplayVoltage / 1000 % 10;
    	V2 = DisplayVoltage / 100  % 10;
    	V3 = DisplayVoltage / 10   % 10;
    	V4 = DisplayVoltage        % 10;

    	sprintf(tx_buf, "Vo: %d%d.%d%d V",V1,V2,V3,V4);
    	SerialPrint(tx_buf);

    }

}

void Serial_Current_Handler(uint8_t nread_write, uint8_t IL1, uint8_t IL2, uint8_t IL3, uint8_t IL4){

	uint8_t I1, I2, I3, I4;

    if(nread_write == 1){

        DisplayIoLimit = (IL1 * 1000) + (IL2 * 100) + (IL3 * 10) + (IL4 * 1);

        if(DisplayIoLimit > 1000){
        	DisplayIoLimit = 1000;
        }

    	I1 = DisplayIoLimit / 1000 % 10;
    	I2 = DisplayIoLimit / 100  % 10;
    	I3 = DisplayIoLimit / 10   % 10;
    	I4 = DisplayIoLimit        % 10;

        sprintf(tx_buf, "ILIM: %d.%d%d%d A",I1,I2,I3,I4);
		SerialPrint(tx_buf);

    }
    else{

    	// extract digits from int
    	I1 = DisplayCurrent / 1000 % 10;
    	I2 = DisplayCurrent / 100  % 10;
    	I3 = DisplayCurrent / 10   % 10;
    	I4 = DisplayCurrent        % 10;

    	sprintf(tx_buf, "Io: %d.%d%d%d A",I1,I2,I3,I4);
    	SerialPrint(tx_buf);

    }

}

void Serial_Command_Handler(){

	uint8_t temp_lsr;
	uint8_t rx_echo[8];

	uint8_t nread_write;
	uint8_t command_char;
	uint8_t space_char;
	uint8_t argument_char[4];

	temp_lsr = SC16IS750_ReadRegister(SC16IS750_REG_LSR); // check line status register
	temp_lsr = temp_lsr & ~(0xFE); // clear all bits except for rx FIFO data ready bit

	if(temp_lsr == 0x01){
		SerialRead();

	}
	else return;

	if( isupper(rx_buf[0]) ){

		LED_DVR_CTRL(1,0,1); // enable serial LED

		command_char = rx_buf[0];
		space_char = rx_buf[1];

		for(int i=0 ; i<4 ; i++){
			argument_char[i] = rx_buf[i+2] - '0';
		}

		for(int i=0 ; i<6 ; i++){
			rx_echo[i] = rx_buf[i];
		}

		rx_echo[6] = 0x0D;

		FlushTxBuffer();
		SerialPrint(rx_echo);

	}
	else command_char = 'Z';

	switch(command_char){

		case 'D': // disable output switch

			output_enabled = 0;
			LED_DVR_CTRL(1,2,0); // disable output LED
			SC16IS750_GPIOSetPinState(2,0); // disable output switch

			sprintf(tx_buf, "ACK");
			SerialPrint(tx_buf);
			break;

		case 'E': // enable output switch

			output_enabled = 1;
			LED_DVR_CTRL(1,2,1); // enable output LED
			SC16IS750_GPIOSetPinState(2,1); // enable output switch

			sprintf(tx_buf, "ACK");
			SerialPrint(tx_buf);
			break;

		case 'I': // set current limit or read back the output current if char sent without arguments

			if(space_char == 0x20){
				 nread_write = 1;
			}
			else nread_write = 0;

			Serial_Current_Handler(nread_write,argument_char[0],argument_char[1],argument_char[2],argument_char[3]);

	  		MCP47CMB22_Current_Set(DisplayIoLimit);

			break;

		case 'N': // instruct the device to report itself on serial

			sprintf(tx_buf, "VLTA-001 ACK");
			SerialPrint(tx_buf);
			break;

		case 'R': // perform a full power on reset of system and peripherals

			sprintf(tx_buf, "RESET ACK");
			SerialPrint(tx_buf);
			NVIC_SystemReset();
			break;

		case 'V': // set voltage limit or read back the output voltage if char sent without arguments

			if(space_char == 0x20){
				 nread_write = 1;
			}
			else nread_write = 0;

			Serial_Voltage_Handler(nread_write,argument_char[0],argument_char[1],argument_char[2],argument_char[3]);

			MCP47CMB22_Voltage_Set(DisplayVoLimit);

			break;

		case 'Z': // address invalid commands as not acknowledge

			sprintf(tx_buf, "NAK");
			SerialPrint(tx_buf);
			break;

		default :

			break;

	}

}

// -----------------------------------  End Serial Comms functions  --------------------------------



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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  SC16IS750_Init();

  LED_DVR_Init();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  ssd1306_Init();

  VoltaDisplaySplash();
  HAL_Delay(2000);

  VoltaDisplayInit();

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim16);

  FlushRxBuffer();

  AD5245_Wiper_Command(0,1,255); // reset digital pot
  SC16IS750_GPIOSetPinState(0,1); // enable preregulator

  //isense_offset = MCP3426_Current_Sense() + 2 - 2;

  sprintf(tx_buf, "VLTA-001 RDY");
  SerialPrint(tx_buf);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  DisplayVoltage = MCP3426_Voltage_Sense() - vsense_offset;
	  DisplayCurrent = MCP3426_Current_Sense() - isense_offset;
	  Current_Limit();
	  CC_CV_Detection();
	  Serial_Command_Handler();

	  if(SerialISR == 1){
		  HAL_Delay(50);
		  LED_DVR_CTRL(1,0,0); // disable serial LED
		  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
		  //Serial_Command_Handler();

		  SerialISR = 0;
	  }
	  //else LED_DVR_CTRL(1,0,0); // disable serial LED


	  	  if(ButtonISR_A == 1){ // output enable/disable

	  		Toggle_Output_Switch();
	  		//SerialRead();
	  		//SerialPrint(rx_buf);

			ButtonISR_A = 0;
	  	  }

	  	  if(ButtonISR_B == 1){ // current mode

	  		//void LED_DVR_CTRL(uint8_t driver_number, uint8_t led_number, uint8_t led_state){
	  		LED_DVR_CTRL(0,0,1); // enable set LED
	  		AdjustCurrent();
	  		LED_DVR_CTRL(0,0,0); // disable set LED
			ButtonISR_B = 0;

	  	  }

	  	  if(ButtonISR_C == 1){ // voltage mode

		  	LED_DVR_CTRL(0,0,1); // enable set LED
	  		AdjustVoltage();
	  		LED_DVR_CTRL(0,0,0); // disable set LED

			ButtonISR_C = 0;
	  	  }

	  	  if(ButtonISR_D == 1){ // trim resolution

			FineAdjust++;

			if (FineAdjust > 2){
				FineAdjust = 0;
			}

			ButtonISR_D = 0;
	  	  }

	  	  if(ButtonISR_E == 1){ // encoder switch

	  		ConstantVoltage(DisplayVoLimit);
	  		MCP47CMB22_Current_Set(DisplayIoLimit);
	  		//MCP47CMB22_Voltage_Set(DisplayVoLimit);
	  		//SerialPrint(tx_buf);
	  		//SerialPrint("string");
	  		//SerialRead();

			ButtonISR_E = 0;
	  	  }

	  	  VoltaDisplayValues(DisplayVoltage, DisplayCurrent, DisplayVoLimit, DisplayIoLimit, FineAdjust, DisplaySelection);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00301347;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00301347;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 40000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
