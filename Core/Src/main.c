/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	DIAG_EN = 0, DIAG_DIS = 1,
} DiagEnable_t;

typedef enum
{
	READ_ERROR = -1, /**< Read Error */
	NOT_ENABLED = -2, /**< Diagnosis not enabled */
	NORMAL = 0, /**< Switch works correctly */
	OVERLOAD = 1, /**< Overload of the Switch */
	SHORT_TO_GND = 2, /**< Short to the ground */
	OVERTEMPERATURE = 3, /**< Overtemperature */
	SHORT_TO_VSS = 4, /**< Short to the supply voltage */
	OPEN_LOAD = 5, /**< Open load detected */
	UNDER_LOAD = 6, /**< Under load condition */
	INVERSE_CURRENT = 7, /**< Inverse current */
} DiagStatus_t;

typedef struct ArduinoShield // data structure to remember the output status of each PROFET U1-U4 and the LEDs status on the Arduino Shield
{
	uint8_t status_In_U1; // = 0;         // PROFET U1 IN status
	uint8_t status_In_U2; // = 0;         // PROFET U2 IN status
	uint8_t status_In_U3; // = 0;         // PROFET U3 IN status
	uint8_t status_In_U4; // = 0;         // PROFET U4 IN status

	uint8_t status_LED1; // = 0;         // LED1 status
	uint8_t status_LED2; // = 0;         // LED2 status
	uint8_t status_LED3; // = 0;         // LED3 status
	uint8_t status_LED4; // = 0;         // LED4 status

	float max_adc_reading; // = 1024.0; // ADC = 10 bit -> 0x400 -> 1024 -> 1 digit = 0.0048828125V resolution
	float adc_reference_voltage; // = 5.0;      // ADC reference voltage = 5V

	int A0_adc_raw_value_PushButton; // = 0; // ADC raw value reading Analog Input A0 (push botton)
	int A1_adc_raw_value_Vbb; // = 0; // ADC raw value reading Analog Input A1 (supply voltage VS)
	int A2_adc_raw_value_IS_1; // = 0; // ADC raw value reading Analog Input A2 (IS sense PROFET+2 U1)
	int A2_adc_raw_value_IS_2; // = 0; // ADC raw value reading Analog Input A2 (IS sense PROFET+2 U2)
	int A3_adc_raw_value_IS_3; // = 0; // ADC raw value reading Analog Input A3 (IS sense PROFET+2 U3)
	int A3_adc_raw_value_IS_4; // = 0; // ADC raw value reading Analog Input A3 (IS sense PROFET+2 U4)

// enter the calibration data if available
	float A1_Vbb_Offset; // = 0.0;        // Vbb offset
	float A1_Vbb_Gain; // = 1.045;       // Vbb gain

// enter the calibration data if available
	float Ampere_Offset; // = 0.0; // PROFET+2 U1 offset -> A2_IS_1_Ampere_Offset
	float Ampere_Gain; // = 1.08;    // PROFET+2 U1 gain   -> A2_IS_1_Ampere_Gain

	float Vbb_Resistor_1; // = 47000.0; // 47kOhm -> check in the schematic resistor R9  (user manual v1.00 page 15 -> 2.5.3 and page 23 area D2/D3/E2/E3)
	float Vbb_Resistor_2; // = 10000.0; // 10kOhm -> check in the schematic resistor R11

	float A1_Vbb_ADC_Voltage; // = 0.0; // Vbb raw voltage data measured from ADC A1 (0-5V)
	float A1_Vbb_Real_Voltage; // = 0.0;        // Vbb voltage (e.g. 12V)
	float A1_Vbb_Real_Voltage_Filter; // = 0.0;   // Vbb voltage with Digital Filter

	float Ux_IS_Voltage[4]; // = 0.0; // PROFET+2 U1 measured voltage -> A2_ADC_Voltage_IS_1

	float Ux_IS_Ampere[4]; // = 0.0; // PROFET+2 U1 calcualted ampere -> A2_IS_1_Ampere

	float Ux_IS_Ampere_Average[4]; // = 0.0; // PROFET+2 U1 calcualted ampere average -> A2_IS_1_Ampere_Average

	int kilis; // typical kilis for the BTS7002
} ArduinoShield;

typedef enum
{     // the PushButton can be configured to digital or analog mode
	digital = 0, analog = 1,
} PushButton;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define   BTS7002                 // Arduino Shield >PROFET+2 12V BTS7002-1EPP selected
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MAX_BUFLEN 1024
char buf[MAX_BUFLEN];
int bufHead = 0, bufTail = 0;
uint8_t tmpbuf;
uint8_t received2 = '\0';
HAL_StatusTypeDef rcvStat2;

bool buttonState = 1; // button state -> reading of the push button -> pressed or not pressed
bool lastButtonState = 1;         // last button state
bool buttonFlag = 0;         // button toggle flag
unsigned int countSM = 0;         // counter for the number of button presses

uint32_t ADC_Value[4];
uint8_t UBuf[500];
uint8_t tcnt = 0;

// Struct Value
ArduinoShield Shield =
{ .status_In_U1 = 0,         // PROFET U1 IN status
		.status_In_U2 = 0,         // PROFET U2 IN status
		.status_In_U3 = 0,         // PROFET U3 IN status
		.status_In_U4 = 0,         // PROFET U4 IN status

		.status_LED1 = 0,         // LED1 status
		.status_LED2 = 0,         // LED2 status
		.status_LED3 = 0,         // LED3 status
		.status_LED4 = 0,         // LED4 status

		.max_adc_reading = 1024.0, // ADC = 10 bit -> 0x400 -> 1024 -> 1 digit = 0.0048828125V resolution
		.adc_reference_voltage = 3.17,         // ADC reference voltage = 5V

		.A0_adc_raw_value_PushButton = 0, // ADC raw value reading Analog Input A0 (push botton)
		.A1_adc_raw_value_Vbb = 0, // ADC raw value reading Analog Input A1 (supply voltage VS)
		.A2_adc_raw_value_IS_1 = 0, // ADC raw value reading Analog Input A2 (IS sense PROFET+2 U1)
		.A2_adc_raw_value_IS_2 = 0, // ADC raw value reading Analog Input A2 (IS sense PROFET+2 U2)
		.A3_adc_raw_value_IS_3 = 0, // ADC raw value reading Analog Input A3 (IS sense PROFET+2 U3)
		.A3_adc_raw_value_IS_4 = 0, // ADC raw value reading Analog Input A3 (IS sense PROFET+2 U4)

		// enter the calibration data if available
		.A1_Vbb_Offset = 0.0,         // Vbb offset
		.A1_Vbb_Gain = 1.045,         // Vbb gain

		// enter the calibration data if available
		.Ampere_Offset = 0.0, // PROFET+2 U1 offset -> A2_IS_1_Ampere_Offset
		.Ampere_Gain = 1.08, // PROFET+2 U1 gain   -> A2_IS_1_Ampere_Gain

		.Vbb_Resistor_1 = 47000.0, // 47kOhm -> check in the schematic resistor R9  (user manual v1.00 page 15 -> 2.5.3 and page 23 area D2/D3/E2/E3)
		.Vbb_Resistor_2 = 10000.0, // 10kOhm -> check in the schematic resistor R11

		.A1_Vbb_ADC_Voltage = 0.0, // Vbb raw voltage data measured from ADC A1 (0-5V)
		.A1_Vbb_Real_Voltage = 0.0,         // Vbb voltage (e.g. 12V)
		.A1_Vbb_Real_Voltage_Filter = 0.0, // Vbb voltage with Digital Filter

		.Ux_IS_Voltage =
		{ 0.0, 0.0, 0.0, 0.0 }, // PROFET+2 U1 measured voltage -> A2_ADC_Voltage_IS_1

		.Ux_IS_Ampere =
		{ 0.0, 0.0, 0.0, 0.0 }, // PROFET+2 U1 calcualted ampere -> A2_IS_1_Ampere

		.Ux_IS_Ampere_Average =
		{ 0.0, 0.0, 0.0, 0.0 }, // PROFET+2 U1 calcualted ampere average -> A2_IS_1_Ampere_Average

#ifdef BTS7002
		.kilis = 22700      // typical kilis for the BTS7002
#endif

#ifdef BTS7004
				.kilis = 20000      // typical kilis for the BTS7004
	#endif

#ifdef BTS7006
				.kilis = 17700      // typical kilis for the BTS7006
	#endif

#ifdef BTS7008
				.kilis = 14500      // typical kilis for the BTS7008
	#endif

		};

// Status
DiagEnable_t diagEnb = DIAG_DIS;
DiagStatus_t diagStatus = NORMAL;

//Current ON/OFF
float CurrentON = 0.0, CurrentOFF = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void enableDiag(char num)
{
	if (num == '1')
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_RESET);
	}
	else if (num == '2')
	{
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_SET);
	}
	else if (num == '3')
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_RESET);
	}
	else if (num == '4')
	{
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_SET);
	}
	diagEnb = DIAG_EN;
}

void disableDiag(char num)
{
	if (num == '1')
	{
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_RESET);
	}
	else if (num == '2')
	{
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_RESET);
	}
	else if (num == '3')
	{
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_RESET);
	}
	else if (num == '4')
	{
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_RESET);
	}
	diagEnb = DIAG_DIS;

}

void ReadADC(char num)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	Shield.A1_adc_raw_value_Vbb = HAL_ADC_GetValue(&hadc1);    // Vbb raw value
	HAL_ADC_Stop(&hadc1);

	/* Change GPIO Channel */
	switch (num)
	{
	case '1':
	case '2':
	case '3':
	case '4':
		break;
	default:
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_SET); // Select sense signal from PROFET+2 device U1 and U3 (DEN1_DEN3= high,DEN2_DEN4 = low)
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_RESET); // Select sense signal from PROFET+2 device U1 and U3
		HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_RESET); // open load in state OFF diagnosis disabled
		HAL_Delay(1); // generously delay of 1ms to give the PROFET+2 time to provide the sense signal
		break;
	}

// tsIS(DIAG) <= 3 x (tON_max + tsIS(ON)_max) = 3 x (210µs + 40) = 750µs!
// Vbb, ADC 1,3 READ
	HAL_ADC_Start_DMA(&hadc1, ADC_Value, 4);
	for (int i = 0; i < 4; i++)
	{
		HAL_DMA_PollForTransfer(&hdma_adc1, HAL_DMA_FULL_TRANSFER, 1000);
	}
	HAL_ADC_Stop_DMA(&hadc1);
// ADC 1,3 SET Data
	Shield.A2_adc_raw_value_IS_1 = ADC_Value[1]; // read analog value from analog input A2 -> device U1 -> sense IS1
	Shield.A3_adc_raw_value_IS_3 = ADC_Value[2]; // read analog value from analog input A3 -> device U3 -> sense IS3

	/* Change GPIO Channel */
	switch (num)
	{
	case '1':
	case '2':
	case '3':
	case '4':
		break;
	default:
		HAL_GPIO_WritePin(DEN1_DEN3_GPIO_Port, DEN1_DEN3_Pin, GPIO_PIN_RESET); // Select sense signal from PROFET+2 device U2 and U4 (DEN1_DEN3= low, DEN2_DEN4 = high)
		HAL_GPIO_WritePin(DEN2_DEN4_GPIO_Port, DEN2_DEN4_Pin, GPIO_PIN_SET); // Select sense signal from PROFET+2 device U2 and U4
		HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_RESET); // open load in state OFF diagnosis disabled
		HAL_Delay(1); // generously delay of 1ms to give the PROFET+2 time to provide the sense signal
		break;
	}

	/* Vbb, ADC 2,4 READ */
	HAL_ADC_Start_DMA(&hadc1, ADC_Value, 4);
	for (int i = 0; i < 4; i++)
	{
		HAL_DMA_PollForTransfer(&hdma_adc1, HAL_DMA_FULL_TRANSFER, 1000);
	}
	HAL_ADC_Stop_DMA(&hadc1);
// ADC 2,4 SET Data
	Shield.A2_adc_raw_value_IS_2 = ADC_Value[1]; // read analog value from analog input A2 -> device U2 -> sense IS2
	Shield.A3_adc_raw_value_IS_4 = ADC_Value[2]; // read analog value from analog input A3 -> device U4 -> sense IS4
}

float readIS(char num)
{
	ReadADC(num);

	// Calculate ADC voltage for analog input A1 -> Vbb
	Shield.A1_Vbb_ADC_Voltage = (float) (Shield.A1_adc_raw_value_Vbb
			/ Shield.max_adc_reading * Shield.adc_reference_voltage); // e.g. 1024/1024*5 -> 5V max
	Shield.A1_Vbb_ADC_Voltage = (Shield.A1_Vbb_ADC_Voltage
			- Shield.A1_Vbb_Offset) * Shield.A1_Vbb_Gain; // e.g. (5V - 0V) * 1 = 5V

	Shield.A1_Vbb_Real_Voltage = (float) ((Shield.A1_Vbb_ADC_Voltage
			* (Shield.Vbb_Resistor_1 + Shield.Vbb_Resistor_2))
			/ Shield.Vbb_Resistor_2); // e.g. (5V *(47k + 10k))/10k = 28.5V max measurable VS
	if (num == '1')
	{
		// Calculate ADC voltage for analog input A2 -> ISense 1
		Shield.Ux_IS_Voltage[0] = Shield.A2_adc_raw_value_IS_1
				/ Shield.max_adc_reading * Shield.adc_reference_voltage; // e.g. 1024/1024*5 -> 5V

		Shield.Ux_IS_Ampere[0] = (float) ((Shield.Ux_IS_Voltage[0]
				* Shield.kilis) / 1000); // e.g. (5*22700)/1000 -> 113.5 A max
		Shield.Ux_IS_Ampere[0] = (Shield.Ux_IS_Ampere[0] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;             // (113.5 - 0) * 1 = 113.5A
		return Shield.Ux_IS_Ampere[0];
	}
	else if (num == '2')
	{
		// Calculate ADC voltage for analog input A2 -> ISense 2
		Shield.Ux_IS_Voltage[1] = Shield.A2_adc_raw_value_IS_2
				/ Shield.max_adc_reading * Shield.adc_reference_voltage; // e.g. calculation for 10A // e.g. (91/1024)*5 = 0,4443359375

		Shield.Ux_IS_Ampere[1] = (float) ((Shield.Ux_IS_Voltage[1]
				* Shield.kilis) / 1000); // e.g. (0,4443359375*22700)/1000 = 10,086 A
		Shield.Ux_IS_Ampere[1] = (Shield.Ux_IS_Ampere[1] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;           // e.g. (10,086-0) * 1 = 10,086A
		return Shield.Ux_IS_Ampere[1];
	}
	else if (num == '3')
	{
		// Calculate ADC voltage for analog input A3 -> ISense 3
		Shield.Ux_IS_Voltage[2] = Shield.A3_adc_raw_value_IS_3
				/ Shield.max_adc_reading * Shield.adc_reference_voltage; // e.g. calculation for 2.2A // e.g. (20/1024)*5 = 0,09765625

		Shield.Ux_IS_Ampere[2] = (float) ((Shield.Ux_IS_Voltage[2]
				* Shield.kilis) / 1000); // (0,09765625*22700)/1000 = 2,216796 A
		Shield.Ux_IS_Ampere[2] = (Shield.Ux_IS_Ampere[2] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;      // e.g. (2,2167-0) * 1 = 2,2167A
		return Shield.Ux_IS_Ampere[2];
	}
	else if (num == '4')
	{
		// Calculate ADC voltage for analog input A3 -> ISense 4
		Shield.Ux_IS_Voltage[3] = Shield.A3_adc_raw_value_IS_4
				/ Shield.max_adc_reading * Shield.adc_reference_voltage;

		Shield.Ux_IS_Ampere[3] = (float) ((Shield.Ux_IS_Voltage[3]
				* Shield.kilis) / 1000);
		Shield.Ux_IS_Ampere[3] = (Shield.Ux_IS_Ampere[3] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;
		return Shield.Ux_IS_Ampere[3];
	}
	else // ALL
	{
		// Calculate ADC voltage for analog input A2 -> ISense 1
		Shield.Ux_IS_Voltage[0] = Shield.A2_adc_raw_value_IS_1
				/ Shield.max_adc_reading * Shield.adc_reference_voltage; // e.g. 1024/1024*5 -> 5V

		Shield.Ux_IS_Ampere[0] = (float) ((Shield.Ux_IS_Voltage[0]
				* Shield.kilis) / 1000); // e.g. (5*22700)/1000 -> 113.5 A max
		Shield.Ux_IS_Ampere[0] = (Shield.Ux_IS_Ampere[0] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;             // (113.5 - 0) * 1 = 113.5A

		// Calculate ADC voltage for analog input A2 -> ISense 2
		Shield.Ux_IS_Voltage[1] = Shield.A2_adc_raw_value_IS_2
				/ Shield.max_adc_reading * Shield.adc_reference_voltage; // e.g. calculation for 10A // e.g. (91/1024)*5 = 0,4443359375

		Shield.Ux_IS_Ampere[1] = (float) ((Shield.Ux_IS_Voltage[1]
				* Shield.kilis) / 1000); // e.g. (0,4443359375*22700)/1000 = 10,086 A
		Shield.Ux_IS_Ampere[1] = (Shield.Ux_IS_Ampere[1] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;           // e.g. (10,086-0) * 1 = 10,086A

		// Calculate ADC voltage for analog input A3 -> ISense 3
		Shield.Ux_IS_Voltage[2] = Shield.A3_adc_raw_value_IS_3
				/ Shield.max_adc_reading * Shield.adc_reference_voltage; // e.g. calculation for 2.2A // e.g. (20/1024)*5 = 0,09765625

		Shield.Ux_IS_Ampere[2] = (float) ((Shield.Ux_IS_Voltage[2]
				* Shield.kilis) / 1000); // (0,09765625*22700)/1000 = 2,216796 A
		Shield.Ux_IS_Ampere[2] = (Shield.Ux_IS_Ampere[2] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;           // e.g. (2,2167-0) * 1 = 2,2167A

		// Calculate ADC voltage for analog input A3 -> ISense 4
		Shield.Ux_IS_Voltage[3] = Shield.A3_adc_raw_value_IS_4
				/ Shield.max_adc_reading * Shield.adc_reference_voltage;

		Shield.Ux_IS_Ampere[3] = (float) ((Shield.Ux_IS_Voltage[3]
				* Shield.kilis) / 1000);
		Shield.Ux_IS_Ampere[3] = (Shield.Ux_IS_Ampere[3] - Shield.Ampere_Offset)
				* Shield.Ampere_Gain;

	}
	//HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_RESET);

}

float diagRead(char num)
{
	uint16_t AnalogDigitalConverterResult = 0;
	float amps = 0.0;
	//DiagStatus_t diagStatus = NORMAL;
	//DiagEnable_t diagEnb = DIAG_DIS;

	if (diagEnb == DIAG_EN)
	{
		//readIS(num);
		//amps = Shield.Ux_IS_Ampere[n];
		amps = readIS(num);

		if (amps > (0.0044 * Shield.kilis))
		{
			return OVERLOAD;
		}
		else if (amps < (0.00002 * Shield.kilis))
		{
			return OPEN_LOAD;
		}
		else
		{
			return NORMAL;
		}
	}

	else
	{
		return NOT_ENABLED;
	}

	return diagStatus;

}

int diagnosisOff(float CurrentON, float CurrentOFF)
{
	if ((CurrentON > (0.0018 * Shield.kilis))
			&& (CurrentON < (0.0044 * Shield.kilis)))
	{
		if ((CurrentOFF > (0.0018 * Shield.kilis))
				&& (CurrentOFF < (0.0044 * Shield.kilis)))
		{
			return SHORT_TO_VSS;
		}
		else
		{
			return OPEN_LOAD;
		}
	}
	else
	{
		if ((CurrentON > (0.0044 * Shield.kilis)))
		{
			return SHORT_TO_GND;
		}
		else
		{
			return NORMAL;
		}
	}
}

void PortStatus(char num)
{
	DiagStatus_t diagStatus = NORMAL;

	switch (num)
	{
	case '1':
		enableDiag(num);
		if ((IN1_GPIO_Port->IDR & IN1_Pin ? 0 : 1) == 1)
		{
			diagStatus = diagRead(num);
		}
		else
		{
			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			CurrentON = readIS(num);
			HAL_Delay(1);

			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			CurrentOFF = readIS(num);
			HAL_Delay(1);

			diagStatus = diagnosisOff(CurrentON, CurrentOFF);
		}
		disableDiag(num);
		break;
	case '2':
		enableDiag(num);
		if ((IN1_GPIO_Port->IDR & IN2_Pin ? 0 : 1) == 1)
		{
			diagStatus = diagRead(num);
		}
		else
		{
			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			CurrentON = readIS(num);

			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			CurrentOFF = readIS(num);

			diagStatus = diagnosisOff(CurrentON, CurrentOFF);
		}
		disableDiag(num);
		break;
	case '3':
		enableDiag(num);
		if ((IN1_GPIO_Port->IDR & IN3_Pin ? 0 : 1) == 1)
		{
			diagStatus = diagRead(num);
		}
		else
		{
			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			CurrentON = readIS(num);

			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			CurrentOFF = readIS(num);

			diagStatus = diagnosisOff(CurrentON, CurrentOFF);
		}
		disableDiag(num);
		break;
	case '4':
		enableDiag(num);
		if ((IN1_GPIO_Port->IDR & IN4_Pin ? 0 : 1) == 1)
		{
			diagStatus = diagRead(num);
		}
		else
		{
			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			CurrentON = readIS(num);

			HAL_GPIO_WritePin(OLOFF_GPIO_Port, OLOFF_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			CurrentOFF = readIS(num);

			diagStatus = diagnosisOff(CurrentON, CurrentOFF);
		}
		disableDiag(num);
		break;
	default:
		readIS(num);
	}

}

void WriteShieldLED(uint8_t _LED1, uint8_t _LED2, uint8_t _LED3, uint8_t _LED4,
		ArduinoShield *Shield)
{
	if (_LED1 == 0)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	}
	if (_LED2 == 0)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	}
	if (_LED3 == 0)
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}
	if (_LED4 == 0)
	{
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	}
	Shield->status_LED1 = _LED1;   // remember LED1 status
	Shield->status_LED2 = _LED2;   // remember LED" status
	Shield->status_LED3 = _LED3;   // remember LED3 status
	Shield->status_LED4 = _LED4;   // remember LED4 status
}

void WriteShieldIN(uint8_t _In1, uint8_t _In2, uint8_t _In3, uint8_t _In4,
		ArduinoShield *Shield)
{
	if (_In1 == 0)
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	}
	if (_In2 == 0)
	{
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
	}
	if (_In3 == 0)
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	}
	if (_In4 == 0)
	{
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
	}
	Shield->status_In_U1 = _In1; // remember PROFET U1 output status
	Shield->status_In_U2 = _In2; // remember PROFET U2 output status
	Shield->status_In_U3 = _In3; // remember PROFET U3 output status
	Shield->status_In_U4 = _In4; // remember PROFET U4 output status
}

void PrintStatus()
{
	sprintf(UBuf, "=================================\r\n\r\n"
			"OUT1: %d\tLED1: %d\r\n"
			"OUT2: %d\tLED2: %d\r\n"
			"OUT3: %d\tLED3: %d\r\n"
			"OUT4: %d\tLED4: %d\r\n\r\n"
			"----------\r\n\r\n", Shield.status_In_U1, Shield.status_LED1,
			Shield.status_In_U2, Shield.status_LED2, Shield.status_In_U3,
			Shield.status_LED3, Shield.status_In_U4, Shield.status_LED4);
	HAL_UART_Transmit(&huart2, (uint8_t*) UBuf, strlen(UBuf), 1000);
}

void PrintADC()
{
	sprintf(UBuf, "Vbb - ADC raw value : %d\r\n"
			"ADC-Voltage (0-5V) : %f\r\n"
			"Vbb-Voltage: %f\r\n\r\n"

			"Out 1 - ADC raw value : %d\r\n"
			"ADC-Voltage (0-5V) : %f\r\n"
			"Sense current OUT 1 : %f\r\n\r\n"

			"Out 2 - ADC raw value : %d\r\n"
			"ADC-Voltage (0-5V) : %f\r\n"
			"Sense current OUT 2 : %f\r\n\r\n"

			"Out 3 - ADC raw value : %d\r\n"
			"ADC-Voltage (0-5V) : %f\r\n"
			"Sense current OUT 3 : %f\r\n\r\n"

			"Out 4 - ADC raw value : %d\r\n"
			"ADC-Voltage (0-5V) : %f\r\n"
			"Sense current OUT 4 : %f\r\n\r\n"

	, Shield.A1_adc_raw_value_Vbb, Shield.A1_Vbb_ADC_Voltage,
			Shield.A1_Vbb_Real_Voltage,

			Shield.A2_adc_raw_value_IS_1, Shield.Ux_IS_Voltage[0],
			Shield.Ux_IS_Ampere[0],

			Shield.A2_adc_raw_value_IS_2, Shield.Ux_IS_Voltage[1],
			Shield.Ux_IS_Ampere[1],

			Shield.A3_adc_raw_value_IS_3, Shield.Ux_IS_Voltage[2],
			Shield.Ux_IS_Ampere[2],

			Shield.A3_adc_raw_value_IS_4, Shield.Ux_IS_Voltage[3],
			Shield.Ux_IS_Ampere[3]);
	HAL_UART_Transmit(&huart2, (uint8_t*) UBuf, strlen(UBuf), 1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		tcnt++;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		HAL_UART_Receive_IT(&huart2, &tmpbuf, 1);
		buf[bufTail] = huart->pRxBuffPtr[0];
		bufTail++;
		bufTail %= 200;
	}
}
int __io_putchar(int ch)
{
	uint8_t *tr = (uint8_t*) &ch;
	HAL_UART_Transmit(&huart2, &tr[0], 1, -1);
	return ch;
}
int __io_getchar()
{
	register int ret;

	__retry: if (bufHead != bufTail)
	{
		ret = buf[bufHead];
		if (ret == '\r')
		{
			ret = '\n';
		}
		bufHead++;
		bufHead %= MAX_BUFLEN;
	}
	else
	{
		goto __retry;
	}
	return ret;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	WriteShieldIN(0, 0, 0, 0, &Shield);
	WriteShieldLED(0, 0, 0, 0, &Shield);

	printf("-----------------------------------------------------\r\n");
	printf(" Infineon PROFET+2 12V Arduino Shield HW Rev 5.0     \r\n");
	printf(" Software Version 1.00                SW Button3     \r\n");
	printf(" PROFET+2 BTS7002-1EPP                               \r\n");
	printf(" PROFET+2 BTS7004-1EPP                               \r\n");
	printf(" PROFET+2 BTS7006-1EPP                               \r\n");
	printf(" PROFET+2 BTS7008-1EPP                     2019-10-29\r\n");
	printf("-----------------------------------------------------\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		rcvStat2 = HAL_UART_Receive(&huart2, &received2, 1, 10);
		if (rcvStat2 == HAL_OK)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) "TX UART2 : ", 11, 1000);
			HAL_UART_Transmit(&huart2, &received2, 1, 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n\r\n", 4, 1000);
			switch (received2)
// switch the outputs based on the button counter
			{
			case '0':  // switch all outputs an LEDs on the Arduino Shield OFF
				WriteShieldLED(0, 0, 0, 0, &Shield);
				WriteShieldIN(0, 0, 0, 0, &Shield);
				break;
			case '1':  // switch OUT1 and LED1 ON
				WriteShieldLED(1, 0, 0, 0, &Shield);
				WriteShieldIN(1, 0, 0, 0, &Shield);
				break;
			case '2':  // switch OUT2 and LED2 ON
				WriteShieldLED(0, 1, 0, 0, &Shield);
				WriteShieldIN(0, 1, 0, 0, &Shield);
				break;
			case '3':  // switch OUT3 and LED3 ON
				WriteShieldLED(0, 0, 1, 0, &Shield);
				WriteShieldIN(0, 0, 1, 0, &Shield);
				break;
			case '4':  // switch OUT4 and LED4 ON
				WriteShieldLED(0, 0, 0, 1, &Shield);
				WriteShieldIN(0, 0, 0, 1, &Shield);
				break;
			default: // switch all outputs an LEDs on the Arduino Shield OFF

//received2 = 0;
				break;
			}
		}
		if (tcnt >= 30)
		{
			PortStatus(received2);
			PrintStatus(&Shield);
			PrintADC();
			tcnt = 0;
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 449;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 9999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

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
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	LED4_Pin | LED3_Pin | IN3_Pin | OLOFF_Pin | DEN2_DEN4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	DEN1_DEN3_Pin | IN4_Pin | LED2_Pin | LED1_Pin | IN2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN2_Pin BTN1_Pin */
	GPIO_InitStruct.Pin = BTN2_Pin | BTN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LED4_Pin LED3_Pin IN3_Pin OLOFF_Pin
	 DEN2_DEN4_Pin */
	GPIO_InitStruct.Pin = LED4_Pin | LED3_Pin | IN3_Pin | OLOFF_Pin
			| DEN2_DEN4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : DEN1_DEN3_Pin IN4_Pin LED2_Pin LED1_Pin
	 IN2_Pin */
	GPIO_InitStruct.Pin = DEN1_DEN3_Pin | IN4_Pin | LED2_Pin | LED1_Pin
			| IN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : IN1_Pin */
	GPIO_InitStruct.Pin = IN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
