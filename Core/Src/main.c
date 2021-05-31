/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
	WRITTEN BY ARTURO DI GRIROLAMO, APRIL 2021
	MPS RESEARCH PROJECT AT THE UNIVERSITY OF MINNESOTA
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define samplesize 170000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CSR     0x00            //Channel select register            1 Byte
#define CFTW    0x04            //Channel Frequency Tuning Word      4 Bytes
#define CPOW    0x05            //Channel Phase Offset Word          2 Bytes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//VINIT VARIABLES
uint32_t FTW0;	//Frequency tuning word for channel 0
uint32_t FTW1;	//Frequency tuning word for channel 1
uint32_t POW0;	//Phase offset word for channel 0
uint32_t POW1;	//Phase offset word for channel 1
uint8_t ftwBytes0[4];
uint8_t ftwBytes1[4];
uint8_t powBytes0[2];
uint8_t powBytes1[2];
uint8_t regBuff;
uint8_t dataBuff;


// ARTURO VARIABLES
uint8_t myRxData[2];
uint8_t myTxData[1] = {0xAA};
uint8_t FreqFlag = 0;
uint8_t FreqFlag2 = 0;
uint16_t FrequencyH = 0;
uint16_t FrequencyL = 0;
uint8_t a[2] = {0xAA,0};
uint8_t b[2] = {0xBB,0};
uint8_t c[2] = {0xFF,0};
uint8_t d_to_dds[2] = {'1',0};
uint8_t needtoadj = 0;

uint8_t the_stuff[3][samplesize];
uint32_t read_data;
uint16_t sampleRateDivider = 700;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MY_DDS9958_FUNCTION_A()
{
	//LED turned on during DDS operation
	LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_0);

	//Write the frequency and phase information for the DDS AD9958 below
	//Refer to the Python code for correct numbers here
	FTW0 = 858993;	//5KHz
	POW0 = 0;		//0 degree
	FTW1 = 858993;	//5KHz
	POW1 = 1500;	//90 degree - 4096; 4480; 1500

	ftwBytes0[0] = FTW0 >> 24;	//MSB
	ftwBytes0[1] = FTW0 >> 16;
	ftwBytes0[2] = FTW0 >> 8;
	ftwBytes0[3] = FTW0 >> 0;	//LSB

	ftwBytes1[0] = FTW1 >> 24;	//MSB
	ftwBytes1[1] = FTW1 >> 16;
	ftwBytes1[2] = FTW1 >> 8;
	ftwBytes1[3] = FTW1 >> 0;	//LSB

	powBytes0[0] = POW0 >> 8;	//MSB
	powBytes0[1] = POW0 >> 0;	//LSB

	powBytes1[0] = POW1 >> 8;	//MSB
	powBytes1[1] = POW1 >> 0;	//LSB


	//************************************
	//ADD DDS FUNCTIONALITY HERE
	//************************************

	//------------------------------------
	//Master reset pulse : Pulse of 1ms duration
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_2);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2);
	//------------------------------------

	//------------------------------------
	//Enable SPI communication on DDS : SDIO_3 pin should be pulled "LOW"
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1);
	HAL_Delay(1);
	//------------------------------------

	//------------------------------------
	//Configure 3-wire SPI : *Redundant* if no read operations are performed
	//write 0x72/42 into CSR register while configuring channel-0 updates
	//write 0xB2/82 into CSR register while configuring channel-1 updates
	//------------------------------------

	//------------------------------------
	//Select channel-0 in CSR register : IO update toggle is not needed for channel selection
	regBuff = CSR;
	dataBuff = 0x70;	//0x40 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);
	//------------------------------------

	//------------------------------------
	//Frequency update for channel-0 : IO update toggle is needed
	regBuff = CFTW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_Delay(1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[1], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[2], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[3], 1, 100);	//LSB
	HAL_Delay(1);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_3);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_3);
	//------------------------------------

	//------------------------------------
	//Select channel-0 in CSR register : IO update toggle is not needed for channel selection
	//*Channel select is regenerated before sending the phase information
	regBuff = CSR;
	dataBuff = 0x70;	//0x40 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);
	//------------------------------------

	//------------------------------------
	//Phase update for channel-0 : IO update toggle is needed
	regBuff = CPOW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &powBytes0[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &powBytes0[1], 1, 100);	//LSB
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_3);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_3);
	//------------------------------------

	//------------------------------------
	//Select channel-1 in CSR register : IO update toggle is not needed for channel selection
	regBuff = CSR;
	dataBuff = 0xB0;	//0x80 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);
	//------------------------------------

	//------------------------------------
	//Frequency update for channel-1 : IO update toggle is needed
	regBuff = CFTW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[1], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[2], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[3], 1, 100);	//LSB
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_3);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_3);
	//------------------------------------

	//------------------------------------
	//Select channel-1 in CSR register : IO update toggle is not needed for channel selection
	//*Channel select is regenerated before sending the phase information
	regBuff = CSR;
	dataBuff = 0xB0;	//0x80 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);
	//------------------------------------

	//------------------------------------
	//Phase update for channel-1 : IO update toggle is needed
	regBuff = CPOW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_0);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &powBytes1[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &powBytes1[1], 1, 100);	//LSB
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_0);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_3);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_3);
	//------------------------------------

	//------------------------------------
	//Disable SPI communication on DDS : SDIO_3 pin should be pulled "HIGH"
	//TO make proper use of this, initiate SDIO_3 pin to logic "HIGH" state and pull down only when SPI operations are needed
	//HAL_GPIO_WritePin(DDS_SDIO_3_GPIO_Port, DDS_SDIO_3_Pin, GPIO_PIN_SET);
	//------------------------------------
}
void MY_DDS9958_FUNCTION_B()
{
	//LED turned on during DDS operation
	LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_0);

	//Write the frequency and phase information for the DDS AD9958 below
	//Refer to the Python code for correct numbers here
	FTW0 = 858993;	//5KHz
	POW0 = 0;		//0 degree
	FTW1 = 858993;	//5KHz
	POW1 = 1500;	//90 degree - 4096; 4480; 1500

	ftwBytes0[0] = FTW0 >> 24;	//MSB
	ftwBytes0[1] = FTW0 >> 16;
	ftwBytes0[2] = FTW0 >> 8;
	ftwBytes0[3] = FTW0 >> 0;	//LSB

	ftwBytes1[0] = FTW1 >> 24;	//MSB
	ftwBytes1[1] = FTW1 >> 16;
	ftwBytes1[2] = FTW1 >> 8;
	ftwBytes1[3] = FTW1 >> 0;	//LSB

	powBytes0[0] = POW0 >> 8;	//MSB
	powBytes0[1] = POW0 >> 0;	//LSB

	powBytes1[0] = POW1 >> 8;	//MSB
	powBytes1[1] = POW1 >> 0;	//LSB


	//************************************
	//ADD DDS FUNCTIONALITY HERE
	//************************************

	//------------------------------------
	//Master reset pulse : Pulse of 1ms duration
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_10);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_10);
	//------------------------------------

	//------------------------------------
	//Enable SPI communication on DDS : SDIO_3 pin should be pulled "LOW"
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_9);
	HAL_Delay(1);
	//------------------------------------

	//------------------------------------
	//Configure 3-wire SPI : *Redundant* if no read operations are performed
	//write 0x72/42 into CSR register while configuring channel-0 updates
	//write 0xB2/82 into CSR register while configuring channel-1 updates
	//------------------------------------

	//------------------------------------
	//Select channel-0 in CSR register : IO update toggle is not needed for channel selection
	regBuff = CSR;
	dataBuff = 0x70;	//0x40 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);
	//------------------------------------

	//------------------------------------
	//Frequency update for channel-0 : IO update toggle is needed
	regBuff = CFTW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_Delay(1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[1], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[2], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes0[3], 1, 100);	//LSB
	HAL_Delay(1);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_11);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_11);
	//------------------------------------

	//------------------------------------
	//Select channel-0 in CSR register : IO update toggle is not needed for channel selection
	//*Channel select is regenerated before sending the phase information
	regBuff = CSR;
	dataBuff = 0x70;	//0x40 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);
	//------------------------------------

	//------------------------------------
	//Phase update for channel-0 : IO update toggle is needed
	regBuff = CPOW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &powBytes0[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &powBytes0[1], 1, 100);	//LSB
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_11);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_11);
	//------------------------------------

	//------------------------------------
	//Select channel-1 in CSR register : IO update toggle is not needed for channel selection
	regBuff = CSR;
	dataBuff = 0xB0;	//0x80 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);
	//------------------------------------

	//------------------------------------
	//Frequency update for channel-1 : IO update toggle is needed
	regBuff = CFTW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[1], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[2], 1, 100);
	HAL_SPI_Transmit (&hspi2, &ftwBytes1[3], 1, 100);	//LSB
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_11);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_11);
	//------------------------------------

	//------------------------------------
	//Select channel-1 in CSR register : IO update toggle is not needed for channel selection
	//*Channel select is regenerated before sending the phase information
	regBuff = CSR;
	dataBuff = 0xB0;	//0x80 is also valid
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &dataBuff, 1, 100);
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);
	//------------------------------------

	//------------------------------------
	//Phase update for channel-1 : IO update toggle is needed
	regBuff = CPOW;
	LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_1);
	HAL_SPI_Transmit (&hspi2, &regBuff, 1, 100);
	HAL_SPI_Transmit (&hspi2, &powBytes1[0], 1, 100);	//MSB
	HAL_SPI_Transmit (&hspi2, &powBytes1[1], 1, 100);	//LSB
	LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_1);

	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_11);
	HAL_Delay(1);
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_11);
	//------------------------------------

	//------------------------------------
	//Disable SPI communication on DDS : SDIO_3 pin should be pulled "HIGH"
	//TO make proper use of this, initiate SDIO_3 pin to logic "HIGH" state and pull down only when SPI operations are needed
	//HAL_GPIO_WritePin(DDS_SDIO_3_GPIO_Port, DDS_SDIO_3_Pin, GPIO_PIN_SET);
	//------------------------------------
}
void delay_10ns (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}

void adcRead1000(){
	uint32_t i;
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_5);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
	for(i = 0; i<samplesize; i++){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
		while (LL_SPI_IsActiveFlag_RXNE(SPI1))
		{
			(void)(uint16_t)(READ_REG(SPI1->DR)); // flush any FIFO content
		}
		__IO uint16_t *spidr = ((__IO uint16_t *)&SPI1->DR);
		*spidr = 0xFFFF;
		uint16_t first = (uint16_t)(READ_REG(SPI1->DR));
		*spidr = 0xFFFF;
		uint16_t second = (uint16_t)(READ_REG(SPI1->DR));
		uint32_t first2 = first;
		read_data = ((first2<<12)+(second));
		the_stuff[2][i] = read_data;
		the_stuff[1][i] = (read_data >> 8);
		the_stuff[0][i] = (read_data >> 16);
		//delay_10ns(sampleRateDivider);
	}
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
}

void transmit1000(){
	uint32_t i;
	uint32_t j;
	for(i  = 0; i <samplesize; i++){
		for(j = 0; j<3; j++){
			HAL_UART_Transmit(&huart1, (uint8_t*)&the_stuff[j][i], sizeof(the_stuff[j][i]), 0xFFFF);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	UNUSED(huart);
	/*if(FreqFlag == 1){
		FrequencyL = myRxData[0]*myRxData[1];
		FreqFlag = 0;
		FreqFlag2 = 1;
		HAL_UART_Receive_DMA(&huart1, myRxData, 2);
		return;
	}if(FreqFlag2 == 1){
		if(myRxData[1] != 1){
			FrequencyH = myRxData[0]*powf(10, myRxData[1]); // end my life please
		}else{
			FrequencyH = myRxData[0]*myRxData[1];
		}
		FreqFlag2 = 0;
		needtoadj = 1;
		HAL_UART_Receive_DMA(&huart1, myRxData, 2); // just kidding
		return;
	}*/
	if((myRxData[0] == 'a' )&& (myRxData[1] == 'a')){
		HAL_UART_Transmit(&huart1, (uint8_t * )&a[0], sizeof(a[0]), 0xFFFF);
    }else if((myRxData[0] == 'b' )&& (myRxData[1] == 'b')){
    	adcRead1000();
    	HAL_UART_Transmit(&huart1, (uint8_t * )&b[0], sizeof(b[0]), 0xFFFF);
    }else if((myRxData[0] == 'c' )&& (myRxData[1] == 'c')){
    	transmit1000();
	}/*else if((myRxData[0] == 'l' )&& (myRxData[1] == 'l')){
		FreqFlag = 1;
	}else if(myRxData[0] == '1'){
		d_to_dds[1] = myRxData[1];
	}*/
	HAL_UART_Receive_DMA(&huart1, myRxData, 2);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_SPI_Enable(SPI1);
  HAL_UART_Receive_DMA(&huart1, myRxData, 2);
  HAL_Delay(500);

  MY_DDS9958_FUNCTION_A();
  MY_DDS9958_FUNCTION_B();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
	  HAL_Delay(500);
	  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
	  HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
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
  hi2c1.Init.Timing = 0x00909FCE;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration  
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_12BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0|LL_GPIO_PIN_7);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7 
                          |LL_GPIO_PIN_8);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0|LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11 
                          |LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15 
                          |LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3 
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7 
                          |LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11 
                          |LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15 
                          |LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3 
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
