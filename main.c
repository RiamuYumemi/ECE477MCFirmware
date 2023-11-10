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
#include "lcd.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
ADC_HandleTypeDef hadc;
uint8_t rxData;
UART_HandleTypeDef huart1;
uint8_t mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	//if mode == 0: it is off:
	if(mode == 0){
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		/* USER CODE END WHILE */
		//PWM 80% Duty Cycle
		TIM2->ARR = 9600000-1;
		TIM2->CCR1 = 7700000;
		mode = 1;
	}else{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		mode = 0;
	}
}



void init_lcd_spi(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    //Code with the GPIOB needs to be modified
    //#define PB_MOSI (5)
    //#define PB_SCK (3)
    //#define PB_DC (8)
    //#define PB_CS (11)
    //#define PB_RST (14)
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 |GPIO_MODER_MODER14|GPIO_MODER_MODER3|GPIO_MODER_MODER5);
    GPIOB->MODER|= GPIO_MODER_MODER8_0|GPIO_MODER_MODER11_0|GPIO_MODER_MODER14_0; //For DC,CS,RST into general purpose output mode 01
    GPIOB->MODER|= GPIO_MODER_MODER3_1|GPIO_MODER_MODER5_1; // Bits for MOSI and SCK for ALt function mode 10
    GPIOB->AFR[0] &=~(GPIO_AFRL_AFRL3|GPIO_AFRL_AFRL5); //Alt function MOSI,SCK turned high
    //^Changes from GPIO_AFRL_AFR to GPIO_AFRL_AFRL
    GPIOB->BSRR = GPIO_BSRR_BS_8|GPIO_BSRR_BS_11|GPIO_BSRR_BS_14; // DC,CS,RST Sets corresponding ODRx bit
    //Need to write to Chip select
    //Modify the above code to coincide with the appropriate pins
    //Look up timers
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI;
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR2 = SPI_CR2_DS_0|SPI_CR2_DS_1|SPI_CR2_DS_2;
    SPI1->CR1 |= SPI_CR1_SPE;

}


char * toArray(int number)
{
    int n = log10(number) + 1;
    int i;
    char *numberArray = calloc(n, sizeof(char));
    for (i = n-1; i >= 0; --i, number /= 10)
    {
        numberArray[i] = (number % 10) + '0';
    }
    return numberArray;
}
void make_word(u16 x, u16 y, char* word, int len){
    for(int i = 0; i < len; i++){
        LCD_DrawChar(x,y,BLACK, BLACK, word[i], 16, 1);
        x += 10;
        if(x == 240){
            x = 10;
            y +=15;
        }
    }

}

void display_temp(int temp){
    LCD_DrawChar(10,160, BLACK, BLACK, 'T', 16,1);
    LCD_DrawChar(20,160, BLACK, BLACK, 'E', 16,1);
    LCD_DrawChar(30,160, BLACK, BLACK, 'M', 16,1);
    LCD_DrawChar(40,160, BLACK, BLACK, 'P', 16,1);
    LCD_DrawChar(50,160, BLACK, BLACK, ':', 16,1);

    char* nums = toArray(temp);
    int len;
    if(temp < 100){
        len = 2;
    }else {
        len = 3;
    }
    make_word(60, 175, nums, len);
}

void display_temp2(int temp){
    LCD_DrawChar(10,240, BLACK, BLACK, 'R', 16,1);
    LCD_DrawChar(20,240, BLACK, BLACK, 'A', 16,1);
    LCD_DrawChar(30,240, BLACK, BLACK, 'W', 16,1);
    LCD_DrawChar(40,240, BLACK, BLACK, ':', 16,1);

    char* nums = toArray(temp);
    int len;
    if(temp < 100){
        len = 2;
    }else {
        len = 3;
    }
    make_word(60, 255, nums, len);
}

void change_temp2(int newTemp){

	LCD_DrawFillRectangle(60,255,100,270,WHITE);

	char* nums = toArray(newTemp);
	int len;
	if(newTemp < 100){
	    len = 2;
	}else {
	    len = 3;
	}
	make_word(60, 255, nums, 4);

}

void change_temp (int newTemp){

	LCD_DrawFillRectangle(60,175,90,190,WHITE);

	char* nums = toArray(newTemp);
	int len;
	if(newTemp < 100){
	    len = 2;
	}else {
	    len = 3;
	}
	make_word(60, 175, nums, len);

}

void display_time(int minutes, int seconds){
    LCD_DrawChar(10,190, BLACK, BLACK, 'T', 16,1);
    LCD_DrawChar(20,190, BLACK, BLACK, 'I', 16,1);
    LCD_DrawChar(30,190, BLACK, BLACK, 'M', 16,1);
    LCD_DrawChar(40,190, BLACK, BLACK, 'E', 16,1);
    LCD_DrawChar(50,190, BLACK, BLACK, ':', 16,1);

    char arrSeconds[2];
    char arrMinutes[2];

    char* secondArray = toArray(seconds);
    char* minuteArray = toArray(minutes);

    if(seconds < 10){
    	arrSeconds[0] = '0';
    	arrSeconds[1] = secondArray[0];
    }
    else{
    	arrSeconds[0] = secondArray[0];
    	arrSeconds[1] = secondArray[1];
    }
    if(minutes < 10){
    	arrMinutes[0] = '0';
    	arrMinutes[1] = minuteArray[0];
    }else{
    	arrMinutes[0] = minuteArray[0];
    	arrMinutes[1] = minuteArray[1];
    }

   make_word(60, 205, arrMinutes, 2);
   LCD_DrawChar(80,205, BLACK, BLACK, ':', 16,1);
   make_word(90, 205, arrSeconds, 2);

}

void change_time(int minutes, int seconds){
	char arrSeconds[2];
	char arrMinutes[2];

	LCD_DrawFillRectangle(60,205,120,220,WHITE);

	char* secondArray = toArray(seconds);
	char* minuteArray = toArray(minutes);

	if(seconds < 10){
	    arrSeconds[0] = '0';
	    arrSeconds[1] = secondArray[0];
	}else{
		arrSeconds[0] = secondArray[0];
	    arrSeconds[1] = secondArray[1];
	}
	if(minutes < 10){
	    arrMinutes[0] = '0';
	    arrMinutes[1] = minuteArray[0];
	}else{
	    arrMinutes[0] = minuteArray[0];
	    arrMinutes[1] = minuteArray[1];
	}

	make_word(60, 205, arrMinutes, 2);
	LCD_DrawChar(80,205, BLACK, BLACK, ':', 16,1);
	make_word(90, 205, arrSeconds, 2);

}

void display_mode(int mode){
    LCD_DrawChar(10,220, BLACK, BLACK, 'M', 16,1);
    LCD_DrawChar(20,220, BLACK, BLACK, 'O', 16,1);
    LCD_DrawChar(30,220, BLACK, BLACK, 'D', 16,1);
    LCD_DrawChar(40,220, BLACK, BLACK, 'E', 16,1);
    LCD_DrawChar(50,220, BLACK, BLACK, ':', 16,1);

    char no[2];
    no[0] = 'N';
    no[1] = 'o';
    char yes[3];
    yes[0] = 'Y';
    yes[1] = 'e';
    yes[2] = 's';

    if(mode == 0){
        make_word(60, 225, no, 2);
    }else{
        make_word(60,225, yes, 3);
    }
}

//Sets pins 0 and 4 to input
//Sets pins 8-11 as output
void inita(){
	RCC -> AHBENR |= 1 << 18;
	GPIOA -> MODER |= 0x00550000;
	GPIOA -> MODER &= 0xFF55FCFC;
}

//pin_num: Pin number in GPIOA
//val: if 0, then pin is set low, otherwise, pin set high
void setn(int32_t pin_num, int32_t val){
	//GPIOA -> ODR &= val; //Can't  remember what this was for
	if(val == 0) {
		GPIOA -> BRR |= 1 << pin_num;
	}
	else{
		GPIOA -> BRR |= 1 << pin_num;
	}
	//pin_num &= 0x01000000;
	//pin_numb |=val;
}

int32_t readpin(int32_t pin_num){
	return (GPIOA->IDR) & (1<<pin_num);
}

void buttons(void){
	inita();
	int32_t pa3 = readpin(3);
	setn(8,pa3);
}


int convertData(uint16_t raw){
	//Slope = 2.6564
	float slope =0.27724;
	float intercept = 423.209;

	int final = ((slope * raw) - intercept);
    //int final = raw / 2;
	return final;

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
	uint16_t raw;
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
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	  HAL_UART_Receive_IT(&huart1, &rxData, 1);

	  LCD_Setup();
	      LCD_direction(0);
	      LCD_Clear(RED);
	      LCD_DrawRectangle(0,0,240,160,WHITE);

	      LCD_DrawChar(10,10, BLACK, BLACK, 'D', 16,1);
	          LCD_DrawChar(20,10, BLACK, BLACK, 'R', 16,1);
	          LCD_DrawChar(30,10, BLACK, BLACK, 'I', 16,1);
	          LCD_DrawChar(40,10, BLACK, BLACK, 'N', 16,1);
	          LCD_DrawChar(50,10, BLACK, BLACK, 'K', 16,1);
	          LCD_DrawChar(60,10, BLACK, BLACK, 'W', 16,1);
	          LCD_DrawChar(70,10, BLACK, BLACK, 'A', 16,1);
	          LCD_DrawChar(80,10, BLACK, BLACK, 'R', 16,1);
	          LCD_DrawChar(90,10, BLACK, BLACK, 'M', 16,1);
	          LCD_DrawChar(100,10, BLACK, BLACK, 'E', 16,1);
	          LCD_DrawChar(110,10, BLACK, BLACK, 'R', 16,1);
	          LCD_DrawChar(120,10, BLACK, BLACK, '+', 16,1);

	          LCD_DrawChar(10,40, BLACK, BLACK, 'H', 16,1);
	          LCD_DrawChar(20,40, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(30,40, BLACK, BLACK, 'r', 16,1);
	          LCD_DrawChar(40,40, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(50,40, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(60,40, BLACK, BLACK, 'a', 16,1);
	          LCD_DrawChar(70,40, BLACK, BLACK, 't', 16,1);
	          LCD_DrawChar(80,40, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(90,40, BLACK, BLACK, 'M', 16,1);
	          LCD_DrawChar(100,40, BLACK, BLACK, 'u', 16,1);
	          LCD_DrawChar(110,40, BLACK, BLACK, 'g', 16,1);
	          LCD_DrawChar(120,40, BLACK, BLACK, 'g', 16,1);
	          LCD_DrawChar(130,40, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(140,40, BLACK, BLACK, 'r', 16,1);
	          LCD_DrawChar(150,40, BLACK, BLACK, 's', 16,1);
	          LCD_DrawChar(160,40, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(170,40, BLACK, BLACK, 'w', 16,1);
	          LCD_DrawChar(180,40, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(190,40, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(200,40, BLACK, BLACK, 'a', 16,1);
	          LCD_DrawChar(210,40, BLACK, BLACK, 'r', 16,1);
	          LCD_DrawChar(220,40, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(230,40, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(10,55, BLACK, BLACK, 'c', 16,1);
	          LCD_DrawChar(20,55, BLACK, BLACK, 'o', 16,1);
	          LCD_DrawChar(30,55, BLACK, BLACK, 'm', 16,1);
	          LCD_DrawChar(40,55, BLACK, BLACK, 'm', 16,1);
	          LCD_DrawChar(50,55, BLACK, BLACK, 'i', 16,1);
	          LCD_DrawChar(60,55, BLACK, BLACK, 't', 16,1);
	          LCD_DrawChar(70,55, BLACK, BLACK, 't', 16,1);
	          LCD_DrawChar(80,55, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(90,55, BLACK, BLACK, 'd', 16,1);
	          LCD_DrawChar(100,55, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(110,55, BLACK, BLACK, 't', 16,1);
	          LCD_DrawChar(120,55, BLACK, BLACK, 'o', 16,1);
	          LCD_DrawChar(130,55, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(140,55, BLACK, BLACK, 'h', 16,1);
	          LCD_DrawChar(150,55, BLACK, BLACK, 'o', 16,1);
	          LCD_DrawChar(160,55, BLACK, BLACK, 't', 16,1);
	          LCD_DrawChar(170,55, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(180,55, BLACK, BLACK, 'd', 16,1);
	          LCD_DrawChar(190,55, BLACK, BLACK, 'r', 16,1);
	          LCD_DrawChar(200,55, BLACK, BLACK, 'i', 16,1);
	          LCD_DrawChar(210,55, BLACK, BLACK, 'n', 16,1);
	          LCD_DrawChar(220,55, BLACK, BLACK, 'k', 16,1);
	          LCD_DrawChar(230,55, BLACK, BLACK, 's', 16,1);

	          LCD_DrawChar(10,70, BLACK, BLACK, 'W', 16,1);
	          LCD_DrawChar(20,70, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(30,70, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(40,70, BLACK, BLACK, 'w', 16,1);
	          LCD_DrawChar(50,70, BLACK, BLACK, 'i', 16,1);
	          LCD_DrawChar(60,70, BLACK, BLACK, 'l', 16,1);
	          LCD_DrawChar(70,70, BLACK, BLACK, 'l', 16,1);
	          LCD_DrawChar(80,70, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(90,70, BLACK, BLACK, 'n', 16,1);
	          LCD_DrawChar(100,70, BLACK, BLACK, 'o', 16,1);
	          LCD_DrawChar(110,70, BLACK, BLACK, 't', 16,1);
	          LCD_DrawChar(120,70, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(130,70, BLACK, BLACK, 's', 16,1);
	          LCD_DrawChar(140,70, BLACK, BLACK, 'l', 16,1);
	          LCD_DrawChar(150,70, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(160,70, BLACK, BLACK, 'e', 16,1);
	          LCD_DrawChar(170,70, BLACK, BLACK, 'p', 16,1);
	          LCD_DrawChar(10,85, BLACK, BLACK, 'u', 16,1);
	          LCD_DrawChar(20,85, BLACK, BLACK, 'n', 16,1);
	          LCD_DrawChar(30,85, BLACK, BLACK, 't', 16,1);
	          LCD_DrawChar(40,85, BLACK, BLACK, 'i', 16,1);
	          LCD_DrawChar(50,85, BLACK, BLACK, 'l', 16,1);
	          LCD_DrawChar(60,85, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(70,85, BLACK, BLACK, 'y', 16,1);
	          LCD_DrawChar(80,85, BLACK, BLACK, 'o', 16,1);
	          LCD_DrawChar(90,85, BLACK, BLACK, 'u', 16,1);
	          LCD_DrawChar(100,85, BLACK, BLACK, 'r', 16,1);
	          LCD_DrawChar(110,85, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(120,85, BLACK, BLACK, 'd', 16,1);
	          LCD_DrawChar(130,85, BLACK, BLACK, 'r', 16,1);
	          LCD_DrawChar(140,85, BLACK, BLACK, 'i', 16,1);
	          LCD_DrawChar(150,85, BLACK, BLACK, 'n', 16,1);
	          LCD_DrawChar(160,85, BLACK, BLACK, 'k', 16,1);
	          LCD_DrawChar(170,85, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(180,85, BLACK, BLACK, 'i', 16,1);
	          LCD_DrawChar(190,85, BLACK, BLACK, 's', 16,1);
	          LCD_DrawChar(200,85, BLACK, BLACK, ' ', 16,1);
	          LCD_DrawChar(10,100, BLACK, BLACK, 'w', 16,1);
	          LCD_DrawChar(20,100, BLACK, BLACK, 'a', 16,1);
	          LCD_DrawChar(30,100, BLACK, BLACK, 'r', 16,1);
	          LCD_DrawChar(40,100, BLACK, BLACK, 'm', 16,1);
	          LCD_DrawChar(50,100, BLACK, BLACK, '-', 16,1);
	          LCD_DrawChar(60,100, BLACK, BLACK, 'i', 16,1);
	          LCD_DrawChar(70,100, BLACK, BLACK, 's', 16,1);
	          LCD_DrawChar(80,100, BLACK, BLACK, 'h', 16,1);

	  int temp = 50;

	  display_temp(temp);
	  display_temp2(temp);

	  int minutes = 12;
	  int seconds = 50;

	  display_time(minutes, seconds);
	  display_mode(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
    /* USER CODE END WHILE */
		  /*
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){
		  			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		  			  		            /* USER CODE END WHILE
		  			  		                  //PWM 80% Duty Cycle
		  			  		              TIM2->ARR = 9600000-1;
		  			  		              TIM2->CCR1 = 7700000;
		  }
		  else{
		  	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		  }*/


		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

		  // Get ADC value
		  HAL_ADC_Start(&hadc);
		  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		  raw = HAL_ADC_GetValue(&hadc);
		  temp = convertData(raw);


		  HAL_Delay(2000);
		  change_temp(temp);
		  change_temp2(raw);
		  seconds--;
		  if(seconds == 0){
		    seconds = 60;
		  	  minutes--;
		  }
		  change_time(minutes,seconds);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
