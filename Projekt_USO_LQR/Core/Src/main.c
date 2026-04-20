/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h> // Potrzebne do memcpy

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

/* USER CODE BEGIN PV */
#define INA237_REG_CONFIG         0x00
#define INA237_REG_ADC_CONFIG     0x01
#define INA237_REG_SHUNT_CAL      0x02
#define INA237_REG_VSHUNT         0x04
#define INA237_REG_VBUS           0x05
#define INA237_REG_DIETEMP        0x06
#define INA237_REG_CURRENT        0x07
#define INA237_REG_POWER          0x08
#define INA237_REG_MANUFACTURER   0x3E

#define SHUNT_RESISTOR_OHM        0.015f
#define INA237_CURRENT_LSB        0.000080f
#define INA237_VOLTAGE_LSB        0.003125f

#define INA237_CAL_VALUE          0x03D7
#define INA237_ADC_CONFIG_VAL   0xB901
#define INA237_CONFIG_HIGH_PRECISION 0x0010

#define INA237_ADDR (0x40 << 1)
#define BUFFER_SIZE 4000
#define SAMPLING_FREQ 1000.0f

volatile float dbg_current_A = 0.0f;
volatile float dbg_position = 0.0;
volatile float dbg_velocity = 0.0;
volatile float dbg_torque = 0.0f;
volatile float dbg_u = 0.0f;
volatile float dbg_velocity_encoder = 0.0f;

float prev_pos = 0.0f;

volatile uint16_t prev_encoder_cnt = 0;



#define INA237_VSHUNT_LSB_V     0.0000025f

float R[2] = {
	3e-6f,
	 0.05f
};

float Q_diagonal[3] = {
    1e-6f,   // pozycja
    2e-2f,   // prędkość
    2e-2f    // prąd
};

float CPR = 3840.0f;
volatile float current_A = 0.0f;
#define INA237_CURRENT_LSB  0.000080f
#define INA237_ADDR         (0x40 << 1)
#define INA237_REG_CURRENT  0x07
volatile uint8_t timer_flag = 0; // Flaga do synchronizacji pętli main
volatile float current_live_A = 0.0f;
volatile float position_rad = 0.0f;
volatile float dbg_position_Kalman = 0.0f;
volatile float dbg_current_A_Kalman = 0.0f;
float pos = 0.0f;

float u = 0.0f;

float A[3][3] = {
    {1.00000000f, 0.00099769f, 0.00001130f},
    {0.00000000f, 0.99515247f, 0.02090289f},
    {0.00000000f, -0.05170580f, 0.61609530f},
};

// Macierz B_discrete [3x1]
float B[3][1] = {
    {0.00000065f},
    {0.00188346f},
    {0.13212621f},
};

#define CONTROL_PROFILE_COMFORT  0
#define CONTROL_PROFILE_SPORT    1

#define CONTROL_PROFILE CONTROL_PROFILE_SPORT

#if CONTROL_PROFILE == CONTROL_PROFILE_COMFORT

float F[3] = {4.87455066f, 0.48550506f, 4.66944218f};
float N_bar = 4.87455066f;

#elif CONTROL_PROFILE == CONTROL_PROFILE_SPORT

float F[3] = {194.89216201f, 0.18402404f, 1.21252480f};
float N_bar = 194.89216201f;

#else
#error "Nieznany CONTROL_PROFILE"
#endif





volatile float dbg_current = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

float Read_Current_INA237(void) {
    uint8_t i2c_buff[2];
    int16_t raw_vshunt;
    float voltage_v;

    if (HAL_I2C_Mem_Read(&hi2c2, INA237_ADDR, INA237_REG_VSHUNT, I2C_MEMADD_SIZE_8BIT, i2c_buff, 2, 10) == HAL_OK) {
        raw_vshunt = (int16_t)((i2c_buff[0] << 8) | i2c_buff[1]);
        voltage_v = (float)raw_vshunt * INA237_VSHUNT_LSB_V;
        current_A = voltage_v / SHUNT_RESISTOR_OHM;
    }
    else {
        printf("I2C ERROR\r\n");
    }
    return current_A;
}

float Read_Position(void)
{
    static int32_t total_ticks = 0;
    static uint16_t prev_counter = 0;

    if (prev_counter == 0) prev_counter = __HAL_TIM_GET_COUNTER(&htim3);

    uint16_t current_counter = __HAL_TIM_GET_COUNTER(&htim3);

    int16_t delta = (int16_t)(current_counter - prev_counter);

    total_ticks += delta;
    prev_counter = current_counter;
    return ((float)total_ticks / CPR) * 6.2831853f;
}

float *KalmanFilter_Update(float A[3][3],
                           float B[3],
                           const float R[2],
                           float Q[3],       // Q jest wektorem diagonalnym
                           float i_meas,
                           float theta_meas)
{
    // Zmienne stanu i kowariancji (zachowane między wywołaniami)
    static float x[3] = {0.0f, 0.0f, 0.0f};
    static float P[3][3] = {
        {0.01f, 0.0f,  0.0f},
        {0.0f,  1.0f,  0.0f},
        {0.0f,  0.0f,  0.1f}
    };


    // Zmienne pomocnicze
    float x_pred[3];
    float P_pred[3][3];
    extern float u; // Napięcie sterujące

    float effective_u = u;

    if (fabsf(u) < 0.15f) {
        effective_u = 0.0f;
    }

    /* =====================================================
     * 1. PREDYKCJA STANU
     * x_pred = A * x + B * u
     * ===================================================== */
    for (int i = 0; i < 3; i++) {
        x_pred[i] = B[i] * effective_u;
        for (int j = 0; j < 3; j++) {
            x_pred[i] += A[i][j] * x[j];
        }
    }

    if (x_pred[1]> 3.80){
    	x_pred[1] = 3.80;
    }

    else if (x_pred[1] < -4.09)
    	x_pred[1] = -4.09;

    /* =====================================================
     * 2. PREDYKCJA KOWARIANCJI
     * P_pred = A * P * A^T + Q
     * ===================================================== */
    // Krok 2a: tmp = A * P
    float tmp[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                tmp[i][j] += A[i][k] * P[k][j];
            }
        }
    }

    // Krok 2b: P_pred = tmp * A^T + Q
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                // A^T[k][j] to to samo co A[j][k]
                P_pred[i][j] += tmp[i][k] * A[j][k];
            }
        }
        // Dodanie szumu procesu Q (tylko na przekątnej)
        P_pred[i][i] += Q[i];
    }

    /* =====================================================
     * 3. INNOWACJA (RESIDUALS)
     * y = z - H * x_pred
     * H = [[1, 0, 0], [0, 0, 1]] (Pomiar: Pozycja, Prąd)
     * ===================================================== */
    float y[2];
    y[0] = theta_meas - x_pred[0]; // Pozycja (indeks 0)
    y[1] = i_meas     - x_pred[2]; // Prąd (indeks 2)

    /* =====================================================
     * 4. MACIERZ INNOWACJI S (2x2)
     * S = H * P_pred * H^T + R
     * ===================================================== */
    float S[2][2];
    // Wyciągamy odpowiednie elementy z P_pred zgodnie z macierzą H
    S[0][0] = P_pred[0][0] + R[0];
    S[0][1] = P_pred[0][2];
    S[1][0] = P_pred[2][0];
    S[1][1] = P_pred[2][2] + R[1];

    /* =====================================================
     * 5. INWERSJA MACIERZY S (S^-1)
     * ===================================================== */
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    float invS[2][2];

    // Zabezpieczenie przed dzieleniem przez zero
    if (fabsf(det) < 1e-9f) det = 1e-9f;
    float invDet = 1.0f / det;

    invS[0][0] =  S[1][1] * invDet;
    invS[0][1] = -S[0][1] * invDet;
    invS[1][0] = -S[1][0] * invDet;
    invS[1][1] =  S[0][0] * invDet;

    /* =====================================================
     * 6. WZMOCNIENIE KALMANA K (3x2)
     * K = P_pred * H^T * S^-1
     * ===================================================== */
    // P_pred * H^T to po prostu kolumny 0 i 2 z macierzy P_pred
    float PHt[3][2];
    for(int i=0; i<3; i++) {
        PHt[i][0] = P_pred[i][0];
        PHt[i][1] = P_pred[i][2];
    }

    float K[3][2];
    for(int i=0; i<3; i++) {
        K[i][0] = PHt[i][0]*invS[0][0] + PHt[i][1]*invS[1][0];
        K[i][1] = PHt[i][0]*invS[0][1] + PHt[i][1]*invS[1][1];
    }

    /* =====================================================
     * 7. KOREKCJA STANU
     * x = x_pred + K * y
     * ===================================================== */
    for (int i = 0; i < 3; i++) {
        x[i] = x_pred[i] + K[i][0] * y[0] + K[i][1] * y[1];
    }

    /* =====================================================
     * 8. KOREKCJA KOWARIANCJI
     * P = P_pred - K * S * K^T
     * (Jest to stabilniejsza numerycznie forma niż (I-KH)P)
     * ===================================================== */
    // Najpierw: KS = K * S
    float KS[3][2];
    for(int i=0; i<3; i++) {
        KS[i][0] = K[i][0]*S[0][0] + K[i][1]*S[1][0];
        KS[i][1] = K[i][0]*S[0][1] + K[i][1]*S[1][1];
    }

    // Teraz: P = P_pred - KS * K^T
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            // K^T[k][j] to K[j][k]
            float val = KS[i][0]*K[j][0] + KS[i][1]*K[j][1];
            P[i][j] = P_pred[i][j] - val;
        }
    }


    return x;
}

float Read_SetPoint(void)
{
    return 3.14f / 2.0f;
}

float IIR_Filter(float x)
{
    const float b0 = 0.02008337f;
    const float b1 = 0.04016673f;
    const float b2 = 0.02008337f;
    const float a1 = -1.56101808f;
    const float a2 =  0.64135154f;

    static float x1 = 0.0f, x2 = 0.0f;
    static float y1 = 0.0f, y2 = 0.0f;

    float y = b0*x + b1*x1 + b2*x2
            - a1*y1 - a2*y2;

    x2 = x1;  x1 = x;
    y2 = y1;  y1 = y;

    return y;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        timer_flag = 1;
    }
}

void print_float(char* label, float val, char* unit) {
    int i_part = (int)val;
    int f_part = (int)((fabs(val) - abs(i_part)) * 10000);
    printf("%s %d.%04d %s\r\n", label, i_part, f_part, unit);
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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
    printf("\r\n=== SYSTEM START ===\r\n");

    uint8_t i2c_data[2];

    uint16_t cfg_reg = INA237_CONFIG_HIGH_PRECISION;
    i2c_data[0] = (cfg_reg >> 8) & 0xFF;
    i2c_data[1] = cfg_reg & 0xFF;
    HAL_I2C_Mem_Write(&hi2c2, INA237_ADDR, INA237_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, i2c_data, 2, 100);

    uint16_t cal = INA237_CAL_VALUE;
    i2c_data[0] = (cal >> 8) & 0xFF;
    i2c_data[1] = cal & 0xFF;
    HAL_I2C_Mem_Write(&hi2c2, INA237_ADDR, INA237_REG_SHUNT_CAL, I2C_MEMADD_SIZE_8BIT, i2c_data, 2, 100);

    uint16_t adc_cfg = INA237_ADC_CONFIG_VAL;
    i2c_data[0] = (adc_cfg >> 8) & 0xFF;
    i2c_data[1] = adc_cfg & 0xFF;
    HAL_I2C_Mem_Write(&hi2c2, INA237_ADDR, INA237_REG_ADC_CONFIG, I2C_MEMADD_SIZE_8BIT, i2c_data, 2, 100);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (timer_flag) {
	          timer_flag = 0;

	          float i_meas = Read_Current_INA237();
	          float theta_meas = Read_Position();

	          float *x = KalmanFilter_Update(A, B, R, Q_diagonal, i_meas, theta_meas);

	          dbg_position_Kalman = x[0];
	          dbg_velocity = x[1];
	          dbg_current_A_Kalman = x[2];

	          dbg_position = theta_meas ;
	          dbg_current = i_meas ;


	          pos = (dbg_position - prev_pos)/0.001;
	          dbg_velocity_encoder = IIR_Filter(pos);
	          prev_pos = dbg_position ;

	          float xd = 6.28f;

	          if (fabsf(xd - x[0]) < 0.01f && fabsf(x[1]) < 0.1f) {
	              u = 0.0f;
	          }
			  else{

				  u = -(F[0] * x[0] + F[1] * x[1] + F[2] * x[2]) + (xd * N_bar);

	              }




	          if (u > 3.3f) u = 3.3f;
	          if (u < -3.3f) u = -3.3f;

	          if (u > 0.0f) {
	              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	          } else {
	              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	              u = -u;
	          }
	          dbg_u = u;
	          uint32_t max_pwm = __HAL_TIM_GET_AUTORELOAD(&htim4);
	          uint32_t u_pwm = (uint32_t)((u / 3.3f) * max_pwm);
	          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, u_pwm);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
