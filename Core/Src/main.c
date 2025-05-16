/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "comp.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad983x.h"
#include "arm_math.h"
#include "dds_stm32.h"
#include "wave.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define __FPU_PRESENT 1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SET_DAC_TIM_FREQ(TIM_HandleTypeDef *htim, uint16_t wavelen,
                      uint32_t freqk);
void ADD_DAC_TIM_PHASE(TIM_HandleTypeDef *htim, uint16_t freq, uint16_t phase);
void hamming_window(q15_t *pWindow);
uint16_t *generate_sine_wave_table(uint32_t table_size, uint16_t amplitudey);
uint16_t *generate_cosine_wave_table(uint32_t table_size, uint16_t amplitudey);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SAMPLE_LENGTH 1024
#define FFT_LENGTH 1024
#define TABLE_SIZE 256 // 波表长度
#define OF 2147.483648 // 2^32 / 2M，用于频率控制字计算
#define PHASE_MAX ((1UL << PHASE_BITS) - 1)
#define PHASE_STEP(freq)                                                       \
  ((uint32_t)((((double)(freq)) * (double)(1ULL << PHASE_BITS)) / SYS_CLK_FREQ))
#define PHASE_BITS 32          // 相位累加器位宽
#define OUTPUT_FREQ 10000      // 默认输出频率(Hz)
#define SYS_CLK_FREQ 2000000UL // DAC采样频率(由TIM6决定)

#define Q15_SBIT 0x8000
#define Q15_MBITS 0x7FFF
#define Q15_MULT 0x8000
#define Q15_PMAX 0x7FFF
#define Q15_NMAX 0x8000

uint16_t adc_value[SAMPLE_LENGTH] = {0};
q15_t fft_in[FFT_LENGTH] = {0};
q15_t fft_out[FFT_LENGTH * 2] = {0};
q15_t cmplx_mag_out[FFT_LENGTH * 2] = {0};
// q15_t cmplx_ang_out[FFT_LENGTH * 2] = {0};
uint8_t adc_conv_finished = 0;
uint8_t fft_finished = 0;
uint8_t cal_phase_err_finished = 1;
uint16_t max_freq = 100;
uint16_t sec_freq = 100;

// DDS *stm32_dds;
uint16_t *sintable = NULL;
uint16_t *costable = NULL;

// AD983X ad9834;
uint8_t comp_flag = 0;
double phase_err = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_DAC3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP3_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_CORDIC_Init();
  MX_TIM7_Init();
  MX_COMP1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t str[] = "\r\n-------------USART_Sending------------------\r\n";
  HAL_UART_Transmit(&huart1, str, sizeof(str) / sizeof(str[0]), 40);

  // 初始化 stm32 dds 设置
  // stm32_dds = dds_init(dactable, TABLE_SIZE, 10000, 2000000);
  // // 设置初始频率
  // stm32_dds->phase_inc = PHASE_STEP(OUTPUT_FREQ);
  // // 填充初始DMA缓冲
  // DDS_UpdateBuffer(dactable, stm32_dds, 0, TABLE_SIZE);
  // HAL_TIM_Base_Start(&htim4);

  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp3);

  // AD983X_init(&ad9834, &hspi1, AD9834_FSYNC_GPIO_Port, AD9834_FSYNC_Pin,
  //             AD9834_RST_GPIO_Port, AD9834_RST_Pin, (uint8_t)75);
  // AD983X_setFrequency(&ad9834, 0, 100000.0);

  // AD983X_setSleep(&ad9834, SLEEP_MODE_NONE);
  // AD983X_setOutputMode(&ad9834, OUTPUT_MODE_TRIANGLE);
  // AD983X_setSignOutput(&ad9834, SIGN_OUTPUT_MSB);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  arm_rfft_instance_q15 forwardS;
  // arm_rfft_instance_q15 backwardS;
  arm_rfft_init_q15(&forwardS, FFT_LENGTH, 0, 1);
  // arm_rfft_init_q15(&backwardS, FFT_LENGTH, 1, 0);
  // arm_rfft_init_1024_q15(&S, 0, 0);

  // 从 0 相位开始采集
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, SAMPLE_LENGTH);
  // HAL_COMP_Start(&hcomp1);
  // while (!comp_flag)
  //   ;
  // uint16_t adc_single_value = 65534;
  // HAL_ADC_Start(&hadc1);
  // do {
  //   HAL_ADC_PollForConversion(&hadc1, 100);
  //   // 判断ADC是否转换成功
  //   if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
  //     // 读取值
  //     adc_single_value = HAL_ADC_GetValue(&hadc1);
  //   }
  // } while (adc_single_value > 2058 || adc_single_value < 2038);
  // HAL_ADC_Stop(&hadc1);

  uint16_t index0 = 0;
  while (1) {
    if (adc_conv_finished == 1) {
      if (fft_finished == 0) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        for (int i = 0; i < SAMPLE_LENGTH; i++) {
          uint8_t adc_fmt_tmp_str[20] = {0};
          sprintf((char *)adc_fmt_tmp_str, "adc: %d\n", adc_value[i]);
          HAL_UART_Transmit(&huart1, adc_fmt_tmp_str,
                            strlen((char *)adc_fmt_tmp_str), 40);
        }
        // memcpy(fft_in, adc_value, FFT_LENGTH);
        for (size_t i = 0; i < FFT_LENGTH; ++i) {
          fft_in[i] = (int16_t)((int32_t)adc_value[i] - 32768);
        }

        q15_t h_window[FFT_LENGTH] = {0};
        hamming_window(h_window);
        arm_mult_q15(fft_in, h_window, fft_in, FFT_LENGTH);
        arm_rfft_q15(&forwardS, fft_in, fft_out);
        arm_cmplx_mag_q15(fft_out, cmplx_mag_out, FFT_LENGTH / 2 + 1);

        uint8_t fft_banner_str[] =
            "\r\n-------------FFT_Result_Sending------------------\r\n";
        HAL_UART_Transmit(&huart1, fft_banner_str,
                          strlen((char *)fft_banner_str), 40);
        for (int i = 0; i < FFT_LENGTH / 16; i++) {
          uint8_t fft_fmt_tmp_str[50] = {0};
          sprintf((char *)fft_fmt_tmp_str, "fft(%dk) : %d\n", i * 2,
                  cmplx_mag_out[i] * 10);
          HAL_UART_Transmit(&huart1, fft_fmt_tmp_str,
                            strlen((char *)fft_fmt_tmp_str), 40);
        }

        // 去除直流分量
        cmplx_mag_out[0] = 0;
        cmplx_mag_out[1] = 0;
        cmplx_mag_out[2] = 0;
        cmplx_mag_out[3] = 0;
        cmplx_mag_out[4] = 0;

        // 查找频谱的第一个峰值
        q15_t max_result = 0;
        uint32_t max_index = 0;
        arm_max_q15(cmplx_mag_out, FFT_LENGTH / 2 + 1, &max_result, &max_index);
        cmplx_mag_out[max_index] = 0;

        // 查找频谱的第二个峰值
        q15_t max_second_result = 0;
        uint32_t max_second_index = 0;
        arm_max_q15(cmplx_mag_out, FFT_LENGTH / 2 + 1, &max_second_result,
                    &max_second_index);

        max_freq = (max_index + 1) * 2;
        sec_freq = (max_second_index + 1) * 2;
        // 将频率近似到5k的整数倍
        max_freq = ((max_freq + 2) / 5) * 5;
        sec_freq = ((sec_freq + 2) / 5) * 5;
        uint8_t cmp_str_buf[50] = {0};
        sprintf((char *)cmp_str_buf, "max: %uk, sec: %uk\r\n", max_freq,
                sec_freq);
        HAL_UART_Transmit(&huart1, cmp_str_buf, strlen((char *)cmp_str_buf),
                          40);

        // AD983X_setFrequency(&ad9834, 0, max_freq * 1000);
        // SET_DAC_TIM_FREQ(&htim7, max_freq);
        // SET_DAC_TIM_FREQ(&htim15, sec_freq);
        SET_DAC_TIM_FREQ(&htim7, 100, max_freq * 1000);
        SET_DAC_TIM_FREQ(&htim15, 100, sec_freq * 1000);
        HAL_TIM_Base_Start(&htim7);
        HAL_TIM_Base_Start(&htim15);
        sintable = generate_sine_wave_table(100, 2048);
        costable = generate_cosine_wave_table(100, 2048);
        // uint8_t dactable_str_buf[50] = {0};
        // for (uint16_t i = 0; i < 100; i++) {
        //   sprintf((char *)dactable_str_buf, "dactable[%d]: %d\n", i,
        //   sintable[i]); HAL_UART_Transmit(&huart1, dactable_str_buf,
        //                     strlen((char *)dactable_str_buf), 40);
        // }
        HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_1, (uint32_t *)sintable, 100,
                          DAC_ALIGN_12B_R);
        HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_2, (uint32_t *)sintable, 100,
                          DAC_ALIGN_12B_R);

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, SAMPLE_LENGTH);
        adc_conv_finished = 0;
        fft_finished = 1;
      } else if (fft_finished && !cal_phase_err_finished) {
        // 找到 0 相位点
        // for (uint16_t i = 0; i < SAMPLE_LENGTH; i++) {
        //   if ((adc_value[i] >> 4) > 2046 && (adc_value[i] >> 4) < 2050) {
        //     index0 = i;
        //     break;
        //   }
        // }
        index0 = 0;

        uint16_t sum_sin = 0;
        uint16_t sum_cos = 0;
        for (uint8_t i = 0; i < 100; i++) {
          sum_sin +=
              ((adc_value[index0 + i] >> 4) - 2048) * (sintable[i] - 2048);
          sum_cos +=
              ((adc_value[index0 + i] >> 4) - 2048) * (costable[i] - 2048);
        }
        phase_err = atan2(sum_sin, sum_cos);
        // if (phase_err < 0) {
        //   phase_err = -phase_err;
        // } else {
        //   phase_err = 2 * PI - phase_err;
        // }
        // uint8_t phase_str[50] = {0};
        // sprintf((char *)phase_str, "phase_err: %f\n", phase_err);
        // HAL_UART_Transmit(&huart1, phase_str, 50, 50);

        ADD_DAC_TIM_PHASE(&htim7, max_freq * 1000, phase_err);
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, SAMPLE_LENGTH);

        adc_conv_finished = 0;
        cal_phase_err_finished = 1;
        // SET_DAC_TIM_FREQ(&htim7, 100, max_freq * 1000, phase_err);
        // HAL_Delay(10);
      }
    }
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, SAMPLE_TIMES);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  // dds_free(stm32_dds);
  free(sintable);
  free(costable);
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    adc_conv_finished = 1;
  }
}

// void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
//   if (hdac == &hdac3) {
//     DDS_UpdateBuffer(dactable, stm32_dds, 0, TABLE_SIZE / 2);
//   }
// }
//
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  if (hdac == &hdac3) {
    cal_phase_err_finished = 0;
    // 立即开始新的ADC采样，确保持续采集数据
    if (!adc_conv_finished) {
      HAL_ADC_Stop_DMA(&hadc1);
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, SAMPLE_LENGTH);
    }
  }
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  if (hcomp == &hcomp1) {
    comp_flag = 1;
    // __HAL_TIM_SetCounter(&htim3, 0);
    // HAL_DAC_Stop_DMA(&hdac3, DAC_CHANNEL_1);
    // HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_1, (uint32_t *)sin_wave,
    // WAVE_LENGTH,
    //                   DAC_ALIGN_12B_R);
    // HAL_DAC_Stop_DMA(&hdac3, DAC_CHANNEL_2);
    // HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_2, (uint32_t *)sin_wave,
    // WAVE_LENGTH,
    //                   DAC_ALIGN_12B_R);
    // __HAL_TIM_SET_COUNTER(&htim7, 0);
    // __HAL_TIM_SET_COUNTER(&htim15, 0);

    // uint16_t i = 100;
    // while (i > 0) {
    //   i--;
    // }
  }
}

q15_t float2q15(float f) {
  if (f >= 1.0)
    return Q15_PMAX;
  if (f <= -1.0)
    return Q15_NMAX;

  if (f < 0) {
    return -(Q15_MBITS & (int16_t)(-f * Q15_MULT));
  } else {
    return Q15_MBITS & (int16_t)(f * Q15_MULT);
  }
}

void hamming_window(q15_t *pWindow) {
  float alpha = 25.0 / 46;
  float factor = 2.0 * PI / FFT_LENGTH;
  for (int i = 0; i < FFT_LENGTH; i++) {
    float f = alpha - (1.0 - alpha) * arm_cos_f32(factor * i);
    pWindow[i] = float2q15(f);
  }
}

void SET_DAC_TIM_FREQ(TIM_HandleTypeDef *htim, uint16_t wavelen,
                      uint32_t freqk) {
  uint32_t arr = 160000000 / (wavelen * freqk) - 1;
  __HAL_TIM_SET_AUTORELOAD(htim, arr);
}

void ADD_DAC_TIM_PHASE(TIM_HandleTypeDef *htim, uint16_t freq, uint16_t phase) {
  uint32_t cnt = htim->Instance->CNT;
  cnt = cnt + (65535 - (phase * 160000000) / (2 * PI * freq));
  __HAL_TIM_SetCounter(htim, cnt);
}

uint16_t *generate_sine_wave_table(uint32_t table_size, uint16_t amplitude) {
  // 分配波表内存
  uint16_t *wave_table = (uint16_t *)malloc(table_size * sizeof(uint16_t));
  if (wave_table == NULL) {
    return NULL; // 内存分配失败
  }

  // 计算角频率
  float omega = 2.0f * M_PI / table_size;

  // 填充波表
  for (uint32_t i = 0; i < table_size; i++) {
    float value = amplitude * (sin(omega * i) + 1.0f) / 2.0f +
                  100; // 将值映射到 [100, amplitude]
    wave_table[i] = (uint16_t)value;
  }

  return wave_table;
}

uint16_t *generate_cosine_wave_table(uint32_t table_size, uint16_t amplitude) {
  // 分配波表内存
  uint16_t *wave_table = (uint16_t *)malloc(table_size * sizeof(uint16_t));
  if (wave_table == NULL) {
    return NULL; // 内存分配失败
  }

  // 计算角频率
  float omega = 2.0f * M_PI / table_size;

  // 填充波表
  for (uint32_t i = 0; i < table_size; i++) {
    float value = amplitude * (cos(omega * i) + 1.0f) / 2.0f +
                  100; // 将值映射到 [100, amplitude]
    wave_table[i] = (uint16_t)value;
  }

  return wave_table;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
