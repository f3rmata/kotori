#ifndef __AD9834_H__
#define __AD9834_H__
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include <stdbool.h>
#include <stdint.h>

//--------------------- 枚举类型定义 ---------------------

typedef enum {
  SIGN_OUTPUT_NONE = 0x0000,
  SIGN_OUTPUT_MSB = 0x0028,
  SIGN_OUTPUT_MSB_2 = 0x0020,
  SIGN_OUTPUT_COMPARATOR = 0x0038,
} SignOutput;

typedef enum {
  OUTPUT_MODE_SINE = 0x0000,
  OUTPUT_MODE_TRIANGLE = 0x0002,
} OutputMode;

typedef enum {
  SLEEP_MODE_NONE = 0x0000,
  SLEEP_MODE_MCLK = 0x0080,
  SLEEP_MODE_DAC = 0x0040,
  SLEEP_MODE_ALL = 0x00C0,
} SleepMode;

//--------------------- 基础结构体 AD983X ---------------------

typedef struct AD983X AD983X;

struct AD983X {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *m_select_port;
  uint16_t m_select_pin;
  GPIO_TypeDef *m_reset_port;
  uint16_t m_reset_pin;
  uint16_t m_reg;
  uint32_t m_clk_scaler;
};

//--------------------- 相关函数实现 ---------------------

void AD983X_init(AD983X *self, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *select_port, uint16_t select_pin,
                 GPIO_TypeDef *reset_port, uint16_t reset_pin, uint8_t clk_mhz);
void AD983X_setFrequency(AD983X *self, uint8_t reg, float frequency);
void AD983X_setFrequencyWord(AD983X *self, uint8_t reg, float frequency);
void AD983X_setPhaseWord(AD983X *self, uint8_t reg, uint32_t phase);
void AD983X_setSignOutput(AD983X *self, SignOutput out);
void AD983X_setOutputMode(AD983X *self, OutputMode out);
void AD983X_setSleep(AD983X *self, SleepMode out);
void AD983X_reset(AD983X *self);
void AD983X_ctor(AD983X *self, uint16_t select_pin, uint16_t reset_pin,
                 uint8_t clk_mhz);
void AD983X_writeReg(AD983X *self, uint16_t value);

#endif
