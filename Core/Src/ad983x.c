#include "ad983x.h"
#include <stdbool.h>
#include <stdint.h>

// ------------------ 寄存器定义 ------------------
#define REG_FREQ1 0x8000
#define REG_FREQ0 0x4000
#define REG_PHASE0 0xC000
#define REG_PHASE1 0xE000

#define REG_B28 0x2000
#define REG_HLB 0x1000
#define REG_FSEL 0x0800
#define REG_PSEL 0x0400
#define REG_PINSW 0x0200
#define REG_RESET 0x0100
#define REG_SLEEP1 0x0080
#define REG_SLEEP12 0x0040
#define REG_OPBITEN 0x0020
#define REG_SIGNPIB 0x0010
#define REG_DIV2 0x0008
#define REG_MODE 0x0002

#define SIGN_OUTPUT_MASK (REG_OPBITEN | REG_SIGNPIB | REG_DIV2 | REG_MODE)
#define SLEEP_MASK (REG_SLEEP1 | REG_SLEEP12)

// ------------------ AD983X 基础函数 ------------------

void AD983X_writeReg(AD983X *self, uint16_t value) {
  HAL_GPIO_WritePin(self->m_select_port, self->m_select_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(self->hspi, (uint8_t *)&value, 2, 40);
  HAL_GPIO_WritePin(self->m_select_port, self->m_select_pin, GPIO_PIN_SET);
}

void AD983X_writeReg2(AD983X *self, uint16_t value1, uint16_t value2) {
  uint16_t value[2] = {value1, value2};
  HAL_GPIO_WritePin(self->m_select_port, self->m_select_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(self->hspi, (uint8_t *)value, 4, 40);
  HAL_GPIO_WritePin(self->m_select_port, self->m_select_pin, GPIO_PIN_SET);
}

void AD983X_setFrequencyWord(AD983X *self, uint8_t reg, float frequency) {
  uint16_t reg_freq = reg ? REG_FREQ1 : REG_FREQ0;
  AD983X_writeReg2(self, reg_freq | ((uint32_t)frequency & 0x3FFF),
                   reg_freq | (((uint32_t)frequency >> 14) & 0x3FFF));
}

void AD983X_setPhaseWord(AD983X *self, uint8_t reg, uint32_t phase) {
  AD983X_writeReg(self, (reg ? REG_PHASE1 : REG_PHASE0) | (phase & 0x0FFF));
}

void AD983X_setSignOutput(AD983X *self, SignOutput out) {
  self->m_reg = (self->m_reg & ~SIGN_OUTPUT_MASK) | out;
  AD983X_writeReg(self, self->m_reg);
}

void AD983X_setOutputMode(AD983X *self, OutputMode out) {
  if (out == OUTPUT_MODE_TRIANGLE) {
    self->m_reg = (self->m_reg & ~SIGN_OUTPUT_MASK) | out;
  } else {
    self->m_reg &= ~REG_MODE;
  }
  AD983X_writeReg(self, self->m_reg);
}

void AD983X_setSleep(AD983X *self, SleepMode out) {
  self->m_reg = (self->m_reg & ~SLEEP_MASK) | out;
  AD983X_writeReg(self, self->m_reg & ~REG_PINSW);
}

void AD983X_reset(AD983X *self) {
  HAL_GPIO_WritePin(self->m_reset_port, self->m_reset_pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(self->m_reset_port, self->m_reset_pin, GPIO_PIN_RESET);
}

void AD983X_ctor(AD983X *self, uint16_t select_pin, float frequency_hz,
                 uint16_t reset_pin) {
  self->m_select_pin = select_pin;
  self->m_frequency_hz = frequency_hz;
  self->m_reg = REG_B28;
  HAL_GPIO_WritePin(self->m_select_port, self->m_select_pin, GPIO_PIN_SET);
  self->m_reset_pin = reset_pin;
  HAL_GPIO_WritePin(self->m_reset_port, self->m_reset_pin, GPIO_PIN_SET);
  self->m_reg |= REG_PINSW | REG_RESET;
}

void AD983X_setFrequency(AD983X *self, uint8_t reg, float frequency) {
  uint32_t freq_word =
      (uint32_t)((frequency * (1L << 28)) / (self->m_frequency_hz));
  AD983X_setFrequencyWord(self, reg, freq_word);
}

void AD983X_init(AD983X *self, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *select_port, uint16_t select_pin,
                 GPIO_TypeDef *reset_port, uint16_t reset_pin,
                 float frequency_mhz) {
  self->hspi = hspi;
  self->m_select_port = select_port;
  self->m_select_pin = select_pin;
  self->m_reset_port = reset_port;
  self->m_reset_pin = reset_pin;
  self->m_frequency_hz = frequency_mhz;
  AD983X_reset(self);
  AD983X_writeReg(self, self->m_reg);
  AD983X_setFrequency(self, 0, 10000);
  AD983X_setFrequency(self, 1, 10000);
  AD983X_setPhaseWord(self, 0, 0);
  AD983X_setPhaseWord(self, 1, 0);
}
