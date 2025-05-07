#include "dds_stm32.h"
#include "arm_math.h"
#include <stdlib.h>

// 创建波表，放在堆上
uint16_t *create_wavetable(uint32_t size) {
  uint16_t *table = (uint16_t *)malloc(sizeof(uint16_t) * size);
  if (table == NULL) {
    // exit(1);
    return NULL;
  }
  for (uint32_t i = 0; i < size; i++) {
    table[i] = 768 * (arm_sin_f32(2 * PI * i / size) + 1) + 100;
  }
  return table;
}

// DDS初始化
DDS *dds_init(uint16_t *dactable, uint32_t table_size, uint32_t freq,
              uint32_t sample_rate) {
  DDS *dds = (DDS *)malloc(sizeof(DDS));
  if (dds == NULL) {
    // exit(1);
    return NULL;
  }
  dds->wavetable = create_wavetable(table_size);
  memcpy(dactable, dds->wavetable, table_size);
  dds->table_size = table_size;
  dds->phase_acc = 0;
  // 计算相位步进
  dds->phase_inc = (uint32_t)(freq * table_size / sample_rate);
  return dds;
}

void DDS_SetPhase(DDS *dds, float phase_deg, float phase_max) {
  // phase_deg: 0~360°
  uint32_t phase_word = (uint32_t)((phase_deg / 360.0f) * (float)phase_max);
  dds->phase_acc += phase_word;
}

void DDS_UpdateBuffer(uint16_t *buffer, DDS *dds, uint16_t offset,
                      uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    // 取相位累加器高8位作为波表索引
    buffer[offset + i] = dds->wavetable[(dds->phase_acc >> 24) & 0xFF];
    dds->phase_acc += dds->phase_inc;
  }
}

// 释放DDS
void dds_free(DDS *dds) {
  if (dds) {
    if (dds->wavetable)
      free(dds->wavetable);
    free(dds);
  }
}

/**
// 示例：主函数
int main() {
  double freq = 1000.0;         // 输出频率1kHz
  double sample_rate = 48000.0; // 采样率48kHz
  DDS *dds = dds_init(TABLE_SIZE, freq, sample_rate);

  // 生成100个采样点
  for (int i = 0; i < 100; i++) {
    double sample = dds_output(dds);
    printf("%f\n", sample);
  }

  // 释放资源
  dds_free(dds);
  return 0;
}
*/
