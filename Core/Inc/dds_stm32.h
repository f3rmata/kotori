#ifndef DDS_STM32
#define DDS_STM32
#include <stdint.h>

// 一些宏定义
#define TABLE_SIZE 256 // 波表长度

typedef struct {
  uint16_t *wavetable; // 指向堆上波表的指针
  // uint16_t *dactable;  // 指向堆上DAC输出波表的指针
  uint32_t table_size;
  uint32_t phase_acc; // 相位累加器
  uint32_t phase_inc; // 相位增量
} DDS;

uint16_t *create_wavetable(uint32_t size);
DDS *dds_init(uint16_t *dactable, uint32_t table_size, uint32_t freq,
              uint32_t sample_rate);
uint16_t dds_output(DDS *dds);
void dds_free(DDS *dds);

#endif
