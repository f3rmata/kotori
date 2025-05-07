#ifndef DDS_STM32
#define DDS_STM32
#include <stdint.h>

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

void DDS_SetPhase(DDS *dds, float phase_deg, float phase_max);
void DDS_UpdateBuffer(uint16_t *buffer, DDS *dds, uint16_t offset,
                      uint16_t length);
void dds_free(DDS *dds);

#endif
