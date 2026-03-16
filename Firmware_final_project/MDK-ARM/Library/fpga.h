#ifndef FPGA_H
#define FPGA_H

#include <stdint.h>

typedef struct
{
  uint16_t cur_u_raw;
  uint16_t cur_v_raw;
  uint16_t temp_raw;
  uint16_t vdc_raw;
  uint16_t enc_id_raw;
  uint16_t enc_rx_word[7];
  uint16_t fpga_error_raw;
} fpga_meas_raw_t;

extern void FPGA_ReadAllRaw(fpga_meas_raw_t *out);


#endif
