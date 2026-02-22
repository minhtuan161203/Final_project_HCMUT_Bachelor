#include "fpga.h"
#include "define.h"
#include "stm32f4xx_hal.h"

static inline uint16_t FPGA_RD16(uint32_t addr)
{
  return *(__IO uint16_t*)(addr);
}

void FPGA_ReadAllRaw(fpga_meas_raw_t *out)
{
  if(out == 0) return;

  out->cur_u_raw      = FPGA_RD16(REG_CURRENT_PHASE_U);
  out->cur_v_raw      = FPGA_RD16(REG_CURRENT_PHASE_V);
  out->temp_raw       = FPGA_RD16(REG_TEMPARATURE_SENSOR);
  out->vdc_raw        = FPGA_RD16(REG_DC_BUS_VOLTAGE);
  out->enc_id_raw     = FPGA_RD16(REG_ENCODER_ID);

  out->enc_rx_word[0] = FPGA_RD16(REG_ENCODER_RX_WORD0);
  out->enc_rx_word[1] = FPGA_RD16(REG_ENCODER_RX_WORD1);
  out->enc_rx_word[2] = FPGA_RD16(REG_ENCODER_RX_WORD2);
  out->enc_rx_word[3] = FPGA_RD16(REG_ENCODER_RX_WORD3);
  out->enc_rx_word[4] = FPGA_RD16(REG_ENCODER_RX_WORD4);
  out->enc_rx_word[5] = FPGA_RD16(REG_ENCODER_RX_WORD5);
  out->enc_rx_word[6] = FPGA_RD16(REG_ENCODER_RX_WORD6);

  out->fpga_error_raw = FPGA_RD16(REG_FPGA_ERROR);
}
