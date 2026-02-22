#include "measurement.h"
#include "define.h"
#include "stm32f4xx_hal.h"   // for __IO

static inline uint16_t FPGA_RD16(uint32_t addr)
{
  return *(__IO uint16_t*)(addr);
}

void Measurement_UpdateFromFPGA(Parameterhandle_t *pHandle)
{
  // 1) DC bus voltage: raw is unsigned around OFFSET
  pHandle->fVdc = (float)(FPGA_RD16(REG_DC_BUS_VOLTAGE) - OFFSET)
                  / Resolution16bits * INPUT_RANGE_VDC;

  // 2) Phase current U: raw unsigned, subtract calibrated offset, convert to Ampere
  pHandle->fIabc[0] = -(float)((int32_t)FPGA_RD16(REG_CURRENT_PHASE_U) - (int32_t)pHandle->u16Offset_Ia)
                      / Resolution16bits * INPUT_RANGE_I;

  // 3) Phase current V
  pHandle->fIabc[1] = -(float)((int32_t)FPGA_RD16(REG_CURRENT_PHASE_V) - (int32_t)pHandle->u16Offset_Ib)
                      / Resolution16bits * INPUT_RANGE_I;

  // 4) Phase current W: 2-shunt assumption → Iw = -(Iu + Iv)
  pHandle->fIabc[2] = -pHandle->fIabc[0] - pHandle->fIabc[1];

  // 5) Temperature: raw unsigned around OFFSET
  pHandle->fTemparature = (float)(FPGA_RD16(REG_TEMPARATURE_SENSOR) - OFFSET)
                          / Resolution16bits * INPUT_RANGE_TEMPARATURE;
}
