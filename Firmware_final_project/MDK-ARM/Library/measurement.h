#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <stdint.h>

typedef struct
{
  float fVdc;
  float fIabc[3];        // [0]=U, [1]=V, [2]=W
  float fTemparature;

  uint16_t u16Offset_Ia; // offset raw Ia (calib)
  uint16_t u16Offset_Ib; // offset raw Ib (calib)
} Parameterhandle_t;

void Measurement_UpdateFromFPGA(Parameterhandle_t *pHandle);

#endif
