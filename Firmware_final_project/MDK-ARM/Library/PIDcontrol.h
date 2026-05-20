#ifndef __FP_PID_H__
#define __FP_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sPI
{
	float fDtSec;
	float fIn;
	float fKp;
	float fKi;
	float fUpOutLim;
	float fLowOutLim;
	float fPout;
	float fIout;
	float fIprevIn;
	float fIprevOut;
	float fOut;
	void (*m_calc)(struct sPI*);
	void (*m_rst)(struct sPI*);
} tPI;

#define PI_DEFAULTS {     \
	.fDtSec = 1.0f,      \
	.fIn = 0.0f,         \
	.fKp = 0.0f,         \
	.fKi = 0.0f,         \
	.fUpOutLim = 0.0f,   \
	.fLowOutLim = 0.0f,  \
	.fPout = 0.0f,       \
	.fIout = 0.0f,       \
	.fIprevIn = 0.0f,    \
	.fIprevOut = 0.0f,   \
	.fOut = 0.0f,        \
	.m_calc = tPI_calc,  \
	.m_rst = tPI_rst     \
}

void tPI_calc(tPI *ptPI);
void tPI_rst(tPI *ptPI);

#ifdef __cplusplus
}
#endif

#endif /* __FP_PID_H__ */
