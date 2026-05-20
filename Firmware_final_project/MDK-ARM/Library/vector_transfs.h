#ifndef __VECTOR_TRANSFS_H__
#define __VECTOR_TRANSFS_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sFRClarke
{
	float fA;
	float fB;
	float fAl;
	float fBe;
	void (*m_ab2albe)(struct sFRClarke*);
} tFRClarke;

typedef struct sIFClarke
{
	float fAl;
	float fBe;
	float fA;
	float fB;
	float fC;
	void (*m_albe2abc)(struct sIFClarke*);
} tIFClarke;

typedef struct sFPark
{
	float fAl;
	float fBe;
	float fSinAng;
	float fCosAng;
	float fD;
	float fQ;
	void (*m_albe2dq)(struct sFPark*);
} tFPark;

typedef struct sIPark
{
	float fD;
	float fQ;
	float fSinAng;
	float fCosAng;
	float fAl;
	float fBe;
	void (*m_dq2albe)(struct sIPark*);
} tIPark;

#define FR_CLARKE_DEFAULTS {        \
	.fA = 0.0f,                    \
	.fB = 0.0f,                    \
	.fAl = 0.0f,                   \
	.fBe = 0.0f,                   \
	.m_ab2albe = tFRClarke_ab2albe \
}

#define IF_CLARKE_DEFAULTS {         \
	.fAl = 0.0f,                    \
	.fBe = 0.0f,                    \
	.fA = 0.0f,                     \
	.fB = 0.0f,                     \
	.fC = 0.0f,                     \
	.m_albe2abc = tIFClarke_albe2abc \
}

#define F_PARK_DEFAULTS {         \
	.fAl = 0.0f,                  \
	.fBe = 0.0f,                  \
	.fSinAng = 0.0f,              \
	.fCosAng = 0.0f,              \
	.fD = 0.0f,                   \
	.fQ = 0.0f,                   \
	.m_albe2dq = tFPark_albe2dq   \
}

#define I_PARK_DEFAULTS {         \
	.fD = 0.0f,                   \
	.fQ = 0.0f,                   \
	.fSinAng = 0.0f,              \
	.fCosAng = 0.0f,              \
	.fAl = 0.0f,                  \
	.fBe = 0.0f,                  \
	.m_dq2albe = tIPark_dq2albe   \
}

void tFRClarke_ab2albe(tFRClarke *ptFRClarke);
void tIFClarke_albe2abc(tIFClarke *ptIFClarke);
void tFPark_albe2dq(tFPark *ptFPark);
void tIPark_dq2albe(tIPark *ptIPark);

#ifdef __cplusplus
}
#endif

#endif /* __VECTOR_TRANSFS_H__ */
