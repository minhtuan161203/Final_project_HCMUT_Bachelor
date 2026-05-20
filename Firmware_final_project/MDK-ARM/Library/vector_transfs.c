#include "vector_transfs.h"

#define SQRT_3_F 1.73205080757f

void tFRClarke_ab2albe(tFRClarke *ptFRClarke)
{
	ptFRClarke->fAl = ptFRClarke->fA;
	ptFRClarke->fBe = (ptFRClarke->fA + (2.0f * ptFRClarke->fB)) / SQRT_3_F;
}

void tIFClarke_albe2abc(tIFClarke *ptIFClarke)
{
	ptIFClarke->fA = ptIFClarke->fAl;
	ptIFClarke->fB = 0.5f * ((-ptIFClarke->fAl) + (SQRT_3_F * ptIFClarke->fBe));
	ptIFClarke->fC = 0.5f * ((-ptIFClarke->fAl) - (SQRT_3_F * ptIFClarke->fBe));
}

void tFPark_albe2dq(tFPark *ptFPark)
{
	ptFPark->fD = (ptFPark->fAl * ptFPark->fCosAng) + (ptFPark->fBe * ptFPark->fSinAng);
	ptFPark->fQ = (ptFPark->fBe * ptFPark->fCosAng) - (ptFPark->fAl * ptFPark->fSinAng);
}

void tIPark_dq2albe(tIPark *ptIPark)
{
	ptIPark->fAl = (ptIPark->fD * ptIPark->fCosAng) - (ptIPark->fQ * ptIPark->fSinAng);
	ptIPark->fBe = (ptIPark->fQ * ptIPark->fCosAng) + (ptIPark->fD * ptIPark->fSinAng);
}
