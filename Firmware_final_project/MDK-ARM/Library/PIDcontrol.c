#include "PIDcontrol.h"

void tPI_calc(tPI *ptPI)
{
	float fPreOut;
	float fIin;
	float fIout;

	ptPI->fPout = ptPI->fIn * ptPI->fKp;

	fIin = ptPI->fIn * ptPI->fKi;
	fIout = ptPI->fIprevOut + (0.5f * ptPI->fDtSec * (fIin + ptPI->fIprevIn));
	ptPI->fIprevIn = fIin;

	fPreOut = ptPI->fPout + fIout;

	if (fPreOut > ptPI->fUpOutLim)
	{
		fPreOut = ptPI->fUpOutLim;
		fIout = fPreOut - ptPI->fPout;
	}
	if (fPreOut < ptPI->fLowOutLim)
	{
		fPreOut = ptPI->fLowOutLim;
		fIout = fPreOut - ptPI->fPout;
	}

	ptPI->fIout = fIout;
	ptPI->fIprevOut = fIout;
	ptPI->fOut = fPreOut;
}

void tPI_rst(tPI *ptPI)
{
	ptPI->fIn = 0.0f;
	ptPI->fIout = 0.0f;
	ptPI->fIprevIn = 0.0f;
	ptPI->fIprevOut = 0.0f;
	ptPI->fOut = 0.0f;
	ptPI->fPout = 0.0f;
}
