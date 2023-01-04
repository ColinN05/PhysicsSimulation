#include "Time.h"

double Time::PCFreq_ms;
__int64 Time::PCStartCount = 0;

void Time::Init()
{
	LARGE_INTEGER li;
	QueryPerformanceFrequency(&li);
	PCFreq_ms = double(li.QuadPart) / 1000000.0;

	QueryPerformanceCounter(&li);
	PCStartCount = li.QuadPart;
}

float Time::GetTime()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	double t = double(li.QuadPart - PCStartCount) / PCFreq_ms;
	return (float)t/1000000.0f;
}