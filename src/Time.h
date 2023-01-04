#pragma once
#include <windows.h>

class Time
{
public:
	static void Init();
	static float GetTime();
private:
	static double PCFreq_ms;
	static __int64 PCStartCount;
};