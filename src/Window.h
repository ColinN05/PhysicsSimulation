#pragma once
#include <windows.h>
#include <iostream>

struct Window
{
	HWND windowHandle;
	HDC deviceContext;
	HGLRC renderContext;

	bool Destroyed;

	Window(uint32_t width = 960, uint32_t height = 540);
	void Init();
	void Update();
	void SwapBuffers();

	uint32_t Width, Height;
	float TargetAspectRatio = 1920.0f/1080.0f;
};