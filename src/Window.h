#include <windows.h>
#include <iostream>

struct Window
{
	HWND windowHandle;
	HDC deviceContext;
	HGLRC renderContext;

	static bool Destroyed;

	Window();
	void Init();
	void Update();
	void SwapBuffers();
};