#include "window.h"
#include <iostream>
#include <GL/glew.h>

LRESULT CALLBACK WndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    LONG_PTR windowLongPtr = GetWindowLongPtr(hwnd, GWLP_USERDATA);
    Window* window = reinterpret_cast<Window*>(windowLongPtr);

    switch (uMsg)
    {
        case WM_CREATE:
        {
            CREATESTRUCT* createStruct = reinterpret_cast<CREATESTRUCT*>(lParam);
            Window* window = reinterpret_cast<Window*>(createStruct->lpCreateParams);
            SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)window);
            break;
        }
        case WM_DESTROY:
        {
            if (!window) break;
            window->Destroyed = true;
            break;
        }
    	case WM_SIZE:
    	{
            if (!window) break;

            int resizeWidth = LOWORD(lParam);
            int resizeHeight = HIWORD(lParam);

            window->Width = resizeWidth; window->Height = resizeHeight;

            float r = window->TargetAspectRatio;
            glViewport((resizeWidth- r * resizeHeight) / 2, 0, r * resizeHeight,resizeHeight);
    		break;
    	}
    	default:
    	{
    		return DefWindowProc(hwnd, uMsg, wParam, lParam);
    	}
    }
    return (long long int)NULL;
}

void Window::Update()
{
    MSG msg;
    while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) > 0)
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    Sleep(0);
}

Window::Window(uint32_t width, uint32_t height)
{
    const wchar_t CLASS_NAME[] = L"Sample Window Class";

    Destroyed = false;

    WNDCLASS windowClass = {};

    windowClass.lpfnWndProc = WndProc;
    windowClass.hInstance = NULL;
    windowClass.lpszClassName = (LPCSTR)CLASS_NAME;
    windowClass.style = CS_OWNDC;

    RegisterClass(&windowClass);

    windowHandle = CreateWindowEx(
        0,
        (LPCSTR)CLASS_NAME,
        (LPCSTR)"Title",
        WS_OVERLAPPEDWINDOW,

        CW_USEDEFAULT, CW_USEDEFAULT, width, height,

        NULL,
        NULL,
        NULL,
        (void*)this
    );

    Width = width; Height = height;
    TargetAspectRatio = (float)width/(float)height;


    if (windowHandle == NULL)
    {
        // window creation failed
        std::cout << "Failed to create window!\n";
    }

    PIXELFORMATDESCRIPTOR pfd =
    {
        sizeof(PIXELFORMATDESCRIPTOR),
        1,
        PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,    // Flags
        PFD_TYPE_RGBA,        // The kind of framebuffer. RGBA or palette.
        32,                   // Colordepth of the framebuffer.
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        24,                   // Number of bits for the depthbuffer
        8,                    // Number of bits for the stencilbuffer
        0,                    // Number of Aux buffers in the framebuffer.
        PFD_MAIN_PLANE,
        0, 0, 0, 0
    };

    HDC dc = GetDC(windowHandle);
    int pf = ChoosePixelFormat(dc, &pfd);
    SetPixelFormat(dc, pf, &pfd);
    HGLRC rc = wglCreateContext(dc);
    wglMakeCurrent(dc, rc);

    deviceContext = dc;
    renderContext = rc;

    ShowWindow(windowHandle, SW_SHOW);
    UpdateWindow(windowHandle);
}

void Window::SwapBuffers()
{
    wglSwapLayerBuffers(deviceContext, WGL_SWAP_MAIN_PLANE);
}