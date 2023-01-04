#include <windows.h>
#include <iostream>
#include "Window.h"
#include "Physics.h"
#include "stb_image.h"
#include <cmath>
#include "Time.h"
#include <algorithm>

bool Window::Destroyed;

int main()
{
	Window win = Window();

	Time::Init();

	PhysicsRenderer::InitializeGLEW();
	PhysicsRenderer renderer;
	renderer.Init();
	renderer.SetCameraPosition(Vec3(0.0f,0.0f,15.0f));

	PhysicsWorld world;
	world.Gravity = 0.0f;

/*

	for (int i = 2; i >= -2; i--)
	{
		for (int j = -2; j <= i; j++)
		{
			for (int k = 0; k < 5; k++)
			{
				RigidSphere* sphere = new RigidSphere;
				sphere->SetMass(100.0f);
				sphere->SetRadius(1.0f);
				world.SpawnRigidBody(sphere, Vec3(2.0f * j + 2.0f - i,2.1f * (float)k, (2.0f - i) * sqrtf(3.0f)));	
			}
		}
	}

	RigidBox* box = new RigidBox;
	box->SetExtent(Vec3(40.0f, 20.0f, 1.0f));
	box->SetMass(10000.0f);
	box->AngularMomentum = {0.0f, 10000000.0f, 10000.0f};
	world.SpawnRigidBody(box, Vec3(-10.0f,-10.0f,-5.0f));

	RigidBox* box2 = new RigidBox;
	box2->SetExtent(Vec3(40.0f, 20.0f, 1.0f));
	box2->SetMass(10000.0f);
	box2->AngularMomentum = {0.0f, -10000000.0f, 10000.0f};
	world.SpawnRigidBody(box2, Vec3(-10.0f,-10.0f,5.0f));

*/

	RigidBox* box1 = new RigidBox;
	box1->SetExtent(Vec3(0.2f, 5.0f, 12.0f));
	box1->SetMass(100.0f);
	world.SpawnRigidBody(box1, Vec3(-10.0f,0.0f,0.0f));
	ApplyImpulse(box1, Vec3(200.0f,0.0f,0.0f));
	box1->AngularMomentum = {50.0f,0.0f,100.0f};

	RigidBox* box2 = new RigidBox;
	box2->SetExtent(Vec3(1.0f, 1.0f, 7.0f));
	box2->SetMass(100.0f);
	world.SpawnRigidBody(box2, Vec3(10.0f,0.0f,0.0f));
	ApplyImpulse(box2, Vec3(-200.0f,0.0f,0.0f));
	box2->AngularMomentum = {20.0f,50.0f,100.0f};

	bool simulationRunning = false;

	float time;

	unsigned int count = 0;

	while (!Window::Destroyed)
	{
		float deltaTime = Time::GetTime() - time;
		time = Time::GetTime();

		count++;
		if (count % 100 == 1)
		{
			std::cout << 1.0f/deltaTime << " fps\n";
		}

		win.Update();

		if (simulationRunning)
		{
			world.Step(deltaTime);
		}

		if (GetAsyncKeyState(VK_RSHIFT))
			simulationRunning = true;
		if (GetAsyncKeyState(0x52))
		{
			simulationRunning = false;
		}

		renderer.DetectCameraInput(deltaTime);
		renderer.ClearBuffers();

		for (RigidBody* rigidBody : world.RigidBodies)
		{
			renderer.RenderRigidBody(rigidBody);
		}

		win.SwapBuffers();
	}
}