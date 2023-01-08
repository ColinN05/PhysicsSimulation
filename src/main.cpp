#include <windows.h>
#include <iostream>
#include "Window.h"
#include "Physics.h"
#include "PhysicsRenderer.h"
#include "stb_image.h"
#include <cmath>
#include "Time.h"

int main()
{
	Window win = Window(800,600);

	Time::Init();

	PhysicsRenderer::InitializeGLEW();
	PhysicsRenderer renderer;
	renderer.Init();
	renderer.SetCameraPosition(Vec3(50.0f,45.0f,70.0f));
	renderer.SetCameraAngle(-0.6f, -0.25f);
	renderer.SetRenderWindow(&win);

	PhysicsWorld world;
	world.SetGravity(0.0f);

	for (int i = 0; i < 100; i++)
	{
		float x = ((double) rand() / (RAND_MAX));
		float y = ((double) rand() / (RAND_MAX));
		float z = ((double) rand() / (RAND_MAX)); 
		RigidSphere* sphere = world.SpawnRigidSphere(2.0f * Vec3(x,y,z));
		sphere->SetRadius(0.1f);
		sphere->SetMass(1000.0f);
		sphere->Material.Restitution = 0.6f;
		sphere->ApplyImpulse(Vec3(0.0f,0.0f,-10000.0f));
	}

	RigidBox* box = world.SpawnRigidBox(Vec3(5.0f,0.0f,-30.0f));
	box->SetExtent(Vec3(5.0f,5.0f,0.5f));
	box->SetMass(1000000.0f);
	box->Material.Restitution = 0.6;

	bool simulationRunning = false;

	float time;

	unsigned int count = 0;

	while (!win.Destroyed)
	{
		float deltaTime = Time::GetTime() - time;
		time = Time::GetTime();

		count++;
		if (count % 1000 == 1)
			std::cout << (int)(1.0f / deltaTime) << "fps\n";

		win.Update();

		if (simulationRunning)
			world.Step(deltaTime);

		if (GetAsyncKeyState(VK_RSHIFT)) // run simulation
			simulationRunning = true;
		if (GetAsyncKeyState(0x52)) // pause simulation
			simulationRunning = false;

		renderer.DetectCameraInput(deltaTime);
		renderer.ClearBuffers();
		renderer.RenderWorld(&world);

		win.SwapBuffers();
	}
}

/*
	for (int i = -2; i <= 2; i++)
	{
		for (int j = -2; j <= i; j++)
		{
			for (int k = -5; k < 5; k++)
			{
				RigidSphere* sphere = world.SpawnRigidSphere(Vec3(2.0f * j - i,2.1f * (float)k, (2.0f - i) * sqrtf(3.0f)));
				sphere->SetMass(10.0f);
				sphere->SetRadius(1.0f);
				sphere->Material.Restitution = 0.5f;
			}
		}
	}
*/