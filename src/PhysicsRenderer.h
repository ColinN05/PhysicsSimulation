#pragma once
#include "VecMath.h"
#include "Window.h"
#include "Shader.h"

class PhysicsRenderer
{
public:
	PhysicsRenderer();

	static void InitializeGLEW();
	void Init();
	void DetectCameraInput(float deltaTime);

	void RenderRigidBody(class RigidBody* rb);
	void RenderRigidBox(class RigidBox* rb);
	void RenderRigidSphere(class RigidSphere* rs);

	void RenderWorld(class PhysicsWorld* world);

	void SetCameraPosition(Vec3 pos);
	void SetCameraAngle(float theta, float phi);
	void SetCameraFOV(float fov);
	void SetCameraNear(float cameraNear);
	void SetCameraFar(float cameraFar);

	inline const Vec3 GetCameraPosition() { return CameraPosition; }

	void RecalculateCameraViewMatrix();
	void RecalculateCameraProjectionMatrix();

	void SetRenderWindow(Window* window);

	void ClearBuffers();
private:
	unsigned int CubeVertexArrayID, CubeVertexBufferID, CubeIndexBufferID;
	unsigned int SphereVertexArrayID, SphereVertexBufferID;
	int UVSphereRes = 20;
	Shader PrimaryShader;
	unsigned int TextureID;

	Vec3 CameraPosition = {0.0f,0.0f,0.0f};
	float CameraTheta = 0.0f, CameraPhi = 0.0f;
	Mat4 CameraViewMatrix;
	Mat4 CameraProjectionMatrix;
	float CameraNear = 0.1f, CameraFar = 2500.0f;
	float CameraFOV = CONST_PI / 4.0f;
	Window* RenderWindow = nullptr;
};