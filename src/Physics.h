#pragma once
#include "VecMath.h"
#include <vector>
#include <string>
#include "Shader.h"

struct PhysicsMaterial
{
	float Restitution = 0.75f, StaticFriction = 0.0f, DynamicFriction = 0.0f;
};

struct RigidBody
{
	float Mass = 1.0f;
	Mat3 Ibody, Ibodyinv;

	Vec3 Position = {0.0f,0.0f,0.0f};
	Quat QuatRotation = {1.0f,0.0f,0.0f,0.0f};

	Vec3 LinearMomentum = {0.0f,0.0f,0.0f}, AngularMomentum = {0.0f,0.0f,0.0f};

	Mat3 Iinv;
	Mat3 Rotation;

	Vec3 Force, Torque;

	PhysicsMaterial Material;

	virtual void SetMass(float mass);
	void SetPosition(Vec3 position);
	inline Vec3 GetPosition() {return Position;}
	virtual void RecalculateBodyInertiaTensor() = 0;

	Vec3 Velocity();
	Vec3 Omega();

	std::string Type;

	bool bSimulateGravity = true;
};

void ApplyImpulse(RigidBody* rb, Vec3 impulse);
void ApplyImpulseAtPoint(RigidBody* rb, Vec3 impulse, Vec3 point);
void ApplyForce(RigidBody* rb, Vec3 f);
void ApplyForceAtPoint(RigidBody* rb ,Vec3 f, Vec3 p);

struct RigidBox : public  RigidBody
{
	RigidBox();

	virtual void RecalculateBodyInertiaTensor() override;
	void SetExtent(Vec3 extent);

	float ExtentX, ExtentY, ExtentZ;
};

struct RigidSphere : public RigidBody
{
	RigidSphere();

	virtual void RecalculateBodyInertiaTensor() override;
	void SetRadius(float radius);
	float Radius = 1.0f;
};

struct CollisionPair
{
	RigidBody* Body1,* Body2;
};

struct PhysicsWorld
{
	void Step(float deltaTime);
	void SpawnRigidBody(RigidBody* rb,Vec3 position);

	std::vector<CollisionPair> CollisionBroadPhase();
	void CollisionNarrowPhaseAndResolve(std::vector<CollisionPair>& potentialCollisions);

	std::vector<RigidBody*> RigidBodies;

	float Gravity = 9.81f;
};

#define PHYSICS_TOLERANCE 0.005f

class PhysicsRenderer
{
public:
	PhysicsRenderer();

	static void InitializeGLEW();
	void Init();
	void DetectCameraInput(float deltaTime);

	void RenderRigidBody(RigidBody* rb);
	void RenderRigidBox(RigidBox* rb);
	void RenderRigidSphere(RigidSphere* rs);

	void SetCameraPosition(Vec3 pos);
	inline const Vec3 GetCameraPosition() { return CameraPosition; }
	void RecalculateCameraViewMatrix();
	void RecalculateCameraProjectionMatrix();

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
	float CameraNear = 0.1f, CameraFar = 25000.0f;
	float CameraFOV = CONST_PI / 4.0f;
	float RenderAspectRatio = 1.0f;
};