#pragma once
#include "VecMath.h"
#include <vector>
#include <string>

struct PhysicsMaterial
{
	float Restitution = 1.0f, StaticFriction = 0.0f, DynamicFriction = 0.0f;
};

class RigidBody
{
public:
	void Update(float deltaTime);

	virtual void SetMass(float mass);
	const inline float GetMass() const { return Mass; }

	inline Mat3 GetIbody() { return Ibody; }
	inline Mat3 GetIbodyinv() { return Ibodyinv; }
	inline Mat3 GetIinv() { return Iinv; }

	void SetPosition(Vec3 position);
	inline Vec3 GetPosition() { return Position; }
	inline Quat GetQuatRotation() { return QuatRotation; }

	virtual void RecalculateBodyInertiaTensor() = 0;

	void ApplyImpulse(Vec3 impulse);
	void ApplyImpulseAtPoint(Vec3 impulse, Vec3 point);
	void ApplyForce(Vec3 force);
	void ApplyForceAtPoint(Vec3 force, Vec3 point);

	Vec3 Velocity();
	Vec3 Omega();

	inline bool GetSimulateGravity() { return bSimulateGravity; }

	inline std::string GetType() { return Type; }

	PhysicsMaterial Material;
protected:
	std::string Type;

	float Mass = 1.0f;
	Mat3 Ibody, Ibodyinv;

	Vec3 Position = {0.0f,0.0f,0.0f};
	Quat QuatRotation = {1.0f,0.0f,0.0f,0.0f};

	Vec3 LinearMomentum = {0.0f,0.0f,0.0f}, AngularMomentum = {0.0f,0.0f,0.0f};

	Mat3 Iinv;
	Mat3 Rotation;

	Vec3 Force, Torque;

	bool bSimulateGravity = true;
};

struct RigidBox : public  RigidBody
{
	RigidBox();

	virtual void RecalculateBodyInertiaTensor() override;
	void SetExtent(Vec3 extent);

	Vec3 Extent;
};

struct RigidSphere : public RigidBody
{
public:
	RigidSphere();

	virtual void RecalculateBodyInertiaTensor() override;
	void SetRadius(float radius);
	inline float GetRadius() {return Radius;}
private:
	float Radius = 1.0f;
};

struct CollisionPair
{
	RigidBody* Body1,* Body2;
};

class PhysicsWorld
{
public:
	PhysicsWorld();
	~PhysicsWorld();

	void Step(float deltaTime);
	RigidSphere* SpawnRigidSphere(Vec3 position = {0.0f,0.0f,0.0f});
	RigidBox* SpawnRigidBox(Vec3 position = {0.0f,0.0f,0.0f});
	void SetGravity(float gravity);

	std::vector<CollisionPair> CollisionBroadPhase();
	void CollisionNarrowPhaseAndResolve(std::vector<CollisionPair>& potentialCollisions);

	inline const std::vector<RigidBody*>& GetRigidBodies() const { return RigidBodies; }

private:
	std::vector<RigidBody*> RigidBodies;
	float Gravity = 9.81f;
};