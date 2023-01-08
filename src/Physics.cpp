#include "Physics.h"
#include <cmath>
#include <iostream>
#include <GL/glew.h>
#include "stb_image.h"
#include <windows.h>
#include <algorithm>

void RigidBody::Update(float deltaTime)
{
	// Linear motion
	LinearMomentum += Force * deltaTime;
	Position += Velocity() * deltaTime;

	// Rotational motion
	AngularMomentum += Torque * deltaTime;
	Rotation = QuatRotation.ToRotMat3();
	Iinv = (Rotation * Ibodyinv) * Transpose(Rotation);
	Quat omegaQuat = {0.0f, Omega().x, Omega().y, Omega().z};
	QuatRotation += (omegaQuat * QuatRotation) * 0.5f * deltaTime; 
	QuatRotation = QuatRotation * (1.0f/(0.0000001f + QuatAbs(QuatRotation)));

	// reset force / torque
	Force = {0.0f,0.0f,0.0f};
	Torque = {0.0f,0.0f,0.0f};
}

void RigidBody::SetMass(float mass)
{
	Mass = mass;
	RecalculateBodyInertiaTensor();
}

void RigidBody::SetPosition(Vec3 position)
{
	Position = position;
}

Vec3 RigidBody::Velocity()
{
	return LinearMomentum * (1.0f/ Mass);
}

Vec3 RigidBody::Omega()
{
	return Iinv * AngularMomentum;
}

RigidBox::RigidBox()
{
	Type = "RigidBox";

	Rotation = 
	{
		{
			{1.0f,0.0f,0.0f},
			{0.0f, 1.0f, 0.0f},
			{0.0f, 0.0f, 1.0f}
		}
	};

	RecalculateBodyInertiaTensor();
}

void RigidBox::RecalculateBodyInertiaTensor()
{
	float w = Extent.x * 2.0f;
	float h = Extent.y * 2.0f;
	float d = Extent.z * 2.0f;
	Ibody = 
	{{
		{ Mass/12.0f * (h * h + d * d), 0.0f, 0.0f},
		{0.0f, Mass/12.0f * (w * w + d * d),0.0f},
		{0.0f, 0.0f, Mass/12.0f * (w * w + h * h)}
	}};
	
	Ibodyinv = Inverse(Ibody);
}

void RigidBox::SetExtent(Vec3 extent)
{	
	Extent = extent;
	RecalculateBodyInertiaTensor();
}

void RigidSphere::RecalculateBodyInertiaTensor()
{
	Ibody = 
	{{
		{2.0f/3.0f * Mass * Radius * Radius, 0.0f,0.0f},
		{0.0f, 2.0f/3.0f * Mass * Radius * Radius, 0.0f},
		{0.0f, 0.0f, 2.0f/3.0f * Mass * Radius * Radius}
	}};

	Ibodyinv = Inverse(Ibody);
}

RigidSphere::RigidSphere()
{
	Type = "RigidSphere";

	Rotation = 
	{
		{
			{1.0f,0.0f,0.0f},
			{0.0f, 1.0f, 0.0f},
			{0.0f, 0.0f, 1.0f}
		}
	};

	RecalculateBodyInertiaTensor();
}

void RigidSphere::SetRadius(float radius)
{
	Radius = radius;

	RecalculateBodyInertiaTensor();
}

void RigidBody::ApplyImpulse(Vec3 impulse)
{
	LinearMomentum += impulse;
}

void RigidBody::ApplyImpulseAtPoint(Vec3 impulse, Vec3 point)
{
	LinearMomentum += impulse;
	AngularMomentum += CrossProduct(point - Position, impulse);
}

void RigidBody::ApplyForce(Vec3 force)
{
	Force += force;
}

void RigidBody::ApplyForceAtPoint(Vec3 force, Vec3 point)
{
	Force += force;
	Torque += CrossProduct(point - Position, force);
}

PhysicsWorld::PhysicsWorld()
{

}

PhysicsWorld::~PhysicsWorld()
{
	for (RigidBody* body : RigidBodies)
	{
		delete body;
	}
}

void PhysicsWorld::Step(float deltaTime)
{
	// gravity
	for (RigidBody* rb : RigidBodies)
	{
		if (rb->GetSimulateGravity())
		{
			rb->ApplyForce(Vec3(0.0f, - Gravity * rb->GetMass(), 0.0f));
		}
	}

	std::vector<CollisionPair> potentialCollisions = CollisionBroadPhase();
	CollisionNarrowPhaseAndResolve(potentialCollisions);

	// Update rigid body motion
	for (RigidBody* rb : RigidBodies)
	{
		rb->Update(deltaTime);
	}
}

RigidSphere* PhysicsWorld::SpawnRigidSphere(Vec3 position)
{
	RigidSphere* sphere = new RigidSphere;
	sphere->SetPosition(position);
	RigidBodies.push_back((RigidBody*)sphere);
	return sphere;
}

RigidBox* PhysicsWorld::SpawnRigidBox(Vec3 position)
{
	RigidBox* box = new RigidBox;
	box->SetPosition(position);
	RigidBodies.push_back((RigidBody*)box);
	return box;
}

void PhysicsWorld::SetGravity(float gravity)
{
	Gravity = gravity;
}

std::vector<CollisionPair> PhysicsWorld::CollisionBroadPhase()
{
	std::vector<CollisionPair> potentialCollisions;
	for (int i = 0; i < RigidBodies.size(); i++)
	{
		for (int j = 0; j < i; j++)
		{
			// for now
			RigidBody* body1 = RigidBodies[i];
			RigidBody* body2 = RigidBodies[j];
			Vec3 AABB1Center = body1->GetPosition();
			Vec3 AABB2Center = body2->GetPosition();

			Vec3 AABB1Extent;
			Vec3 AABB2Extent;
			const std::string body1Type = body1->GetType();
			const std::string body2Type = body2->GetType();
			
			if (body1Type == "RigidSphere")
			{
				AABB1Extent = ((RigidSphere*)body1)->GetRadius() * Vec3(1.0f,1.0f,1.0f);
			}
			else if (body1Type == "RigidBox")
			{
				RigidBox* box = (RigidBox*)body1;
				float diagonalLength = Length(2.0f * box->Extent);
				AABB1Extent = diagonalLength * Vec3(1.0f,1.0f,1.0f);
			}

			if (body2Type == "RigidSphere")
			{
				AABB2Extent = ((RigidSphere*)body2)->GetRadius() * Vec3(1.0f,1.0f,1.0f);
			}
			else if (body2Type == "RigidBox")
			{
				RigidBox* box = (RigidBox*)body2;
				float diagonalLength = Length(box->Extent);
				AABB2Extent = diagonalLength * Vec3(1.0f,1.0f,1.0f);
			}

			//if (AABB)

			CollisionPair potentialPair = {body1, body2};
			potentialCollisions.push_back(potentialPair);
		}
	}
	return potentialCollisions;
}

void PhysicsWorld::CollisionNarrowPhaseAndResolve(std::vector<CollisionPair>& potentialCollisions)
{
	for (CollisionPair collisionPair : potentialCollisions)
	{
		RigidBody* rb1 = collisionPair.Body1; 
		const std::string rb1Type = rb1->GetType();
		RigidBody* rb2 = collisionPair.Body2;
		const std::string rb2Type = rb2->GetType();

		// Sphere-sphere collision
		if (rb1Type == "RigidSphere" && rb2Type == "RigidSphere")
		{
			RigidSphere* sphere1 = (RigidSphere*)rb1;
			RigidSphere* sphere2 = (RigidSphere*)rb2;
			const Vec3 R12 = sphere2->GetPosition() - sphere1->GetPosition();
			if (Length(R12) < sphere1->GetRadius() + sphere2->GetRadius())
			{
				Vec3 N = Normalize(R12); // collision normal vector
				Vec3 collisionPoint = N * sphere1->GetRadius() + sphere1->GetPosition();

				float m1 = sphere1->GetMass();
				float m2 = sphere2->GetMass(); 
				float e = std::min(sphere1->Material.Restitution, sphere2->Material.Restitution);
				Vec3 r1 = collisionPoint - sphere1->GetPosition();
				Vec3 r2 = collisionPoint - sphere2->GetPosition();
				Vec3 vp1 = sphere1->Velocity() + CrossProduct(sphere1->Omega() , r1);
				Vec3 vp2 = sphere2->Velocity() + CrossProduct(sphere2->Omega() , r2);
				Vec3 vr = vp2 - vp1;
				float j = -(1.0f + e) * DotProduct(vr, N) / (1.0f/m1 + 1.0f/m2 + DotProduct(CrossProduct(sphere1->GetIinv() * CrossProduct(r1,N),r1)+CrossProduct(sphere2->GetIinv() * CrossProduct(r2,N),r2),N));  
				Vec3 J = -1.0f * j * N;

				rb1->ApplyImpulseAtPoint(J, collisionPoint);
				rb2->ApplyImpulseAtPoint(-1.0f * J, collisionPoint);

				sphere2->SetPosition(sphere2->GetPosition() + N * (sphere1->GetRadius() + sphere2->GetRadius() - Length(R12)));
			}
		}
		// Box-sphere collision
		if ((rb1Type == "RigidBox" && rb2Type == "RigidSphere"))
		{
			RigidSphere* sphere = (RigidSphere*)rb2;
			RigidBox* box = (RigidBox*)rb1;

			Mat3 boxRotation = box->GetQuatRotation().ToRotMat3();
			Vec3 closestPointBoxCoords = Inverse(boxRotation) * (sphere->GetPosition() - box->GetPosition());
			Clamp(closestPointBoxCoords.x,-box->Extent.x,box->Extent.x);
			Clamp(closestPointBoxCoords.y,-box->Extent.y,box->Extent.y);
			Clamp(closestPointBoxCoords.z,-box->Extent.z,box->Extent.z);
			Vec3 closestPoint = boxRotation * closestPointBoxCoords + box->GetPosition();

			if (Length(closestPoint - sphere->GetPosition()) < sphere->GetRadius())
			{
				Vec3 N = Normalize(closestPoint - sphere->GetPosition());

				float m1 = sphere->GetMass();
				float v1 = Project(sphere->Velocity(), N); 
				float m2 = box->GetMass(); 
				float v2 = Project(box->Velocity(), N);

				sphere->SetPosition( -sphere->GetRadius() * N + closestPoint);

				Vec3& collisionPoint = closestPoint;

				Vec3 r1 = collisionPoint - box->GetPosition();
				Vec3 r2 = collisionPoint - sphere->GetPosition();

				float e = std::min(sphere->Material.Restitution, box->Material.Restitution);

				Vec3 vp1 = box->Velocity() + CrossProduct(box->Omega() , r1);
				Vec3 vp2 = sphere->Velocity() + CrossProduct(sphere->Omega() , r2);
				Vec3 vr = vp2 - vp1;
				float j = -(1.0f + e) * DotProduct(vr, N) / (1.0f/m1 + 1.0f/m2 + DotProduct(CrossProduct(box->GetIinv() * CrossProduct(r1,N),r1)+CrossProduct(sphere->GetIinv() * CrossProduct(r2,N),r2),N));  
				Vec3 J1 = -1.0f * j * N;

				box->ApplyImpulseAtPoint(J1, closestPoint);
				sphere->ApplyImpulseAtPoint(-1.0f * J1, closestPoint);

			}
		}
		// Box-box collision
		if (rb1Type == "RigidBox" && rb2Type == "RigidBox")
		{
			RigidBox* box1 = (RigidBox*)rb1;
			RigidBox* box2 = (RigidBox*)rb2;
			//////////////////////////

			Mat3 model1Scale = 
			{{
				{box1->Extent.x, 0.0f,0.0f},
				{0.0f, box1->Extent.y, 0.0f},
				{0.0f, 0.0f, box1->Extent.z}
			}};
			Mat3 model1Rot = box1->GetQuatRotation().ToRotMat3();
			Mat3 model1 = model1Rot * model1Scale;

			Mat3 model2Scale = 
			{{
				{box2->Extent.x, 0.0f,0.0f},
				{0.0f, box2->Extent.y, 0.0f},
				{0.0f, 0.0f, box2->Extent.z}
			}};
			Mat3 model2Rot = box2->GetQuatRotation().ToRotMat3();
			Mat3 model2 = model2Rot * model2Scale;

			Mat3 normalTransform1 = Transpose(Inverse(model1));
			Mat3 normalTransform2 = Transpose(Inverse(model2));
			std::vector<Vec3> axes = 
			{
				Normalize(normalTransform1 * Vec3(1.0f,0.0f,0.0f)),
				Normalize(normalTransform1 * Vec3(0.0f,1.0f,0.0f)),
				Normalize(normalTransform1 * Vec3(0.0f,0.0f,1.0f)),
				Normalize(normalTransform2 * Vec3(1.0f,0.0f,0.0f)),
				Normalize(normalTransform2 * Vec3(0.0f,1.0f,0.0f)),
				Normalize(normalTransform2 * Vec3(0.0f,0.0f,1.0f))
			};

			std::vector<Vec3> cubeVertices;
			std::vector<float> pm = {-1.0f,1.0f};
			for (float x : pm) {for (float y : pm) {for (float z : pm) {cubeVertices.push_back(Vec3(x,y,z));}}}

			bool colliding = true;

			Vec3 collisionPoint;

			for (int axisIndex = 0; axisIndex < axes.size(); axisIndex++)
			{
				Vec3 axis = axes[axisIndex];
				std::vector<float> proj1;
				float proj1Min, proj1Max;
				std::vector<float> proj2;
				float proj2Min, proj2Max;

				for (Vec3 untransformedVertex : cubeVertices)
				{
					proj1.push_back(Project(model1 * untransformedVertex + box1->GetPosition(), axis));
					proj2.push_back(Project(model2 * untransformedVertex + box2->GetPosition(), axis));
				}
				proj1Min = *std::min_element(proj1.begin(), proj1.end());
				proj1Max = *std::max_element(proj1.begin(), proj1.end());
				proj2Min = *std::min_element(proj2.begin(), proj2.end());
				proj2Max = *std::max_element(proj2.begin(), proj2.end());

				if (proj1Min > proj2Max || proj1Max < proj2Min)
				{
					colliding = false; // seperating axis found
				}

				if (axisIndex < 3)
				{
					if (proj1Max > proj2Min && proj1Min < proj2Max)
						collisionPoint += (proj2Min + proj1Max) / 2.0f * axis;
					else if (proj1Min < proj2Max && proj2Min < proj2Max)
						collisionPoint += (proj1Min + proj2Max) / 2.0f * axis;
				}
			}

			if (colliding)
			{
				std::cout << "Box-box collision detected\n";
				//////////// need to do collision response . .. . 
			}
		}
	}
}