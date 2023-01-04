#include "Physics.h"
#include <cmath>
#include <iostream>
#include <GL/glew.h>
#include "stb_image.h"
#include <windows.h>
#include <algorithm>

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
	float w = ExtentX * 2.0f;
	float h = ExtentY * 2.0f;
	float d = ExtentZ * 2.0f;
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
	ExtentX = extent.x;
	ExtentY = extent.y;
	ExtentZ = extent.z;
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

void ApplyImpulse(RigidBody* rb, Vec3 impulse)
{
	rb->LinearMomentum += impulse;
}

void ApplyImpulseAtPoint(RigidBody* rb, Vec3 impulse, Vec3 point)
{
	rb->LinearMomentum += impulse;

	rb->AngularMomentum += CrossProduct(point - rb->Position, impulse);
}

void ApplyForce(RigidBody* rb, Vec3 f)
{
	rb->Force += f;
}

void ApplyForceAtPoint(RigidBody* rb, Vec3 f, Vec3 p)
{
	rb->Force += f;
	rb->Torque += CrossProduct((p-rb->Position), f);
}

void PhysicsWorld::Step(float deltaTime)
{
	// gravity
	for (RigidBody* rb : RigidBodies)
	{
		if (rb->bSimulateGravity)
		{
			ApplyForce(rb, Vec3(0.0f, - Gravity * rb->Mass, 0.0f));
		}
	}

	std::vector<CollisionPair> potentialCollisions = CollisionBroadPhase();
	CollisionNarrowPhaseAndResolve(potentialCollisions);

	// Update rigid body motion
	for (RigidBody* rb : RigidBodies)
	{
		// Linear motion
		rb->LinearMomentum += rb->Force * deltaTime;
		rb->Position += rb->Velocity() * deltaTime;

		// Rotational motion
		rb->AngularMomentum += rb->Torque * deltaTime;
		rb->Rotation = rb->QuatRotation.ToRotMat3();
		rb->Iinv = (rb->Rotation * rb->Ibodyinv) * Transpose(rb->Rotation);
		Quat omegaQuat = {0.0f, rb->Omega().x, rb->Omega().y, rb->Omega().z};
		rb->QuatRotation += (omegaQuat * rb->QuatRotation) * 0.5f * deltaTime; 
		rb->QuatRotation = rb->QuatRotation * (1.0f/(0.0000001f + QuatAbs(rb->QuatRotation)));
	}

	// Reset forces/torques on rigid bodies
	for (RigidBody* rb : RigidBodies)
	{
		rb->Force = {0.0f,0.0f,0.0f};
		rb->Torque = {0.0f,0.0f,0.0f};
	}
}

void PhysicsWorld::SpawnRigidBody(RigidBody* rb,Vec3 position)
{
	RigidBodies.push_back(rb);
	rb->SetPosition(position);
	rb->RecalculateBodyInertiaTensor();
}

std::vector<CollisionPair> PhysicsWorld::CollisionBroadPhase()
{
	std::vector<CollisionPair> potentialCollisions;
	for (int i = 0; i < RigidBodies.size(); i++)
	{
		for (int j = 0; j < i; j++)
		{
			// for now
			CollisionPair potentialPair = {RigidBodies[i], RigidBodies[j]};
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
		RigidBody* rb2 = collisionPair.Body2;

		// Sphere-sphere collision
		if (rb1->Type == "RigidSphere" && rb2->Type == "RigidSphere")
		{
			RigidSphere* sphere1 = (RigidSphere*)rb1;
			RigidSphere* sphere2 = (RigidSphere*)rb2;
			Vec3 R12 = sphere2->Position - sphere1->Position;
			if (Length(R12) < sphere1->Radius + sphere2->Radius)
			{
				Vec3 N = Normalize(R12);
				Vec3 collisionPoint = N * sphere1->Radius + sphere1->Position;
					
				float m1 = sphere1->Mass;
				float m2 = sphere2->Mass; 
				float e = sphere1->Material.Restitution * sphere2->Material.Restitution;
				Vec3 r1 = collisionPoint - sphere1->GetPosition();
				Vec3 r2 = collisionPoint - sphere2->GetPosition();

				Vec3 vp1 = sphere1->Velocity() + CrossProduct(sphere1->Omega() , r1);
				Vec3 vp2 = sphere2->Velocity() + CrossProduct(sphere2->Omega() , r2);
				Vec3 vr = vp2 - vp1;
				float j = -(1.0f + e) * DotProduct(vr, N) / (1.0f/m1 + 1.0f/m2 + DotProduct(CrossProduct(sphere1->Iinv * CrossProduct(r1,N),r1)+CrossProduct(sphere2->Iinv * CrossProduct(r2,N),r2),N));  
				Vec3 J = -1.0f * j * N;

				ApplyImpulseAtPoint(rb1,  J, collisionPoint);
				ApplyImpulseAtPoint(rb2, -1.0f * J, collisionPoint);

				sphere2->SetPosition(sphere2->GetPosition() + N * (sphere1->Radius + sphere2->Radius - Length(R12)));
			}
		}
		// Box-sphere collision
		if ((rb1->Type == "RigidBox" && rb2->Type == "RigidSphere"))
		{
			RigidSphere* sphere = (RigidSphere*)rb2;
			RigidBox* box = (RigidBox*)rb1;

			Mat3 boxRotation = box->QuatRotation.ToRotMat3();
			Vec3 closestPointBoxCoords = Inverse(boxRotation) * (sphere->GetPosition() - box->GetPosition());
			Clamp(closestPointBoxCoords.x,-box->ExtentX,box->ExtentX);
			Clamp(closestPointBoxCoords.y,-box->ExtentY,box->ExtentY);
			Clamp(closestPointBoxCoords.z,-box->ExtentZ,box->ExtentZ);
			Vec3 closestPoint = boxRotation * closestPointBoxCoords + box->GetPosition();

			if (Length(closestPoint - sphere->GetPosition()) < sphere->Radius)
			{
				Vec3 sphereNormal = Normalize(closestPoint - sphere->GetPosition());
				Vec3& N = sphereNormal;

				float m1 = sphere->Mass;
				float v1 = Project(sphere->Velocity(), sphereNormal); 
				float m2 = box->Mass; 
				float v2 = Project(box->Velocity(), sphereNormal);

				sphere->SetPosition( - sphere->Radius * sphereNormal + closestPoint);

				Vec3& collisionPoint = closestPoint;

				Vec3 r1 = collisionPoint - box->GetPosition();
				Vec3 r2 = collisionPoint - sphere->GetPosition();

				float e = 0.7f;

				Vec3 vp1 = box->Velocity() + CrossProduct(box->Omega() , r1);
				Vec3 vp2 = sphere->Velocity() + CrossProduct(sphere->Omega() , r2);
				Vec3 vr = vp2 - vp1;
				float j = -(1.0f + e) * DotProduct(vr, N) / (1.0f/m1 + 1.0f/m2 + DotProduct(CrossProduct(box->Iinv * CrossProduct(r1,N),r1)+CrossProduct(sphere->Iinv * CrossProduct(r2,N),r2),N));  
				Vec3 J1 = -1.0f * j * N;

				ApplyImpulseAtPoint(box, J1, closestPoint);
				ApplyImpulseAtPoint(sphere, -1.0f * J1, closestPoint);

			}
		}
		// Box-box collision
		if (rb1->Type == "RigidBox" && rb2->Type == "RigidBox")
		{
			RigidBox* box1 = (RigidBox*)rb1;
			RigidBox* box2 = (RigidBox*)rb2;
			//////////////////////////

			Mat3 model1Scale = 
			{{
				{box1->ExtentX, 0.0f,0.0f},
				{0.0f, box1->ExtentY, 0.0f},
				{0.0f, 0.0f, box1->ExtentZ}
			}};
			Mat3 model1Rot = box1->QuatRotation.ToRotMat3();
			Mat3 model1 = model1Rot * model1Scale;

			Mat3 model2Scale = 
			{{
				{box2->ExtentX, 0.0f,0.0f},
				{0.0f, box2->ExtentY, 0.0f},
				{0.0f, 0.0f, box2->ExtentZ}
			}};
			Mat3 model2Rot = box2->QuatRotation.ToRotMat3();
			Mat3 model2 = model2Rot * model2Scale;

			Mat3 normalTransform1 = Transpose(Inverse(model1));
			Mat3 normalTransform2 = Transpose(Inverse(model2));
			std::vector<Vec3> axes = 
			{
				normalTransform1 * Vec3(1.0f,0.0f,0.0f),
				normalTransform1 * Vec3(0.0f,1.0f,0.0f),
				normalTransform1 * Vec3(0.0f,0.0f,1.0f),
				normalTransform2 * Vec3(1.0f,0.0f,0.0f),
				normalTransform2 * Vec3(0.0f,1.0f,0.0f),
				normalTransform2 * Vec3(0.0f,0.0f,1.0f)
			};

			std::vector<Vec3> cubeVertices;
			std::vector<float> pm = {-1.0f,1.0f};
			for (float x : pm) {for (float y : pm) {for (float z : pm) {cubeVertices.push_back(Vec3(x,y,z));}}}

			bool colliding = true;

			for (Vec3 axis : axes)
			{
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
			}

			if (colliding)
			{
				std::cout << "Box-box collision detected\n";
				// need to resolve collision
			}
		}
	}
}

PhysicsRenderer::PhysicsRenderer()
	: PrimaryShader("src/VertexShader.glsl", "src/FragmentShader.glsl")
{
}

void PhysicsRenderer::InitializeGLEW()
{
	auto glewInitResult = glewInit();
	if (glewInitResult != GLEW_OK)
	{
		std::cout << "GLEW failed to initialize!" << glewGetErrorString(glewInitResult) << '\n';
	}

	glClearColor(0.1,0.1,0.1,1);
	glEnable(GL_DEPTH_TEST);
}

void PhysicsRenderer::Init()
{
	// Generate cube vertex array
	float cubeVertices[] = 
	{
		-1.0f,-1.0f,1.0f,  0.0f, 0.0f,   0.0f, 0.0f, 1.0f,
		1.0f,-1.0f,1.0f,  1.0f, 0.0f,   0.0f, 0.0f, 1.0f,
		1.0f,1.0f,1.0f,  1.0f, 1.0f,   0.0f, 0.0f, 1.0f,
		-1.0f,1.0f,1.0f,  0.0f, 1.0f,   0.0f, 0.0f, 1.0f,

		1.0f,-1.0f,1.0f,  0.0f, 0.0f,   1.0f, 0.0f, 0.0f,
		1.0f,-1.0f,-1.0f,  1.0f, 0.0f,  1.0f, 0.0f, 0.0f,
		1.0f,1.0f,-1.0f,  1.0f, 1.0f,  1.0f, 0.0f, 0.0f,
		1.0f,1.0f,1.0f,  0.0f, 1.0f,   1.0f, 0.0f, 0.0f,

		1.0f, 1.0f, 1.0f,   0.0f, 0.0f,   0.0f, 1.0f, 0.0f,
		1.0f, 1.0f, -1.0f,  1.0f, 0.0f,   0.0f, 1.0f, 0.0f,
		-1.0f, 1.0f, -1.0f,  1.0f, 1.0f,   0.0f, 1.0f, 0.0f,
		-1.0f, 1.0f, 1.0f,   0.0f, 1.0f,   0.0f, 1.0f, 0.0f,

		-1.0f,-1.0f,-1.0f,  0.0f, 0.0f,   0.0f, 0.0f, -1.0f,
		1.0f,-1.0f,-1.0f,  1.0f, 0.0f,   0.0f, 0.0f, -1.0f,
		1.0f,1.0f,-1.0f,  1.0f, 1.0f,   0.0f, 0.0f, -1.0f,
		-1.0f,1.0f,-1.0f,  0.0f, 1.0f,   0.0f, 0.0f, -1.0f,

		-1.0f,-1.0f,1.0f,  0.0f, 0.0f,   -1.0f, 0.0f, 0.0f,
		-1.0f,-1.0f,-1.0f,  1.0f, 0.0f,  -1.0f, 0.0f, 0.0f,
		-1.0f,1.0f,-1.0f,  1.0f, 1.0f,  -1.0f, 0.0f, 0.0f,
		-1.0f,1.0f,1.0f,  0.0f, 1.0f,   -1.0f, 0.0f, 0.0f,

		1.0f, -1.0f, 1.0f,   0.0f, 0.0f,   0.0f, -1.0f, 0.0f,
		1.0f, -1.0f, -1.0f,  1.0f, 0.0f,   0.0f, -1.0f, 0.0f,
		-1.0f, -1.0f, -1.0f,  1.0f, 1.0f,   0.0f, -1.0f, 0.0f,
		-1.0f, -1.0f, 1.0f,   0.0f, 1.0f,   0.0f, -1.0f, 0.0f
	};

	unsigned int cubeIndices[] = 
	{
		0,1,2,
		2,3,0,

		4,5,6,
		6,7,4,

		8,9,10,
		10,11,8,

		12, 13, 14,
		14, 15, 12,

		16, 17, 18, 
		18, 19, 16,

		20, 21, 22,
		22, 23, 20
	};

	glGenVertexArrays(1, &CubeVertexArrayID);
	glBindVertexArray(CubeVertexArrayID);

	unsigned int CubeVertexBufferID;
	glGenBuffers(1, &CubeVertexBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, CubeVertexBufferID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

	unsigned CubeIndexBufferID;
	glGenBuffers(1, &CubeIndexBufferID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, CubeIndexBufferID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(0 * sizeof(float)));
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(5 * sizeof(float)));
	glEnableVertexAttribArray(2);

	glBindVertexArray(0);

	// Generate sphere vertex array
	std::vector<float> sphereVertices = {};
	for (int i = 0; i < UVSphereRes; i++)
	{
		for (int j = 0; j < UVSphereRes; j++)
		{
			float pi = 3.1415926f;
			float t1 = (float)i * 2.0f * pi / (float)UVSphereRes;
			float p1 = (float)j * pi / (float)UVSphereRes - pi/2.0f;
			float t2 = ((float)i+1.0f) * 2.0f * pi / (float)UVSphereRes;
			float p2 = ((float)j+1.0f) * pi / (float)UVSphereRes - pi/2.0f;

			float P1[3] = {cosf(p1)*cosf(t1), sinf(p1), cosf(p1)*sinf(t1)};
			float P2[3] = {cosf(p1)*cosf(t2), sinf(p1), cosf(p1)*sinf(t2)};
			float P3[3] = {cosf(p2)*cosf(t2), sinf(p2), cosf(p2)*sinf(t2)};
			float P4[3] = {cosf(p2)*cosf(t1), sinf(p2), cosf(p2)*sinf(t1)};

			float normal[3] = {P1[0],P1[1],P1[2]};

			sphereVertices.push_back(P1[0]); sphereVertices.push_back(P1[1]); sphereVertices.push_back(P1[2]);
			sphereVertices.push_back(t1/(2.0f * pi)); sphereVertices.push_back((p1+pi/2.0f)/pi);
			sphereVertices.push_back(normal[0]); sphereVertices.push_back(normal[1]); sphereVertices.push_back(normal[2]); 

			sphereVertices.push_back(P2[0]); sphereVertices.push_back(P2[1]); sphereVertices.push_back(P2[2]);
			sphereVertices.push_back(t2/(2.0f * pi)); sphereVertices.push_back((p1+pi/2.0f)/pi);
			sphereVertices.push_back(normal[0]); sphereVertices.push_back(normal[1]); sphereVertices.push_back(normal[2]);

			sphereVertices.push_back(P3[0]); sphereVertices.push_back(P3[1]); sphereVertices.push_back(P3[2]);
			sphereVertices.push_back(t2/(2.0f * pi)); sphereVertices.push_back((p2+pi/2.0f)/pi);
			sphereVertices.push_back(normal[0]); sphereVertices.push_back(normal[1]); sphereVertices.push_back(normal[2]);  

			sphereVertices.push_back(P1[0]); sphereVertices.push_back(P1[1]); sphereVertices.push_back(P1[2]);
			sphereVertices.push_back(t1/(2.0f * pi)); sphereVertices.push_back((p1+pi/2.0f)/pi);
			sphereVertices.push_back(normal[0]); sphereVertices.push_back(normal[1]); sphereVertices.push_back(normal[2]); 

			sphereVertices.push_back(P3[0]); sphereVertices.push_back(P3[1]); sphereVertices.push_back(P3[2]);
			sphereVertices.push_back(t2/(2.0f * pi)); sphereVertices.push_back((p2+pi/2.0f)/pi);
			sphereVertices.push_back(normal[0]); sphereVertices.push_back(normal[1]); sphereVertices.push_back(normal[2]); 

			sphereVertices.push_back(P4[0]); sphereVertices.push_back(P4[1]); sphereVertices.push_back(P4[2]);
			sphereVertices.push_back(t1/(2.0f * pi)); sphereVertices.push_back((p2+pi/2.0f)/pi);
			sphereVertices.push_back(normal[0]); sphereVertices.push_back(normal[1]); sphereVertices.push_back(normal[2]); 
		}
	}

	glGenVertexArrays(1, &SphereVertexArrayID);
	glBindVertexArray(SphereVertexArrayID);

	unsigned int SphereVertexBufferID;
	glGenBuffers(1, &SphereVertexBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, SphereVertexBufferID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * sphereVertices.size(), &sphereVertices[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(0 * sizeof(float)));
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(5 * sizeof(float)));
	glEnableVertexAttribArray(2);

	glBindVertexArray(0);

	// load textures
	glGenTextures(1, &TextureID);
	glBindTexture(GL_TEXTURE_2D, TextureID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	int width, height, nrChannels;

	unsigned char* data = stbi_load("assets/checker.jpg", &width, &height, &nrChannels, 0);

	if (data)
	{
	    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	    glGenerateMipmap(GL_TEXTURE_2D);
	}
	else
	{
	    std::cout << "Failed to load texture" << std::endl;
	}
	stbi_image_free(data);

	RecalculateCameraViewMatrix();
	RecalculateCameraProjectionMatrix();

}

void PhysicsRenderer::DetectCameraInput(float deltaTime)
{
	Vec3 forward = { sinf(CameraTheta), 0.0f, -cosf(CameraPhi) * cosf(CameraTheta) };
	Vec3 right = { cosf(CameraTheta), 0.0f, sinf(CameraTheta) };
	Vec3 up = { 0.0f, 1.0f, 0.0f };

	// Update camera position
	if (GetAsyncKeyState(0x57)) CameraPosition += 20.0f * forward * deltaTime;
	if (GetAsyncKeyState(0x41)) CameraPosition -= 20.0f * right * deltaTime;
	if (GetAsyncKeyState(0x53)) CameraPosition -= 20.0f * forward * deltaTime;
	if (GetAsyncKeyState(0x44)) CameraPosition += 20.0f * right * deltaTime;
	if (GetAsyncKeyState(VK_RIGHT)) CameraTheta += 2.0f * deltaTime;
	if (GetAsyncKeyState(VK_LEFT)) CameraTheta -= 2.0f * deltaTime;
	if (GetAsyncKeyState(VK_UP)) CameraPhi += 2.0f * deltaTime;
	if (GetAsyncKeyState(VK_DOWN)) CameraPhi -= 2.0f * deltaTime;
	if (GetAsyncKeyState(VK_SPACE)) CameraPosition += 20.0f * deltaTime * up;
	if (GetAsyncKeyState(VK_LSHIFT)) CameraPosition -= 20.0f * up * deltaTime;

	if (CameraPhi > CONST_PI / 2.0f) CameraPhi = CONST_PI/2.0f;
	if (CameraPhi < - CONST_PI / 2.0f) CameraPhi = - CONST_PI / 2.0f;
	if (CameraTheta > 2.0f * CONST_PI) CameraTheta -= 2.0f * CONST_PI;
	if (CameraTheta < -2.0f * CONST_PI) CameraTheta += 2.0f * CONST_PI;
}

void PhysicsRenderer::RenderRigidBody(RigidBody* rb)
{
	if (rb->Type == "RigidBox")
	{
		RenderRigidBox((RigidBox*)rb);
	}
	else if (rb->Type == "RigidSphere")
	{
		RenderRigidSphere((RigidSphere*)rb);
	}


}

void PhysicsRenderer::RenderRigidBox(RigidBox* rb)
{
	glBindVertexArray(CubeVertexArrayID);
	PrimaryShader.Use();
	glBindTexture(GL_TEXTURE_2D, TextureID);

	RecalculateCameraViewMatrix();
	unsigned int viewLoc = glGetUniformLocation(PrimaryShader.GetID(), "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, (const GLfloat*)CameraViewMatrix.entries);

	RecalculateCameraProjectionMatrix();
	unsigned int ProjLoc = glGetUniformLocation(PrimaryShader.GetID(), "proj");
	glUniformMatrix4fv(ProjLoc, 1, GL_FALSE, (const GLfloat*)CameraProjectionMatrix.entries);

	Mat4 modelScale = 
	{{
		{rb->ExtentX, 0.0f,0.0f,0.0f},
		{0.0f, rb->ExtentY, 0.0f, 0.0f},
		{0.0f, 0.0f, rb->ExtentZ,0.0f},
		{0.0f,0.0f,0.0f,1.0f}
	}};
	Mat4 modelRot = rb->QuatRotation.ToRotMat3().ToMat4();

	Mat4 modelTrans = 
	{{
			{1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f},
			{rb->Position.x, rb->Position.y, rb->Position.z, 1.0f}
	}};


	Mat4 model = modelTrans * modelRot * modelScale;

	unsigned int modelLoc = glGetUniformLocation(PrimaryShader.GetID(), "model");
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (const GLfloat*)(model.entries));

	glDrawElements(GL_TRIANGLES, 192 / 4, GL_UNSIGNED_INT, 0);
}

void PhysicsRenderer::RenderRigidSphere(RigidSphere* rs)
{
	glBindVertexArray(SphereVertexArrayID);
	PrimaryShader.Use();
	glBindTexture(GL_TEXTURE_2D, TextureID);

	RecalculateCameraViewMatrix();
	unsigned int viewLoc = glGetUniformLocation(PrimaryShader.GetID(), "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, (const GLfloat*)CameraViewMatrix.entries);

	RecalculateCameraProjectionMatrix();
	unsigned int ProjLoc = glGetUniformLocation(PrimaryShader.GetID(), "proj");
	glUniformMatrix4fv(ProjLoc, 1, GL_FALSE, (const GLfloat*)CameraProjectionMatrix.entries);

	Mat4 modelScale = 
	{{
		{rs->Radius, 0.0f,0.0f,0.0f},
		{0.0f, rs->Radius, 0.0f, 0.0f},
		{0.0f, 0.0f, rs->Radius,0.0f},
		{0.0f,0.0f,0.0f,1.0f}
	}};
	Mat4 modelRot = rs->QuatRotation.ToRotMat3().ToMat4();

	Mat4 modelTrans = 
	{{
			{1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f},
			{rs->Position.x, rs->Position.y, rs->Position.z, 1.0f}
	}};


	Mat4 model = modelTrans * modelRot * modelScale;

	unsigned int modelLoc = glGetUniformLocation(PrimaryShader.GetID(), "model");
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (const GLfloat*)(model.entries));

	glDrawArrays(GL_TRIANGLES, 0, 6 * UVSphereRes * UVSphereRes);
}

void PhysicsRenderer::SetCameraPosition(Vec3 pos)
{
	CameraPosition = pos;
}

void PhysicsRenderer::RecalculateCameraViewMatrix()
{
	// Camera
	CameraViewMatrix =
	{{
		{cosf(CameraTheta), -sinf(CameraPhi) * sinf(CameraTheta),-cosf(CameraPhi) * sinf(CameraTheta), 0.0f},
		{0.0f, cosf(CameraPhi), -sinf(CameraPhi), 0.0f},
		{sinf(CameraTheta), sinf(CameraPhi) * cosf(CameraTheta), cosf(CameraPhi) * cosf(CameraTheta), 0.0f},
		{-CameraPosition.x * cosf(CameraTheta) - CameraPosition.z * sinf(CameraTheta),
		 CameraPosition.x * sinf(CameraPhi) * sinf(CameraTheta) -CameraPosition.y * cosf(CameraPhi) - CameraPosition.z * sinf(CameraPhi) * cosf(CameraTheta),
		 CameraPosition.x * cosf(CameraPhi) * sinf(CameraTheta) + CameraPosition.y * sinf(CameraPhi) - CameraPosition.z * cosf(CameraPhi) * cosf(CameraTheta),1.0f}
	}};
}

void PhysicsRenderer::RecalculateCameraProjectionMatrix()
{
	CameraProjectionMatrix = 
	{{
		{1.0f / (RenderAspectRatio * tanf(CameraFOV / 2.0f)), 0.0f,0.0f,0.0f},
		{0.0f,1.0f/tanf(CameraFOV/2.0f), 0.0f, 0.0f},
		{0.0f,0.0f,-(CameraFar + CameraNear)/(CameraFar - CameraNear),-1.0f},
		{0.0f,0.0f,-2.0f * (CameraFar * CameraNear) / (CameraFar - CameraNear),0.0f}
	}};
}

void PhysicsRenderer::ClearBuffers()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}