#include "PhysicsRenderer.h"
#include <GL/glew.h>
#include "Physics.h"
#include <cmath>
#include "stb_image.h"

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
	sphereVertices.reserve(UVSphereRes * UVSphereRes * 18);
	for (int i = 0; i < UVSphereRes; i++)
	{
		for (int j = 0; j < UVSphereRes; j++)
		{
			const float pi = CONST_PI;
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
	if (rb->GetType() == "RigidBox")
	{
		RenderRigidBox((RigidBox*)rb);
	}
	else if (rb->GetType() == "RigidSphere")
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

	unsigned int ProjLoc = glGetUniformLocation(PrimaryShader.GetID(), "proj");
	glUniformMatrix4fv(ProjLoc, 1, GL_FALSE, (const GLfloat*)CameraProjectionMatrix.entries);

	Mat4 modelScale = 
	{{
		{rb->Extent.x, 0.0f,0.0f,0.0f},
		{0.0f, rb->Extent.y, 0.0f, 0.0f},
		{0.0f, 0.0f, rb->Extent.z,0.0f},
		{0.0f,0.0f,0.0f,1.0f}
	}};
	Mat4 modelRot = rb->GetQuatRotation().ToRotMat3().ToMat4();

	const Vec3 rbPosition = rb->GetPosition();
	Mat4 modelTrans = 
	{{
			{1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f},
			{rbPosition.x, rbPosition.y, rbPosition.z, 1.0f}
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

	unsigned int ProjLoc = glGetUniformLocation(PrimaryShader.GetID(), "proj");
	glUniformMatrix4fv(ProjLoc, 1, GL_FALSE, (const GLfloat*)CameraProjectionMatrix.entries);

	const float rsRadius = rs->GetRadius();
	Mat4 modelScale = 
	{{
		{rsRadius, 0.0f,0.0f,0.0f},
		{0.0f, rsRadius, 0.0f, 0.0f},
		{0.0f, 0.0f, rsRadius,0.0f},
		{0.0f,0.0f,0.0f,1.0f}
	}};
	Mat4 modelRot = rs->GetQuatRotation().ToRotMat3().ToMat4();

	const Vec3 rsPosition = rs->GetPosition();
	Mat4 modelTrans = 
	{{
			{1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f},
			{rsPosition.x, rsPosition.y, rsPosition.z, 1.0f}
	}};

	Mat4 model = modelTrans * modelRot * modelScale;

	unsigned int modelLoc = glGetUniformLocation(PrimaryShader.GetID(), "model");
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (const GLfloat*)(model.entries));

	glDrawArrays(GL_TRIANGLES, 0, 6 * UVSphereRes * UVSphereRes);
}

void PhysicsRenderer::RenderWorld(PhysicsWorld* world)
{
	for (RigidBody* body : world->GetRigidBodies())
	{
		RenderRigidBody(body);
	}
}

void PhysicsRenderer::SetCameraPosition(Vec3 pos)
{
	CameraPosition = pos;
}

void PhysicsRenderer::SetCameraAngle(float theta, float phi)
{
	CameraTheta = theta; CameraPhi = phi;
}

void PhysicsRenderer::SetCameraFOV(float fov)
{
	CameraFOV = fov;
	RecalculateCameraProjectionMatrix();
}

void PhysicsRenderer::SetCameraNear(float cameraNear)
{
	CameraNear = cameraNear;
	RecalculateCameraProjectionMatrix();
}

void PhysicsRenderer::SetCameraFar(float cameraFar)
{
	CameraFar = cameraFar;
	RecalculateCameraProjectionMatrix();
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
	float RenderAspectRatio = 1.0f;
	if (RenderWindow) { RenderAspectRatio = RenderWindow->TargetAspectRatio; }
	CameraProjectionMatrix = 
	{{
		{1.0f / (RenderAspectRatio * tanf(CameraFOV / 2.0f)), 0.0f,0.0f,0.0f},
		{0.0f,1.0f/tanf(CameraFOV/2.0f), 0.0f, 0.0f},
		{0.0f,0.0f,-(CameraFar + CameraNear)/(CameraFar - CameraNear),-1.0f},
		{0.0f,0.0f,-2.0f * (CameraFar * CameraNear) / (CameraFar - CameraNear),0.0f}
	}};
}

void PhysicsRenderer::SetRenderWindow(Window* window)
{
	RenderWindow = window;
	RecalculateCameraProjectionMatrix();
}

void PhysicsRenderer::ClearBuffers()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}