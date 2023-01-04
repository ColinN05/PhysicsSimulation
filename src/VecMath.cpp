#include "VecMath.h"
#include <cmath>

Mat4 Mat4::Mult(Mat4 A, Mat4 B)
{
	Mat4 C = {};

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			C.entries[i][j] = A.entries[0][j] * B.entries[i][0] + A.entries[1][j] * B.entries[i][1] + A.entries[2][j] * B.entries[i][2] + A.entries[3][j] * B.entries[i][3];
		}
	}
	return C;
}

Mat4 TransformMat(Vec3 pos, Vec3 rot)
{
	Mat4 m = 
	{
		{
			{cosf(rot.y), -sinf(rot.x) * sinf(rot.y),cosf(rot.x) * sinf(rot.y), 0.0f},
			{0.0f, cosf(rot.x), sinf(rot.x), 0.0f},
			{-sinf(rot.y), -sinf(rot.x) * cosf(rot.y), cosf(rot.x) * cosf(rot.y), 0.0f},
			{pos.x, pos.y, pos.z, 1.0f}
		}
	};
	return m;
}

Mat4 PerspectiveProjectionMat(float aspect, float fov, float neardist, float fardist)
{
	Mat4 m = 
	{
		{
			{1.0f / (aspect * tanf(fov / 2.0f)), 0.0f,0.0f,0.0f},
			{0.0f,1.0f/tanf(fov/2.0f), 0.0f, 0.0f},
			{0.0f,0.0f,-(fardist + neardist)/(fardist - neardist),-1.0f},
			{0.0f,0.0f,-2.0f * (fardist * neardist) / (fardist - neardist),0.0f}
		}
	};
	return m;
}

Mat3 Mat3::Mult(Mat3 A, Mat3 B)
{
	Mat3 C = {};

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			C.entries[i][j] = A.entries[0][j] * B.entries[i][0] + A.entries[1][j] * B.entries[i][1] + A.entries[2][j] * B.entries[i][2];
		}
	}
	return C;
}


Vec3::Vec3(float xp, float yp, float zp)
	: x(xp), y(yp), z(zp)
{
}

Mat4 Mat3::ToMat4()
{
	Mat4 A;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			A.entries[i][j] = this->entries[i][j];
		}
	}


	A.entries[3][0] = 0.0f;
	A.entries[3][1] = 0.0f;
	A.entries[3][2] = 0.0f;
	A.entries[3][3] = 1.0f;

	A.entries[0][3] = 0.0f;
	A.entries[1][3] = 0.0f;
	A.entries[2][3] = 0.0f;

	return A;
}

Vec3 operator*(float f, Vec3 v)
{
	return {f * v.x, f * v.y, f * v.z};
}

Mat3 Vec3::StarMatrix()
{
	Mat3 A = 
	{
		{
			{0.0f, z, -y},
			{-z, 0.0f, x},
			{y,-x, 0.0f}
		}
	};

	return A;
}

Vec3 CrossProduct(Vec3 v, Vec3 w)
{
	Vec3 ret = 
	{
		v.y * w.z - w.y * v.z,
		- v.x * w.z + w.x * v.z,
		v.x * w.y - w.x * v.y
	};

	return ret;
}

float Length(Vec3 v)
{
	return sqrtf(0.00001f+v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 Normalize(Vec3 v)
{
	return v * (1.0f/Length(v));
}

float DotProduct(Vec3 v, Vec3 w)
{
	return v.x * w.x + v.y * w.y + v.z * w.z;
}

float Project(Vec3 v, Vec3 dir)
{
	return DotProduct(v, Normalize(dir));
}

Vec3 ProjectNormal(Vec3 v, Vec3 normal)
{
	return v - Project(v, normal) * normal;
}

void Clamp(float& f, float min, float max)
{
	if (f > max) f = max;
	if (f < min) f = min;
}

Mat3 Inverse(Mat3 A)
{
	// computes the inverse of a matrix m
	float det = A.entries[0][0] * (A.entries[1][1] * A.entries[2][2] - A.entries[2][1] * A.entries[1][2]) -
             A.entries[0][1] * (A.entries[1][0] * A.entries[2][2] - A.entries[1][2] * A.entries[2][0]) +
             A.entries[0][2] * (A.entries[1][0] * A.entries[2][1] - A.entries[1][1] * A.entries[2][0]);

	float invdet = 1.0f / det;

	Mat3 inv;
	inv.entries[0][0] = (A.entries[1][1] * A.entries[2][2] - A.entries[2][1] * A.entries[1][2]) * invdet;
	inv.entries[0][1] = (A.entries[0][2] * A.entries[2][1] - A.entries[0][1] * A.entries[2][2]) * invdet;
	inv.entries[0][2] = (A.entries[0][1] * A.entries[1][2] - A.entries[0][2] * A.entries[1][1]) * invdet;
	inv.entries[1][0] = (A.entries[1][2] * A.entries[2][0] - A.entries[1][0] * A.entries[2][2]) * invdet;
	inv.entries[1][1] = (A.entries[0][0] * A.entries[2][2] - A.entries[0][2] * A.entries[2][0]) * invdet;
	inv.entries[1][2] = (A.entries[1][0] * A.entries[0][2] - A.entries[0][0] * A.entries[1][2]) * invdet;
	inv.entries[2][0] = (A.entries[1][0] * A.entries[2][1] - A.entries[2][0] * A.entries[1][1]) * invdet;
	inv.entries[2][1] = (A.entries[2][0] * A.entries[0][1] - A.entries[0][0] * A.entries[2][1]) * invdet;
	inv.entries[2][2] = (A.entries[0][0] * A.entries[1][1] - A.entries[1][0] * A.entries[0][1]) * invdet;

	return inv;
}

Mat3 Transpose(Mat3 A)
{
	Mat3 B = {};
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			B.entries[i][j] = A.entries[j][i];
		}
	}
	return B;
}

Vec2 operator*(float f, Vec2 v)
{
	return {f * v.x, f * v.y};
}

Mat3 Quat::ToRotMat3()
{
	Mat3 A = 
	{{
		{1.0f-2.0f*y*y-2.0f*z*z, 2.0f*x*y+2.0f*w*z, 2.0f*x*z-2.0f*w*y},
		{2.0f*x*y-2.0f*w*z, 1.0f-2.0f*x*x-2.0f*z*z,2.0f*y*z+2.0f*w*x},
		{2.0f*x*z+2.0f*w*y,2.0f*y*z-2.0f*w*x,1.0f-2.0f*x*x-2.0f*y*y}
	}};
	return A;
}

float QuatAbs(Quat q)
{
	return sqrtf(q.w * q.w +q.x * q.x + q.y * q.y + q.z * q.z);
}