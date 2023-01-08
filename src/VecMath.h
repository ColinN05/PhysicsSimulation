#pragma once

#define CONST_PI 3.14159f

struct Mat4
{
	float entries[4][4];

	Mat4 Mult(Mat4 A, Mat4 B);

	Mat4 operator*(Mat4 B)
	{
		return Mult(*this,B);
	}
};

struct Mat3;

struct Vec3
{
	Vec3(float xp=0.0f, float yp=0.0f, float zp=0.0f);

	float x,y,z;

	Vec3 operator+(const Vec3 v)
	{
		Vec3 u;
		u.x = x + v.x;
		u.y = y + v.y;
		u.z = z + v.z;
		return u;
	}

	Vec3 operator-(Vec3 v)
	{
		return {x - v.x, y - v.y, z - v.z};
	}

	Vec3 operator*(const float f)
	{
		return {f * x, f * y, f * z};
	}

	void operator+=(Vec3 v)
	{
		x = x + v.x;
		y = y + v.y;
		z = z + v.z;
	}

	void operator-=(Vec3 v)
	{
		x = x - v.x;
		y = y - v.y;
		z = z - v.z;
	}

	Mat3 StarMatrix();

	float MaxComponent();
};

Vec3 CrossProduct(Vec3 v, Vec3 w);

float Length(Vec3 v);

Vec3 Normalize(Vec3 v);

float DotProduct(Vec3 v, Vec3 w);

float Project(Vec3 v, Vec3 dir);

Vec3 ProjectNormal(Vec3 v, Vec3 normal);

void Clamp(float& f, float min, float max);


struct Mat3
{
	float entries[3][3];

	Mat3 Mult(Mat3 A, Mat3 B);

	Mat3 operator*(Mat3 B)
	{
		return Mult(*this,B);
	}

	Vec3 operator*(Vec3 v)
	{
		Vec3 a0 = {entries[0][0], entries[0][1], entries[0][2]};
		Vec3 a1 = {entries[1][0], entries[1][1], entries[1][2]};
		Vec3 a2 = {entries[2][0], entries[2][1], entries[2][2]};

		Vec3 w = a0 * v.x + a1 * v.y + a2 * v.z;
		return w;
	}

	void operator+=(Mat3 A)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				entries[i][j] += A.entries[i][j];
			}
		}
	}

	Mat4 ToMat4();
};

Mat3 Inverse(Mat3 A);
Mat3 Transpose(Mat3 A);

Vec3 operator*(float f, Vec3 v);

struct Vec2
{
	float x,y;

	Vec2 operator+(const Vec2 v)
	{
		Vec2 u;
		u.x = x + v.x;
		u.y = y + v.y;
		return u;
	}

	Vec2 operator*(const float f)
	{
		return {f * x, f * y};
	}

	void operator+=(Vec2 v)
	{
		x = x + v.x;
		y = y + v.y;
	}

	void operator-=(Vec2 v)
	{
		x = x - v.x;
		y = y - v.y;
	}
};

Vec2 operator*(float f, Vec2 v);

struct Quat
{
	float w, x, y, z;

	Quat operator*(Quat s)
	{
		Quat ret;
		float& w1 = w; float& w2 = s.w;
		float& x1 = x; float& x2 = s.x;
		float& y1 = y; float& y2 = s.y;
		float& z1 = z; float& z2 = s.z;

		ret.w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
		ret.x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
		ret.y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
		ret.z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

		return ret;
	}

	void operator+=(Quat q)
	{
		w += q.w;
		x += q.x;
		y += q.y;
		z += q.z;
	}

	Quat operator*(float f)
	{
		return {w * f, x * f, y * f, z * f};
	}

	Mat3 ToRotMat3();
};

float QuatAbs(Quat q);