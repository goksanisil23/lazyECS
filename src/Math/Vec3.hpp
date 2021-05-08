#pragma once


class Vec3
{
public:
	Vec3()
		: x(0.0f), y(0.0f), z(0.0f)
	{}

	Vec3(float x, float y, float z)
		: x(x), y(y), z(z)
	{}

	Vec3 operator+(Vec3 const& rhs) const
	{
		return Vec3(
			this->x + rhs.x,
			this->y + rhs.y,
			this->z + rhs.z);
	}

	Vec3 operator+=(Vec3 const& rhs)
	{
		this->x += rhs.x;
		this->y += rhs.y;
		this->z += rhs.z;

		return *this;
	}

	Vec3 operator-(Vec3 const& rhs) const
	{
		return Vec3(
			this->x - rhs.x,
			this->y - rhs.y,
			this->z - rhs.z);
	}

	Vec3 operator-=(Vec3 const& rhs)
	{
		this->x -= rhs.x;
		this->y -= rhs.y;
		this->z -= rhs.z;

		return *this;
	}

	Vec3 operator*(Vec3 const& rhs) const
	{
		return Vec3(
			this->x * rhs.x,
			this->y * rhs.y,
			this->z * rhs.z);
	}

	Vec3 operator*=(Vec3 const& rhs)
	{
		this->x *= rhs.x;
		this->y *= rhs.y;
		this->z *= rhs.z;

		return *this;
	}

	Vec3 operator*(float rhs) const
	{
		return Vec3(this->x * rhs, 
					this->y * rhs, 
					this->z * rhs
					);
	}

	Vec3 operator*=(float rhs)
	{
		this->x *= rhs;
		this->y *= rhs;
		this->z *= rhs;

		return *this;
	}


	float x, y, z;
};
