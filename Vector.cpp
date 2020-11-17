#include "Vector.h"

#include <math.h>

// Constructors
// default to unit vector
Vector::Vector() : x(1.0), y(0.0), z(0.0) 
{}

Vector::Vector(float x, float y, float z) : x(x), y(y), z(z)
{}

Vector::Vector(const Vector &other) : x(other.x), y(other.y), z(other.z)
{}

// cross product
Vector Vector::cross(const Vector &a, const Vector &b)
{
    // return the cross product of a and b
    return Vector(a.y * b.z - a.z * b.y, 
                    a.z * b.x - a.x * b.z, 
                    a.x * b.y - a.y * b.x);
}

// dot product
float Vector::dot(const Vector &a, const Vector &b)
{
    // return the dot product of a and b
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// vector addition
Vector Vector::operator+(const Vector &other)
{
    return Vector(x + other.x, y + other.y, z + other.z);
}

// vector subtraction
Vector Vector::operator-(const Vector &other)
{
    return Vector(x - other.x, y - other.y, z - other.z);
}

// scalar multiplication
Vector Vector::operator*(float scalar) const
{
    return Vector(x * scalar, y * scalar, z * scalar);
}

// scalar division
Vector Vector::operator/(float scalar)
{
    return Vector(x / scalar, y / scalar, z / scalar);
}

// return the magnitude of the vector
float Vector::magnitude()
{
    return sqrt(x*x + y*y + z*z);
}

// return a normalised vector
Vector Vector::normalise()
{
    float magnitude = this->magnitude();
    return Vector(*this / magnitude);
}

// output vector to a stream
std::ostream& operator<<(std::ostream &outStream, Vector vector)
{
    outStream << "(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
    return outStream;
}