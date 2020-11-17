#ifndef _VECTOR_H
#define _VECTOR_H

#include <iostream>

// Need a Vector class for creating Quaternions

class Vector
{ // class Vector
public:

    // vector x, y, z components
    float x;
    float y;
    float z;

    // constructor
    Vector();
    Vector(float x, float y, float z);
    Vector(const Vector &other);

    // static method for cross product return a new vector object
    static Vector cross(const Vector &a, const Vector &b);

    // static method for dot product returns a scalar value
    static float dot(const Vector &a, const Vector &b);

    // addition operator
    Vector operator+(const Vector &other);

    // subtraction operator
    Vector operator-(const Vector &other);

    // multiply by a scalar
    Vector operator*(float scalar) const;

    // divide by a scalar
    Vector operator/(float scalar);

    // return magnitude of vector
    float magnitude();

    // returns the normalised vector
    Vector normalise();

}; // class Vector

// stream output
std::ostream& operator <<(std::ostream &outStream, Vector vector);

#endif