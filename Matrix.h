#pragma once
#include <math.h>

//Class 1: Vector
class Vector { //Represents a 3D vector in space

  public: float v[4]; 
  
  Vector(float x = 0, float y = 0, float z = 0, float w = 1) { //Constructor
    v[0] = x; 
    v[1] = y;
    v[2] = z;
    v[3] = w; //w is used for homogeneous coordinates, which are used in graphics transformations. It is also used as a quaternion component(deals with rotations)
  }

  Vector operator*(float s) const { //Operator for scalar multiplication. Scales each component of the vector by the scalar value. It creates a new vector.
    return Vector(v[0] * s, v[1] * s, v[2] * s, 1);
  }
  
  Vector operator+(const Vector &v2) const { //Operator for vector addition. Adds the corresponding components of two vectors.
    return Vector(v[0] + v2.v[0], v[1] + v2.v[1], v[2] + v2.v[2], 1); 
  }
  
  Vector operator-(const Vector &v2) const { //Operator for vector subtraction. Subtracts the corresponding components of two vectors.
    return Vector(v[0] - v2.v[0], v[1] - v2.v[1], v[2] - v2.v[2], 1); 
  }
    
  Vector &operator*=(float s) { //Operator for scalar multiplication, which modifies the original vector in place.
    *this = *this * s;
    return *this;
  }

  float &operator[](int i) { //Indexing operator to access individual vector components.
    return v[i];
  }

  float length() { //Calculates the vector's magnitude or length.
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  void normalize() { //Normalizes the vector to have a length of 1.
    float l2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if(!l2) return;
    float rl = 1 / sqrt(l2);
    *this *= rl;
  }

  float dot(const Vector &v2) const { // Calculates the dot product of two vectors (which is scalar product)
    return v[0] * v2.v[0] + v[1] * v2.v[1] + v[2] * v2.v[2];
  }
  
  Vector cross(const Vector &v2) const { // Calculates the cross product of two vectors. (which is vector product)
    return Vector(v[1] * v2.v[2] - v[2] * v2.v[1], v[2] * v2.v[0] - v[0] * v2.v[2], v[0] * v2.v[1] - v[1] * v2.v[0]);
  }

  Vector project(const Vector &v2) const { //Projects the current vector onto the given vector.
    Vector nv2 = v2;
    nv2.normalize();
    float s = dot(nv2);
    return nv2 * s;
  }
};

//Class 2: Matrix
class Matrix {
  public: float m[4][4]; //Array that stores matrix elements 
  
  Matrix() : Matrix(1, 0, 0, 0,  0, 1, 0 ,0,  0, 0, 1, 0, 0, 0, 0, 1) {} //Constructor which initializes matrix to identity matrix

  Matrix(float m00, float m01, float m02, float m03,
         float m10, float m11, float m12, float m13,
          float m20, float m21, float m22, float m23,
          float m30, float m31, float m32, float m33) { //Another constructor which uses the 4x4 matrix format

    m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
    m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
    m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
    m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
  }
  
  static Matrix identity() { //Creates an identity matrix
    return Matrix(1, 0, 0, 0,  0, 1, 0 ,0,  0, 0, 1, 0, 0, 0, 0, 1);
  }
  
  static Matrix scaling(float s) { //Creates a scaling matrix that scales all coordinates by the given factor.
    return Matrix(s, 0, 0, 0,  0, s, 0 ,0,  0, 0, s, 0, 0, 0, 0, 1);
  }
  
  static Matrix scaling(float u, float v, float w) { //Creates a non-uniform scaling matrix that scales the x, y, and z coordinates by different factors.
    return Matrix(u, 0, 0, 0,  0, v, 0 ,0,  0, 0, w, 0, 0, 0, 0, 1);
  }
  
  static Matrix translation(float x, float y, float z) { //Creates a translation matrix that translates the coordinates by the given x, y, and z values.
    return Matrix(1, 0, 0, x,  0, 1, 0, y,  0, 0, 1, z, 0, 0, 0, 1);
  }
    
  static Matrix rotation(float a, float x, float y, float z) { //Creates a rotation matrix that rotates the coordinates around a given axis by the specified angle.

    float cosa = cos(a);
    float rcosa = 1 - cosa;
    float sina = sin(a);

    return Matrix(
      x * x * rcosa + cosa,     x * y * rcosa - z * sina, x * z * rcosa + y * sina, 0,
      y * x * rcosa + z * sina, y * y * rcosa + cosa,     y * z * rcosa - x * sina, 0,
      z * x * rcosa - y * sina, z * y * rcosa + x * sina, z * z * rcosa + cosa, 0,
      0, 0, 0, 1
      );
  }

  static Matrix perspective(float fov, float near, float far) { //Creates a perspective projection matrix that projects a 3D scene onto a 2D plane.
    float scale = tan(fov * 0.5 * M_PI / 180);

    return Matrix(
      scale, 0, 0, 0,
      0, scale, 0, 0,
      0, 0, -far * near / (far - near), 0,
      0, 0, -1, 0
      );    
  }

  //Operator Overloading Methods

  Vector operator *(const Vector &v) { //This method multiplies a matrix by a vector. The result is a new vector that is transformed by the matrix.
    return Vector(
      v.v[0] * m[0][0] + v.v[1] * m[0][1] + v.v[2] * m[0][2] + m[0][3],
      v.v[0] * m[1][0] + v.v[1] * m[1][1] + v.v[2] * m[1][2] + m[1][3],
      v.v[0] * m[2][0] + v.v[1] * m[2][1] + v.v[2] * m[2][2] + m[2][3],
      v.v[0] * m[3][0] + v.v[1] * m[3][1] + v.v[2] * m[3][2] + m[3][3]
      );
  }

  Matrix operator *(const Matrix &m2) { //This method multiplies two matrices. The result is a new matrix that is the product of the two original matrices.
    Matrix mr;
    for(int y = 0; y < 4; y++)
        for(int x = 0; x < 4; x++)
          mr.m[y][x] = m[y][0] * m2.m[0][x] + m[y][1] * m2.m[1][x] + m[y][2] * m2.m[2][x] + m[y][3] * m2.m[3][x];   
    return mr;
  }

  Matrix &operator *=(const Matrix &m2) { //Method which modifies current matrix in place.
    *this = *this * m2;
    return *this;
  }
};

