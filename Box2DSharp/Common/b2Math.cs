/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Diagnostics;

/// <summary>
/// A 2D column vector.
/// </summary>
public struct b2Vec2
{
    public float x, y;

    public float this[int i]
    {
        get
        {
            if (i == 0) return x;
            if (i == 1) return y;
            Debug.Assert(false, "Incorrect Vec2 element!");
            return 0;
        }
        set
        {
            if (i == 0) x = value;
            else if (i == 1) y = value;
            else
            {
                Debug.Assert(false, "Incorrect Vec2 element!");
            }
        }
    }

    /// <summary>
    /// Construct using coordinates.
    /// </summary>
    public b2Vec2(float x)
    {
        this.x = x;
        this.y = x;
    }

    /// <summary>
    /// Construct using coordinates.
    /// </summary>
    public b2Vec2(float x, float y)
    {
        this.x = x;
        this.y = y;
    }

    public b2Vec2(b2Vec2 other)
    {
        this.x = other.x;
        this.y = other.y;
    }

    /// <summary>
    /// Set this vector to all zeros.
    /// </summary>
    public void SetZero() { x = 0.0f; y = 0.0f; }

    /// <summary>
    /// Set this vector to some specified coordinates.
    /// </summary>
    public void Set(float x, float y) { this.x = x; this.y = y; }

    public void Set(float xy) { x = xy; y = xy; }

    /// <summary>
    ///  Get the length of this vector (the norm).
    /// </summary>
    public float Length()
    {
        return (float)System.Math.Sqrt(x * x + y * y);
    }

    /// <summary>
    /// Get the length squared. For performance, use this instead of
    /// Length (if possible).
    /// </summary>
    public float LengthSquared()
    {
        return x * x + y * y;
    }

    /// <summary>
    /// Convert this vector into a unit vector. Returns the length.
    /// </summary>
    public float Normalize()
    {
        float length = Length();
        if (length < float.Epsilon)
        {
            return 0.0f;
        }
        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;

        return length;
    }

    /// <summary>
    /// Does this vector contain finite coordinates?
    /// </summary>
    public bool IsValid()
    {
        return GlobalMembers.b2IsValid(x) && GlobalMembers.b2IsValid(y);
    }

    /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	public b2Vec2 Skew()
	{
		return new b2Vec2(-y, x);
    }

    /// <summary>
    /// Negate this vector.
    /// </summary>
    public static b2Vec2 operator -(b2Vec2 v1)
    {
        b2Vec2 v = new b2Vec2();
        v.Set(-v1.x, -v1.y);
        return v;
    }

    public static b2Vec2 operator +(b2Vec2 v1, b2Vec2 v2)
    {
        b2Vec2 v = new b2Vec2();
        v.Set(v1.x + v2.x, v1.y + v2.y);
        return v;
    }

    public static b2Vec2 operator -(b2Vec2 v1, b2Vec2 v2)
    {
        b2Vec2 v = new b2Vec2();
        v.Set(v1.x - v2.x, v1.y - v2.y);
        return v;
    }

    public static b2Vec2 operator *(b2Vec2 v1, float a)
    {
        b2Vec2 v = new b2Vec2();
        v.Set(v1.x * a, v1.y * a);
        return v;
    }

    public static b2Vec2 operator *(float a, b2Vec2 v1)
    {
        b2Vec2 v = new b2Vec2();
        v.Set(v1.x * a, v1.y * a);
        return v;
    }

    public static bool operator ==(b2Vec2 a, b2Vec2 b)
    {
        return a.x == b.x && a.y == b.y;
    }

    public static bool operator !=(b2Vec2 a, b2Vec2 b)
    {
        return a.x != b.x || a.y != b.y;
    }

    public override bool Equals(object o)
    {
        return (b2Vec2)o == this;
    }

    public override int GetHashCode()
    {
        return x.GetHashCode() + y.GetHashCode();
    }

    public static b2Vec2 Zero { get { return new b2Vec2(0, 0); } }

    /// <summary>
    /// Peform the dot product on two vectors.
    /// </summary>
    public static float Dot(b2Vec2 a, b2Vec2 b)
    {
        return a.x * b.x + a.y * b.y;
    }

    /// <summary>
    /// Perform the cross product on two vectors. In 2D this produces a scalar.
    /// </summary>
    public static float Cross(b2Vec2 a, b2Vec2 b)
    {
        return a.x * b.y - a.y * b.x;
    }

    /// <summary>
    /// Perform the cross product on a vector and a scalar. 
    /// In 2D this produces a vector.
    /// </summary>
    public static b2Vec2 Cross(b2Vec2 a, float s)
    {
        b2Vec2 v = new b2Vec2();
        v.Set(s * a.y, -s * a.x);
        return v;
    }

    /// <summary>
    /// Perform the cross product on a scalar and a vector. 
    /// In 2D this produces a vector.
    /// </summary>
    public static b2Vec2 Cross(float s, b2Vec2 a)
    {
        b2Vec2 v = new b2Vec2();
        v.Set(-s * a.y, s * a.x);
        return v;
    }

    public static float Distance(b2Vec2 a, b2Vec2 b)
    {
        b2Vec2 c = a - b;
        return c.Length();
    }

    public static float DistanceSquared(b2Vec2 a, b2Vec2 b)
    {
        b2Vec2 c = a - b;
        return Dot(c, c);
    }
}

/// <summary>
/// A 2D column vector with 3 elements.
/// </summary>
public struct b2Vec3
{
    /// <summary>
    /// Construct using coordinates.
    /// </summary>
    public b2Vec3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }

    /// <summary>
    /// Set this vector to all zeros.
    /// </summary>
    public void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

    /// <summary>
    /// Set this vector to some specified coordinates.
    /// </summary>
    public void Set(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }

    /// <summary>
    /// Perform the dot product on two vectors.
    /// </summary>
    public static float Dot(b2Vec3 a, b2Vec3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// <summary>
    /// Perform the cross product on two vectors.
    /// </summary>
    public static b2Vec3 Cross(b2Vec3 a, b2Vec3 b)
    {
        return new b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    /// <summary>
    /// Negate this vector.
    /// </summary>
    public static b2Vec3 operator -(b2Vec3 v)
    {
        return new b2Vec3(-v.x, -v.y, -v.z);
    }

    /// <summary>
    /// Add two vectors component-wise.
    /// </summary>
    public static b2Vec3 operator +(b2Vec3 v1, b2Vec3 v2)
    {
        return new b2Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
    }

    /// <summary>
    /// Subtract two vectors component-wise.
    /// </summary>
    public static b2Vec3 operator -(b2Vec3 v1, b2Vec3 v2)
    {
        return new b2Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
    }

    /// <summary>
    /// Multiply this vector by a scalar.
    /// </summary>
    public static b2Vec3 operator *(b2Vec3 v, float s)
    {
        return new b2Vec3(v.x * s, v.y * s, v.z * s);
    }

    /// <summary>
    /// Multiply this vector by a scalar.
    /// </summary>
    public static b2Vec3 operator *(float s, b2Vec3 v)
    {
        return new b2Vec3(v.x * s, v.y * s, v.z * s);
    }

    public float x, y, z;
}

/// A 2-by-2 matrix. Stored in column-major order.
public class b2Mat22
{
    /// The default constructor does nothing (for performance).
    public b2Mat22()
    {
    }

    /// Construct this matrix using columns.
    public b2Mat22(b2Vec2 c1, b2Vec2 c2)
    {
        ex = c1;
        ey = c2;
    }

    /// Construct this matrix using scalars.
    public b2Mat22(float a11, float a12, float a21, float a22)
    {
        ex.x = a11;
        ex.y = a21;
        ey.x = a12;
        ey.y = a22;
    }

    /// Initialize this matrix using columns.
    public void Set(b2Vec2 c1, b2Vec2 c2)
    {
        ex = c1;
        ey = c2;
    }

    /// Set this to the identity matrix.
    public void SetIdentity()
    {
        ex.x = 1.0f;
        ey.x = 0.0f;
        ex.y = 0.0f;
        ey.y = 1.0f;
    }

    /// Set this matrix to all zeros.
    public void SetZero()
    {
        ex.x = 0.0f;
        ey.x = 0.0f;
        ex.y = 0.0f;
        ey.y = 0.0f;
    }

    public b2Mat22 GetInverse()
    {
        float a = ex.x;
        float b = ey.x;
        float c = ex.y;
        float d = ey.y;
        b2Mat22 B = new b2Mat22();
        float det = a * d - b * c;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }

        B.ex.x = det * d;
        B.ey.x = -det * b;
        B.ex.y = -det * c;
        B.ey.y = det * a;
        return B;
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    public b2Vec2 Solve(b2Vec2 b)
    {
        float a11 = ex.x;
        float a12 = ey.x;
        float a21 = ex.y;
        float a22 = ey.y;
        float det = a11 * a22 - a12 * a21;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }

        b2Vec2 x = new b2Vec2();
        x.x = det * (a22 * b.x - a12 * b.y);
        x.y = det * (a11 * b.y - a21 * b.x);
        return x;
    }

    public b2Vec2 ex = new b2Vec2();
    public b2Vec2 ey = new b2Vec2();
}

/// A 3-by-3 matrix. Stored in column-major order.
public class b2Mat33
{
    /// The default constructor does nothing (for performance).
    public b2Mat33()
    {
    }

    /// Construct this matrix using columns.
    public b2Mat33(b2Vec3 c1, b2Vec3 c2, b2Vec3 c3)
    {
        ex = c1;
        ey = c2;
        ez = c3;
    }

    /// Set this matrix to all zeros.
    public void SetZero()
    {
        ex.SetZero();
        ey.SetZero();
        ez.SetZero();
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    public b2Vec3 Solve33(b2Vec3 b)
    {
        float det = GlobalMembers.b2Dot(ex, GlobalMembers.b2Cross(ey, ez));
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }

        b2Vec3 x = new b2Vec3();
        x.x = det * GlobalMembers.b2Dot(b, GlobalMembers.b2Cross(ey, ez));
        x.y = det * GlobalMembers.b2Dot(ex, GlobalMembers.b2Cross(b, ez));
        x.z = det * GlobalMembers.b2Dot(ex, GlobalMembers.b2Cross(ey, b));
        return x;
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases. Solve only the upper
    /// 2-by-2 matrix equation.

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    public b2Vec2 Solve22(b2Vec2 b)
    {
        float a11 = ex.x;
        float a12 = ey.x;
        float a21 = ex.y;
        float a22 = ey.y;
        float det = a11 * a22 - a12 * a21;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }

        b2Vec2 x = new b2Vec2();
        x.x = det * (a22 * b.x - a12 * b.y);
        x.y = det * (a11 * b.y - a21 * b.x);
        return x;
    }

    /// Get the inverse of this matrix as a 2-by-2.
    /// Returns the zero matrix if singular.

    ///
    public void GetInverse22(b2Mat33 M)
    {
        float a = ex.x;
        float b = ey.x;
        float c = ex.y;
        float d = ey.y;
        float det = a * d - b * c;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }

        M.ex.x = det * d;
        M.ey.x = -det * b;
        M.ex.z = 0.0f;
        M.ex.y = -det * c;
        M.ey.y = det * a;
        M.ey.z = 0.0f;
        M.ez.x = 0.0f;
        M.ez.y = 0.0f;
        M.ez.z = 0.0f;
    }

    /// Get the symmetric inverse of this matrix as a 3-by-3.
    /// Returns the zero matrix if singular.

    /// Returns the zero matrix if singular.
    public void GetSymInverse33(b2Mat33 M)
    {
        float det = GlobalMembers.b2Dot(ex, GlobalMembers.b2Cross(ey, ez));
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }

        float a11 = ex.x;
        float a12 = ey.x;
        float a13 = ez.x;
        float a22 = ey.y;
        float a23 = ez.y;
        float a33 = ez.z;

        M.ex.x = det * (a22 * a33 - a23 * a23);
        M.ex.y = det * (a13 * a23 - a12 * a33);
        M.ex.z = det * (a12 * a23 - a13 * a22);

        M.ey.x = M.ex.y;
        M.ey.y = det * (a11 * a33 - a13 * a13);
        M.ey.z = det * (a13 * a12 - a11 * a23);

        M.ez.x = M.ex.z;
        M.ez.y = M.ey.z;
        M.ez.z = det * (a11 * a22 - a12 * a12);
    }

    public b2Vec3 ex = new b2Vec3();
    public b2Vec3 ey = new b2Vec3();
    public b2Vec3 ez = new b2Vec3();
}


/// Rotation
public struct b2Rot
{
    /// Initialize from an angle in radians
    public b2Rot(float angle)
    {
        /// TODO_ERIN optimize
        s = (float)Math.Sin(angle);
        c = (float)Math.Cos(angle);
    }

    /// Set using an angle in radians.
    public void Set(float angle)
    {
        /// TODO_ERIN optimize
        s = (float)Math.Sin(angle);
        c = (float)Math.Cos(angle);
    }

    /// Set to the identity rotation
    public void SetIdentity()
    {
        s = 0.0f;
        c = 1.0f;
    }

    /// Get the angle in radians
    public float GetAngle()
    {
        return (float)Math.Atan2(s, c);
    }

    /// Get the x-axis
    public b2Vec2 GetXAxis()
    {
        return new b2Vec2(c, s);
    }

    /// Get the u-axis
    public b2Vec2 GetYAxis()
    {
        return new b2Vec2(-s, c);
    }

    /// Sine and cosine
    public float s;

    public float c;
}


/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
public struct b2Transform
{
    public b2Transform(b2Transform other)
    {
        p = other.p;
        q = other.q;
    }

    /// Initialize using a position vector and a rotation.
    public b2Transform(b2Vec2 position, b2Rot rotation)
    {
        this.p = position;
        this.q = rotation;
    }

    /// Set this to the identity transform.
    public void SetIdentity()
    {
        p.SetZero();
        q.SetIdentity();
    }

    /// Set this based on the position and angle.
    public void Set(b2Vec2 position, float angle)
    {
        p = position;
        q.Set(angle);
    }

    public b2Vec2 p;
    public b2Rot q;
}


/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
public struct b2Sweep
{
    /// Get the interpolated transform at a specific time.
    /// @param beta is a factor in [0,1], where 0 indicates alpha0.
    public void GetTransform(b2Transform xf, float beta)
    {
        xf.p = (1.0f - beta) * c0 + beta * c;
        float angle = (1.0f - beta) * a0 + beta * a;
        xf.q.Set(angle);

        // Shift to origin
        xf.p -= GlobalMembers.b2Mul(xf.q, localCenter);
    }

    /// Advance the sweep forward, yielding a new initial state.
    /// @param alpha the new initial time.
    public void Advance(float alpha)
    {
        Debug.Assert(alpha0 < 1.0f);
        float beta = (alpha - alpha0) / (1.0f - alpha0);
        c0 += beta * (c - c0);
        a0 += beta * (a - a0);
        alpha0 = alpha;
    }

    /// Normalize the angles.

    /// Normalize an angle in radians to be between -pi and pi
    public void Normalize()
    {
        float twoPi = 2.0f * DefineConstants.b2_pi;
        float d = twoPi * (float)Math.Floor(a0 / twoPi);
        a0 -= d;
        a -= d;
    }

    public b2Vec2 localCenter;

    ///< local center of mass position
    public b2Vec2 c0;

    ///< center world positions
    public b2Vec2 c;

    public float a0;

    ///< world angles
    public float a;

    /// Fraction of the current time step in the range [0,1]
    /// c0 and a0 are the positions at alpha0.
    public float alpha0;
}

