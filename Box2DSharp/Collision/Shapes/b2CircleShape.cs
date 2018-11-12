/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

using System;

/// A circle shape.
public class b2CircleShape : b2Shape
{
	public b2CircleShape()
	{
		m_type = Type.e_circle;
		m_radius = 0.0f;
		m_p.SetZero();
	}

	/// Implement b2Shape.
	public override b2Shape Clone()
	{
		b2CircleShape clone = new b2CircleShape();
		clone.m_p = m_p;
	    clone.m_radius = m_radius;
		return clone;
	}

	/// @see b2Shape::GetChildCount
	public override int GetChildCount()
	{
		return 1;
	}

	/// Implement b2Shape.
	public override bool TestPoint(b2Transform transform, b2Vec2 p)
	{
		b2Vec2 center = transform.p + GlobalMembers.b2Mul(transform.q, m_p);
		b2Vec2 d = p - center;
		return GlobalMembers.b2Dot(d, d) <= m_radius * m_radius;
	}

	/// Implement b2Shape.

	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.1.2
	// x = s + a * r
	// norm(x) = radius
	public override bool RayCast(b2RayCastOutput output, b2RayCastInput input, b2Transform transform, int childIndex)
	{
		b2Vec2 position = transform.p + GlobalMembers.b2Mul(transform.q, m_p);
		b2Vec2 s = input.p1 - position;
		float b = GlobalMembers.b2Dot(s, s) - m_radius * m_radius;

		// Solve quadratic equation.
		b2Vec2 r = input.p2 - input.p1;
		float c = GlobalMembers.b2Dot(s, r);
		float rr = GlobalMembers.b2Dot(r, r);
		float sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0f || rr < float.Epsilon)
		{
			return false;
		}

		// Find the point of intersection of the line with the circle.
		float a = -(c + (float)Math.Sqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0f <= a && a <= input.maxFraction * rr)
		{
			a /= rr;
			output.fraction = a;
			output.normal = s + a * r;
			output.normal.Normalize();
			return true;
		}

		return false;
	}

	/// @see b2Shape::ComputeAABB
	public override void ComputeAABB(b2AABB aabb, b2Transform transform, int childIndex)
	{
		b2Vec2 p = transform.p + GlobalMembers.b2Mul(transform.q, m_p);
		aabb.lowerBound.Set(p.x - m_radius, p.y - m_radius);
		aabb.upperBound.Set(p.x + m_radius, p.y + m_radius);
	}

	/// @see b2Shape::ComputeMass
	public override void ComputeMass(b2MassData massData, float density)
	{
		massData.mass = density * DefineConstants.b2_pi * m_radius * m_radius;
		massData.center = m_p;

		// inertia about the local origin
		massData.I = massData.mass * (0.5f * m_radius * m_radius + GlobalMembers.b2Dot(m_p, m_p));
	}

	/// Position
	public b2Vec2 m_p = new b2Vec2();
}
