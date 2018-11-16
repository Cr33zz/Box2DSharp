/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
public class b2EdgeShape : b2Shape
{
	public b2EdgeShape()
	{
		m_type = Type.e_edge;
		m_radius = (2.0f * Settings.b2_linearSlop);
		m_vertex0.x = 0.0f;
		m_vertex0.y = 0.0f;
		m_vertex3.x = 0.0f;
		m_vertex3.y = 0.0f;
		m_hasVertex0 = false;
		m_hasVertex3 = false;
	}

	/// Set this as an isolated edge.
	public b2EdgeShape Set(b2Vec2 v1, b2Vec2 v2)
	{
		m_vertex1 = v1;
		m_vertex2 = v2;
		m_hasVertex0 = false;
		m_hasVertex3 = false;
        return this;
    }

    /// Implement b2Shape.
    public override b2Shape Clone()
	{
		b2EdgeShape clone = new b2EdgeShape();
		clone.m_vertex1 = m_vertex1;
	    clone.m_vertex2 = m_vertex2;
	    clone.m_vertex3 = m_vertex3;
	    clone.m_vertex0 = m_vertex0;
	    clone.m_hasVertex0 = m_hasVertex0;
	    clone.m_hasVertex3 = m_hasVertex3;
        return clone;
	}

	/// @see b2Shape::GetChildCount
	public override int GetChildCount()
	{
		return 1;
	}

	/// @see b2Shape::TestPoint
	public override bool TestPoint(b2Transform xf, b2Vec2 p)
	{
		return false;
	}

	/// Implement b2Shape.

	// p = p1 + t * d
	// v = v1 + s * e
	// p1 + t * d = v1 + s * e
	// s * e - t * d = p1 - v1
	public override bool RayCast(b2RayCastOutput output, b2RayCastInput input, b2Transform xf, int childIndex)
	{
		// Put the ray into the edge's frame of reference.
		b2Vec2 p1 = Utils.b2MulT(xf.q, input.p1 - xf.p);
		b2Vec2 p2 = Utils.b2MulT(xf.q, input.p2 - xf.p);
		b2Vec2 d = p2 - p1;

		b2Vec2 v1 = new b2Vec2(m_vertex1);
		b2Vec2 v2 = new b2Vec2(m_vertex2);
		b2Vec2 e = v2 - v1;
		b2Vec2 normal = new b2Vec2(e.y, -e.x);
		normal.Normalize();

		// q = p1 + t * d
		// dot(normal, q - v1) = 0
		// dot(normal, p1 - v1) + t * dot(normal, d) = 0
		float numerator = Utils.b2Dot(normal, v1 - p1);
		float denominator = Utils.b2Dot(normal, d);

		if (denominator == 0.0f)
		{
			return false;
		}

		float t = numerator / denominator;
		if (t < 0.0f || input.maxFraction < t)
		{
			return false;
		}

		b2Vec2 q = p1 + t * d;

		// q = v1 + s * r
		// s = dot(q - v1, r) / dot(r, r)
		b2Vec2 r = v2 - v1;
		float rr = Utils.b2Dot(r, r);
		if (rr == 0.0f)
		{
			return false;
		}

		float s = Utils.b2Dot(q - v1, r) / rr;
		if (s < 0.0f || 1.0f < s)
		{
			return false;
		}

		output.fraction = t;
		if (numerator > 0.0f)
		{
			output.normal = -Utils.b2Mul(xf.q, normal);
		}
		else
		{
			output.normal = Utils.b2Mul(xf.q, normal);
		}
		return true;
	}

	/// @see b2Shape::ComputeAABB
	public override void ComputeAABB(ref b2AABB aabb, b2Transform xf, int childIndex)
	{
		b2Vec2 v1 = Utils.b2Mul(xf, m_vertex1);
		b2Vec2 v2 = Utils.b2Mul(xf, m_vertex2);

		b2Vec2 lower = Utils.b2Min(v1, v2);
		b2Vec2 upper = Utils.b2Max(v1, v2);

		b2Vec2 r = new b2Vec2(m_radius, m_radius);
		aabb.lowerBound = lower - r;
		aabb.upperBound = upper + r;
	}

	/// @see b2Shape::ComputeMass
	public override void ComputeMass(ref b2MassData massData, float density)
	{
		massData.mass = 0.0f;
		massData.center = 0.5f * (m_vertex1 + m_vertex2);
		massData.I = 0.0f;
	}

    public override b2Vec2[] GetVertices()
    {
        return new[] { m_vertex1, m_vertex2 };
    }

    /// These are the edge vertices
    public b2Vec2 m_vertex1 = new b2Vec2();
	public b2Vec2 m_vertex2 = new b2Vec2();

	/// Optional adjacent vertices. These are used for smooth collision.
	public b2Vec2 m_vertex0 = new b2Vec2();
	public b2Vec2 m_vertex3 = new b2Vec2();
	public bool m_hasVertex0;
	public bool m_hasVertex3;
}
