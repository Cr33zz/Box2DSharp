using System;
using System.Diagnostics;

/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
public class b2DistanceProxy
{
	public b2DistanceProxy()
	{
		this.m_vertices = null;
		this.m_count = 0;
		this.m_radius = 0.0f;
	}

	/// Initialize the proxy using the given shape. The shape
	/// must remain in scope while the proxy is in use.
	public void Set(b2Shape shape, int index)
	{
		switch (shape.GetType())
		{
		case b2Shape.Type.e_circle:
		{
				b2CircleShape circle = (b2CircleShape)shape;
				m_vertices = new []{circle.m_p};
				m_count = 1;
				m_radius = circle.m_radius;
		}
			break;

		case b2Shape.Type.e_polygon:
		{
				b2PolygonShape polygon = (b2PolygonShape)shape;
				m_vertices = (b2Vec2[])polygon.m_vertices.Clone();
				m_count = polygon.m_count;
				m_radius = polygon.m_radius;
		}
			break;

		case b2Shape.Type.e_chain:
		{
				b2ChainShape chain = (b2ChainShape)shape;
				Debug.Assert(0 <= index && index < chain.m_count);

				m_buffer[0] = chain.m_vertices[index];
				if (index + 1 < chain.m_count)
				{
					m_buffer[1] = chain.m_vertices[index + 1];
				}
				else
				{
					m_buffer[1] = chain.m_vertices[0];
				}

				m_vertices = m_buffer;
				m_count = 2;
				m_radius = chain.m_radius;
		}
			break;

		case b2Shape.Type.e_edge:
		{
				b2EdgeShape edge = (b2EdgeShape)shape;
				m_vertices = new [] {edge.m_vertex1, edge.m_vertex2};
				m_count = 2;
				m_radius = edge.m_radius;
		}
			break;

		default:
			Debug.Assert(false);
			break;
		}
	}

	/// Initialize the proxy using a vertex cloud and radius. The vertices
	/// must remain in scope while the proxy is in use.
	public void Set(b2Vec2[] vertices, int count, float radius)
	{
		m_vertices = (b2Vec2[])vertices.Clone();
		m_count = count;
		m_radius = radius;
	}

	/// Get the supporting vertex index in the given direction.
	public int GetSupport(b2Vec2 d)
	{
		int bestIndex = 0;
		float bestValue = GlobalMembers.b2Dot(m_vertices[0], d);
		for (int i = 1; i < m_count; ++i)
		{
			float value = GlobalMembers.b2Dot(m_vertices[i], d);
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}

		return bestIndex;
	}

	/// Get the supporting vertex in the given direction.
	public b2Vec2 GetSupportVertex(b2Vec2 d)
	{
		int bestIndex = 0;
		float bestValue = GlobalMembers.b2Dot(m_vertices[0], d);
		for (int i = 1; i < m_count; ++i)
		{
			float value = GlobalMembers.b2Dot(m_vertices[i], d);
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}

		return m_vertices[bestIndex];
	}

	/// Get the vertex count.

	//////////////////////////////////////////////////////////////////////////

	public int GetVertexCount()
	{
		return m_count;
	}

	/// Get a vertex by index. Used by b2Distance.
	public b2Vec2 GetVertex(int index)
	{
		Debug.Assert(0 <= index && index < m_count);
		return m_vertices[index];
	}

	public b2Vec2[] m_buffer = new b2Vec2[2];
	public b2Vec2[] m_vertices;
	public int m_count;
	public float m_radius;
}

/// Used to warm start b2Distance.
/// Set count to zero on first call.
public class b2SimplexCache
{
	public float metric; ///< length or area
	public ushort count;
	public byte[] indexA = new byte[3]; ///< vertices on shape A
	public byte[] indexB = new byte[3]; ///< vertices on shape B
}

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
public class b2DistanceInput
{
	public b2DistanceProxy proxyA = new b2DistanceProxy();
	public b2DistanceProxy proxyB = new b2DistanceProxy();
	public b2Transform transformA = new b2Transform();
	public b2Transform transformB = new b2Transform();
	public bool useRadii;
}

/// Output for b2Distance.
public class b2DistanceOutput
{
	public b2Vec2 pointA = new b2Vec2(); ///< closest point on shapeA
	public b2Vec2 pointB = new b2Vec2(); ///< closest point on shapeB
	public float distance;
	public int iterations; ///< number of GJK iterations used
}

/// Input parameters for b2ShapeCast
public class b2ShapeCastInput
{
	public b2DistanceProxy proxyA = new b2DistanceProxy();
	public b2DistanceProxy proxyB = new b2DistanceProxy();
	public b2Transform transformA = new b2Transform();
	public b2Transform transformB = new b2Transform();
	public b2Vec2 translationB = new b2Vec2();
}

/// Output results for b2ShapeCast
public class b2ShapeCastOutput
{
	public b2Vec2 point = new b2Vec2();
	public b2Vec2 normal = new b2Vec2();
	public float lambda;
	public int iterations;
}

public class b2SimplexVertex
{
	public b2Vec2 wA = new b2Vec2(); // support point in proxyA
	public b2Vec2 wB = new b2Vec2(); // support point in proxyB
	public b2Vec2 w = new b2Vec2(); // wB - wA
	public float a; // barycentric coordinate for closest point
	public int indexA; // wA index
	public int indexB; // wB index
}

public class b2Simplex
{
	public void ReadCache(b2SimplexCache cache, b2DistanceProxy proxyA, b2Transform transformA, b2DistanceProxy proxyB, b2Transform transformB)
	{
		Debug.Assert(cache.count <= 3);

		// Copy data from cache.
		m_count = cache.count;
		b2SimplexVertex[] vertices = { m_v1, m_v2, m_v3 };
		for (int i = 0; i < m_count; ++i)
		{
			b2SimplexVertex v = vertices[i];
			v.indexA = cache.indexA[i];
			v.indexB = cache.indexB[i];
			b2Vec2 wALocal = proxyA.GetVertex(v.indexA);
			b2Vec2 wBLocal = proxyB.GetVertex(v.indexB);
			v.wA = GlobalMembers.b2Mul(transformA, wALocal);
			v.wB = GlobalMembers.b2Mul(transformB, wBLocal);
			v.w = v.wB - v.wA;
			v.a = 0.0f;
		}

		// Compute the new simplex metric, if it is substantially different than
		// old metric then flush the simplex.
		if (m_count > 1)
		{
			float metric1 = cache.metric;
			float metric2 = GetMetric();
			if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < float.Epsilon)
			{
				// Reset the simplex.
				m_count = 0;
			}
		}

		// If the cache is empty or invalid ...
		if (m_count == 0)
		{
			b2SimplexVertex v = vertices[0];
			v.indexA = 0;
			v.indexB = 0;
			b2Vec2 wALocal = proxyA.GetVertex(0);
			b2Vec2 wBLocal = proxyB.GetVertex(0);
			v.wA = GlobalMembers.b2Mul(transformA, wALocal);
			v.wB = GlobalMembers.b2Mul(transformB, wBLocal);
			v.w = v.wB - v.wA;
			v.a = 1.0f;
			m_count = 1;
		}
	}

	public void WriteCache(b2SimplexCache cache)
	{
		cache.metric = GetMetric();
		cache.count = (ushort)m_count;
		b2SimplexVertex[] vertices = { m_v1, m_v2, m_v3 };
		for (int i = 0; i < m_count; ++i)
		{
			cache.indexA[i] = (byte)(vertices[i].indexA);
			cache.indexB[i] = (byte)(vertices[i].indexB);
		}
	}

	public b2Vec2 GetSearchDirection()
	{
		switch (m_count)
		{
		case 1:
			return -m_v1.w;

		case 2:
		{
				b2Vec2 e12 = m_v2.w - m_v1.w;
				float sgn = GlobalMembers.b2Cross(e12, -m_v1.w);
				if (sgn > 0.0f)
				{
					// Origin is left of e12.
					return GlobalMembers.b2Cross(1.0f, e12);
				}
				else
				{
					// Origin is right of e12.
					return GlobalMembers.b2Cross(e12, 1.0f);
				}
		}

		default:
			Debug.Assert(false);
			return new b2Vec2();
        }
	}

	public b2Vec2 GetClosestPoint()
	{
		switch (m_count)
		{
		case 0:
			Debug.Assert(false);
			return new b2Vec2();

		case 1:
			return m_v1.w;

		case 2:
			return m_v1.a * m_v1.w + m_v2.a * m_v2.w;

		case 3:
			return new b2Vec2();

            default:
			Debug.Assert(false);
			return new b2Vec2();
        }
	}

	public void GetWitnessPoints(b2Vec2 pA, b2Vec2 pB)
	{
		switch (m_count)
		{
		case 0:
			Debug.Assert(false);
			break;

		case 1:
			pA = m_v1.wA;
			pB = m_v1.wB;
			break;

		case 2:
			pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
			pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
			break;

		case 3:
			pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
			pB = pA;
			break;

		default:
			Debug.Assert(false);
			break;
		}
	}

	public float GetMetric()
	{
		switch (m_count)
		{
		case 0:
			Debug.Assert(false);
			return 0.0f;

		case 1:
			return 0.0f;

		case 2:
			return GlobalMembers.b2Distance(m_v1.w, m_v2.w);

		case 3:
			return GlobalMembers.b2Cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);

		default:
			Debug.Assert(false);
			return 0.0f;
		}
	}


	// Solve a line segment using barycentric coordinates.
	//
	// p = a1 * w1 + a2 * w2
	// a1 + a2 = 1
	//
	// The vector from the origin to the closest point on the line is
	// perpendicular to the line.
	// e12 = w2 - w1
	// dot(p, e) = 0
	// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
	//
	// 2-by-2 linear system
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	//
	// Define
	// d12_1 =  dot(w2, e12)
	// d12_2 = -dot(w1, e12)
	// d12 = d12_1 + d12_2
	//
	// Solution
	// a1 = d12_1 / d12
	// a2 = d12_2 / d12
	public void Solve2()
	{
		b2Vec2 w1 = new b2Vec2(m_v1.w);
		b2Vec2 w2 = new b2Vec2(m_v2.w);
		b2Vec2 e12 = w2 - w1;

		// w1 region
		float d12_2 = -GlobalMembers.b2Dot(w1, e12);
		if (d12_2 <= 0.0f)
		{
			// a2 <= 0, so we clamp it to 0
			m_v1.a = 1.0f;
			m_count = 1;
			return;
		}

		// w2 region
		float d12_1 = GlobalMembers.b2Dot(w2, e12);
		if (d12_1 <= 0.0f)
		{
			// a1 <= 0, so we clamp it to 0
			m_v2.a = 1.0f;
			m_count = 1;
			m_v1 = m_v2;
			return;
		}

		// Must be in e12 region.
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		m_v1.a = d12_1 * inv_d12;
		m_v2.a = d12_2 * inv_d12;
		m_count = 2;
	}

	// Possible regions:
	// - points[2]
	// - edge points[0]-points[2]
	// - edge points[1]-points[2]
	// - inside the triangle
	public void Solve3()
	{
		b2Vec2 w1 = new b2Vec2(m_v1.w);
		b2Vec2 w2 = new b2Vec2(m_v2.w);
		b2Vec2 w3 = new b2Vec2(m_v3.w);

		// Edge12
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		// a3 = 0
		b2Vec2 e12 = w2 - w1;
		float w1e12 = GlobalMembers.b2Dot(w1, e12);
		float w2e12 = GlobalMembers.b2Dot(w2, e12);
		float d12_1 = w2e12;
		float d12_2 = -w1e12;

		// Edge13
		// [1      1     ][a1] = [1]
		// [w1.e13 w3.e13][a3] = [0]
		// a2 = 0
		b2Vec2 e13 = w3 - w1;
		float w1e13 = GlobalMembers.b2Dot(w1, e13);
		float w3e13 = GlobalMembers.b2Dot(w3, e13);
		float d13_1 = w3e13;
		float d13_2 = -w1e13;

		// Edge23
		// [1      1     ][a2] = [1]
		// [w2.e23 w3.e23][a3] = [0]
		// a1 = 0
		b2Vec2 e23 = w3 - w2;
		float w2e23 = GlobalMembers.b2Dot(w2, e23);
		float w3e23 = GlobalMembers.b2Dot(w3, e23);
		float d23_1 = w3e23;
		float d23_2 = -w2e23;

		// Triangle123
		float n123 = GlobalMembers.b2Cross(e12, e13);

		float d123_1 = n123 * GlobalMembers.b2Cross(w2, w3);
		float d123_2 = n123 * GlobalMembers.b2Cross(w3, w1);
		float d123_3 = n123 * GlobalMembers.b2Cross(w1, w2);

		// w1 region
		if (d12_2 <= 0.0f && d13_2 <= 0.0f)
		{
			m_v1.a = 1.0f;
			m_count = 1;
			return;
		}

		// e12
		if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
		{
			float inv_d12 = 1.0f / (d12_1 + d12_2);
			m_v1.a = d12_1 * inv_d12;
			m_v2.a = d12_2 * inv_d12;
			m_count = 2;
			return;
		}

		// e13
		if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
		{
			float inv_d13 = 1.0f / (d13_1 + d13_2);
			m_v1.a = d13_1 * inv_d13;
			m_v3.a = d13_2 * inv_d13;
			m_count = 2;
			m_v2 = m_v3;
			return;
		}

		// w2 region
		if (d12_1 <= 0.0f && d23_2 <= 0.0f)
		{
			m_v2.a = 1.0f;
			m_count = 1;
			m_v1 = m_v2;
			return;
		}

		// w3 region
		if (d13_1 <= 0.0f && d23_1 <= 0.0f)
		{
			m_v3.a = 1.0f;
			m_count = 1;
			m_v1 = m_v3;
			return;
		}

		// e23
		if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
		{
			float inv_d23 = 1.0f / (d23_1 + d23_2);
			m_v2.a = d23_1 * inv_d23;
			m_v3.a = d23_2 * inv_d23;
			m_count = 2;
			m_v1 = m_v3;
			return;
		}

		// Must be in triangle123
		float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
		m_v1.a = d123_1 * inv_d123;
		m_v2.a = d123_2 * inv_d123;
		m_v3.a = d123_3 * inv_d123;
		m_count = 3;
	}

	public b2SimplexVertex m_v1 = new b2SimplexVertex();
	public b2SimplexVertex m_v2 = new b2SimplexVertex();
	public b2SimplexVertex m_v3 = new b2SimplexVertex();
	public int m_count;
}
