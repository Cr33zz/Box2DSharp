using System;
using System.Diagnostics;

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

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
public class b2ChainShape : b2Shape
{
	public b2ChainShape()
	{
		m_type = Type.e_chain;
		m_radius = (2.0f * Settings.b2_linearSlop);
		m_vertices = null;
		m_count = 0;
		m_hasPrevVertex = false;
		m_hasNextVertex = false;
	}

	/// The destructor frees the vertices using b2Free.
	public new void Dispose()
	{
		Clear();
		base.Dispose();
	}

	/// Clear all data.
	public void Clear()
	{
		m_vertices = null;
		m_count = 0;
	}

	/// Create a loop. This automatically adjusts connectivity.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	public void CreateLoop(b2Vec2[] vertices, int count)
	{
		Debug.Assert(m_vertices == null && m_count == 0);
		Debug.Assert(count >= 3);
		if (count < 3)
		{
			return;
		}

		for (int i = 1; i < count; ++i)
		{
			b2Vec2 v1 = vertices[i - 1];
			b2Vec2 v2 = vertices[i];
			// If the code crashes here, it means your vertices are too close together.
			Debug.Assert(Utils.b2DistanceSquared(v1, v2) > Settings.b2_linearSlop * Settings.b2_linearSlop);
		}

		m_count = count + 1;
		m_vertices = Arrays.InitializeWithDefaultInstances<b2Vec2>(m_count);
		Array.Copy(vertices, m_vertices, count);
		m_vertices[count] = m_vertices[0];
		m_prevVertex = m_vertices[m_count - 2];
		m_nextVertex = m_vertices[1];
		m_hasPrevVertex = true;
		m_hasNextVertex = true;
	}

	/// Create a chain with isolated end vertices.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	public void CreateChain(b2Vec2[] vertices, int count)
	{
		Debug.Assert(m_vertices == null && m_count == 0);
		Debug.Assert(count >= 2);
		for (int i = 1; i < count; ++i)
		{
			// If the code crashes here, it means your vertices are too close together.
			Debug.Assert(Utils.b2DistanceSquared(vertices[i - 1], vertices[i]) > Settings.b2_linearSlop * Settings.b2_linearSlop);
		}

		m_count = count;
        m_vertices = Arrays.InitializeWithDefaultInstances<b2Vec2>(m_count);
        Array.Copy(vertices, m_vertices, m_count);

        m_hasPrevVertex = false;
		m_hasNextVertex = false;

		m_prevVertex.SetZero();
		m_nextVertex.SetZero();
	}

	/// Establish connectivity to a vertex that precedes the first vertex.
	/// Don't call this for loops.
	public void SetPrevVertex(b2Vec2 prevVertex)
	{
		m_prevVertex = prevVertex;
		m_hasPrevVertex = true;
	}

	/// Establish connectivity to a vertex that follows the last vertex.
	/// Don't call this for loops.
	public void SetNextVertex(b2Vec2 nextVertex)
	{
		m_nextVertex = nextVertex;
		m_hasNextVertex = true;
	}

	/// Implement b2Shape. Vertices are cloned using b2Alloc.
	public override b2Shape Clone()
	{
		b2ChainShape clone = new b2ChainShape();
		clone.CreateChain(m_vertices, m_count);
		clone.m_prevVertex = m_prevVertex;
		clone.m_nextVertex = m_nextVertex;
		clone.m_hasPrevVertex = m_hasPrevVertex;
		clone.m_hasNextVertex = m_hasNextVertex;
		return clone;
	}

	/// @see b2Shape::GetChildCount
	public override int GetChildCount()
	{
		// edge count = vertex count - 1
		return m_count - 1;
	}

	/// Get a child edge.
	public void GetChildEdge(b2EdgeShape edge, int index)
	{
		Debug.Assert(0 <= index && index < m_count - 1);
		edge.m_type = b2Shape.Type.e_edge;
		edge.m_radius = m_radius;

		edge.m_vertex1 = m_vertices[index + 0];
		edge.m_vertex2 = m_vertices[index + 1];

		if (index > 0)
		{
			edge.m_vertex0 = m_vertices[index - 1];
			edge.m_hasVertex0 = true;
		}
		else
		{
			edge.m_vertex0 = m_prevVertex;
			edge.m_hasVertex0 = m_hasPrevVertex;
		}

		if (index < m_count - 2)
		{
			edge.m_vertex3 = m_vertices[index + 2];
			edge.m_hasVertex3 = true;
		}
		else
		{
			edge.m_vertex3 = m_nextVertex;
			edge.m_hasVertex3 = m_hasNextVertex;
		}
	}

	/// This always return false.
	/// @see b2Shape::TestPoint
	public override bool TestPoint(b2Transform xf, b2Vec2 p)
	{
		return false;
	}

	/// Implement b2Shape.
	public override bool RayCast(b2RayCastOutput output, b2RayCastInput input, b2Transform xf, int childIndex)
	{
		Debug.Assert(childIndex < m_count);

		b2EdgeShape edgeShape = new b2EdgeShape();

		int i1 = childIndex;
		int i2 = childIndex + 1;
		if (i2 == m_count)
		{
			i2 = 0;
		}

		edgeShape.m_vertex1 = m_vertices[i1];
		edgeShape.m_vertex2 = m_vertices[i2];

		return edgeShape.RayCast(output, input, xf, 0);
	}

	/// @see b2Shape::ComputeAABB
	public override void ComputeAABB(b2AABB aabb, b2Transform xf, int childIndex)
	{
		Debug.Assert(childIndex < m_count);

		int i1 = childIndex;
		int i2 = childIndex + 1;
		if (i2 == m_count)
		{
			i2 = 0;
		}

		b2Vec2 v1 = Utils.b2Mul(xf, m_vertices[i1]);
		b2Vec2 v2 = Utils.b2Mul(xf, m_vertices[i2]);

		aabb.lowerBound = Utils.b2Min(v1, v2);
		aabb.upperBound = Utils.b2Max(v1, v2);
	}

	/// Chains have zero mass.
	/// @see b2Shape::ComputeMass
	public override void ComputeMass(b2MassData massData, float density)
	{
		massData.mass = 0.0f;
		massData.center.SetZero();
		massData.I = 0.0f;
	}

    public override b2Vec2[] GetVertices()
    {
        return m_vertices;
    }

    /// The vertices. Owned by this class.
    public b2Vec2[] m_vertices;

	/// The vertex count.
	public int m_count;

	public b2Vec2 m_prevVertex = new b2Vec2();
	public b2Vec2 m_nextVertex = new b2Vec2();
	public bool m_hasPrevVertex;
	public bool m_hasNextVertex;
}
