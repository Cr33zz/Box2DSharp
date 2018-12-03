// This structure is used to keep track of the best separating axis.
public class b2EPAxis
{
	public enum Type
	{
		e_unknown,
		e_edgeA,
		e_edgeB
	}

	public Type type;
	public int index;
	public float separation;
}

// This holds polygon B expressed in frame A.
public class b2TempPolygon
{
	public b2Vec2[] vertices = Arrays.InitializeWithDefaultInstances<b2Vec2>(Settings.b2_maxPolygonVertices);
	public b2Vec2[] normals = Arrays.InitializeWithDefaultInstances<b2Vec2>(Settings.b2_maxPolygonVertices);
	public int count;
}

// Reference face used for clipping
public class b2ReferenceFace
{
	public int i1;
	public int i2;

	public b2Vec2 v1 = new b2Vec2();
	public b2Vec2 v2 = new b2Vec2();

	public b2Vec2 normal = new b2Vec2();

	public b2Vec2 sideNormal1 = new b2Vec2();
	public float sideOffset1;

	public b2Vec2 sideNormal2 = new b2Vec2();
	public float sideOffset2;
}

// This class collides and edge and a polygon, taking into account edge adjacency.
public class b2EPCollider
{

	// Algorithm:
	// 1. Classify v1 and v2
	// 2. Classify polygon centroid as front or back
	// 3. Flip normal if necessary
	// 4. Initialize normal range to [-pi, pi] about face normal
	// 5. Adjust normal range according to adjacent edges
	// 6. Visit each separating axes, only accept axes within the range
	// 7. Return if _any_ axis indicates separation
	// 8. Clip
	public void Collide(b2Manifold manifold, b2EdgeShape edgeA, b2Transform xfA, b2PolygonShape polygonB, b2Transform xfB)
	{
		m_xf = Utils.b2MulT(xfA, xfB);

		m_centroidB = Utils.b2Mul(m_xf, polygonB.m_centroid);

		m_v0 = edgeA.m_vertex0;
		m_v1 = edgeA.m_vertex1;
		m_v2 = edgeA.m_vertex2;
		m_v3 = edgeA.m_vertex3;

		bool hasVertex0 = edgeA.m_hasVertex0;
		bool hasVertex3 = edgeA.m_hasVertex3;

		b2Vec2 edge1 = m_v2 - m_v1;
		edge1.Normalize();
		m_normal1.Set(edge1.y, -edge1.x);
		float offset1 = Utils.b2Dot(m_normal1, m_centroidB - m_v1);
		float offset0 = 0.0f;
		float offset2 = 0.0f;
		bool convex1 = false;
		bool convex2 = false;

		// Is there a preceding edge?
		if (hasVertex0)
		{
			b2Vec2 edge0 = m_v1 - m_v0;
			edge0.Normalize();
			m_normal0.Set(edge0.y, -edge0.x);
			convex1 = Utils.b2Cross(edge0, edge1) >= 0.0f;
			offset0 = Utils.b2Dot(m_normal0, m_centroidB - m_v0);
		}

		// Is there a following edge?
		if (hasVertex3)
		{
			b2Vec2 edge2 = m_v3 - m_v2;
			edge2.Normalize();
			m_normal2.Set(edge2.y, -edge2.x);
			convex2 = Utils.b2Cross(edge1, edge2) > 0.0f;
			offset2 = Utils.b2Dot(m_normal2, m_centroidB - m_v2);
		}

		// Determine front or back collision. Determine collision normal limits.
		if (hasVertex0 && hasVertex3)
		{
			if (convex1 && convex2)
			{
				m_front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = m_normal0;
					m_upperLimit = m_normal2;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = -m_normal1;
					m_upperLimit = -m_normal1;
				}
			}
			else if (convex1)
			{
				m_front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = m_normal0;
					m_upperLimit = m_normal1;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = -m_normal2;
					m_upperLimit = -m_normal1;
				}
			}
			else if (convex2)
			{
				m_front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = m_normal1;
					m_upperLimit = m_normal2;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = -m_normal1;
					m_upperLimit = -m_normal0;
				}
			}
			else
			{
				m_front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = m_normal1;
					m_upperLimit = m_normal1;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = -m_normal2;
					m_upperLimit = -m_normal0;
				}
			}
		}
		else if (hasVertex0)
		{
			if (convex1)
			{
				m_front = offset0 >= 0.0f || offset1 >= 0.0f;
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = m_normal0;
					m_upperLimit = -m_normal1;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = m_normal1;
					m_upperLimit = -m_normal1;
				}
			}
			else
			{
				m_front = offset0 >= 0.0f && offset1 >= 0.0f;
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = m_normal1;
					m_upperLimit = -m_normal1;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = m_normal1;
					m_upperLimit = -m_normal0;
				}
			}
		}
		else if (hasVertex3)
		{
			if (convex2)
			{
				m_front = offset1 >= 0.0f || offset2 >= 0.0f;
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = -m_normal1;
					m_upperLimit = m_normal2;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = -m_normal1;
					m_upperLimit = m_normal1;
				}
			}
			else
			{
				m_front = offset1 >= 0.0f && offset2 >= 0.0f;
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = -m_normal1;
					m_upperLimit = m_normal1;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = -m_normal2;
					m_upperLimit = m_normal1;
				}
			}
		}
		else
		{
			m_front = offset1 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal1;
			}
		}

		// Get polygonB in frameA
		m_polygonB.count = polygonB.m_count;
		for (int i = 0; i < polygonB.m_count; ++i)
		{
			m_polygonB.vertices[i] = Utils.b2Mul(m_xf, polygonB.m_vertices[i]);
			m_polygonB.normals[i] = Utils.b2Mul(m_xf.q, polygonB.m_normals[i]);
		}

		m_radius = polygonB.m_radius + edgeA.m_radius;

		manifold.pointCount = 0;

		b2EPAxis edgeAxis = ComputeEdgeSeparation();

		// If no valid normal can be found than this edge should not collide.
		if (edgeAxis.type == b2EPAxis.Type.e_unknown)
		{
			return;
		}

		if (edgeAxis.separation > m_radius)
		{
			return;
		}

		b2EPAxis polygonAxis = ComputePolygonSeparation();
		if (polygonAxis.type != b2EPAxis.Type.e_unknown && polygonAxis.separation > m_radius)
		{
			return;
		}

		// Use hysteresis for jitter reduction.
		const float k_relativeTol = 0.98f;
		const float k_absoluteTol = 0.001f;

		b2EPAxis primaryAxis = new b2EPAxis();
		if (polygonAxis.type == b2EPAxis.Type.e_unknown)
		{
			primaryAxis = edgeAxis;
		}
		else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
		{
			primaryAxis = polygonAxis;
		}
		else
		{
			primaryAxis = edgeAxis;
		}

		b2ClipVertex[] ie = Arrays.InitializeWithDefaultInstances<b2ClipVertex>(2);
		b2ReferenceFace rf = new b2ReferenceFace();
		if (primaryAxis.type == b2EPAxis.Type.e_edgeA)
		{
			manifold.type = b2Manifold.Type.e_faceA;

			// Search for the polygon normal that is most anti-parallel to the edge normal.
			int bestIndex = 0;
			float bestValue = Utils.b2Dot(m_normal, m_polygonB.normals[0]);
			for (int i = 1; i < m_polygonB.count; ++i)
			{
				float value = Utils.b2Dot(m_normal, m_polygonB.normals[i]);
				if (value < bestValue)
				{
					bestValue = value;
					bestIndex = i;
				}
			}

			int i1 = bestIndex;
			int i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;

			ie[0].v = m_polygonB.vertices[i1];
			ie[0].id.cf.indexA = 0;
			ie[0].id.cf.indexB = (byte)i1;
			ie[0].id.cf.typeA = (int)b2ContactFeature.Type.e_face;
			ie[0].id.cf.typeB = (int)b2ContactFeature.Type.e_vertex;

			ie[1].v = m_polygonB.vertices[i2];
			ie[1].id.cf.indexA = 0;
			ie[1].id.cf.indexB = (byte)i2;
			ie[1].id.cf.typeA = (int)b2ContactFeature.Type.e_face;
			ie[1].id.cf.typeB = (int)b2ContactFeature.Type.e_vertex;

			if (m_front)
			{
				rf.i1 = 0;
				rf.i2 = 1;
				rf.v1 = m_v1;
				rf.v2 = m_v2;
				rf.normal = m_normal1;
			}
			else
			{
				rf.i1 = 1;
				rf.i2 = 0;
				rf.v1 = m_v2;
				rf.v2 = m_v1;
				rf.normal = -m_normal1;
			}
		}
		else
		{
			manifold.type = b2Manifold.Type.e_faceB;

			ie[0].v = m_v1;
			ie[0].id.cf.indexA = 0;
			ie[0].id.cf.indexB = (byte)primaryAxis.index;
			ie[0].id.cf.typeA = (int)b2ContactFeature.Type.e_vertex;
			ie[0].id.cf.typeB = (int)b2ContactFeature.Type.e_face;

			ie[1].v = m_v2;
			ie[1].id.cf.indexA = 0;
			ie[1].id.cf.indexB = (byte)primaryAxis.index;
			ie[1].id.cf.typeA = (int)b2ContactFeature.Type.e_vertex;
			ie[1].id.cf.typeB = (int)b2ContactFeature.Type.e_face;

			rf.i1 = primaryAxis.index;
			rf.i2 = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
			rf.v1 = m_polygonB.vertices[rf.i1];
			rf.v2 = m_polygonB.vertices[rf.i2];
			rf.normal = m_polygonB.normals[rf.i1];
		}

		rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
		rf.sideNormal2 = -rf.sideNormal1;
		rf.sideOffset1 = Utils.b2Dot(rf.sideNormal1, rf.v1);
		rf.sideOffset2 = Utils.b2Dot(rf.sideNormal2, rf.v2);

		// Clip incident edge against extruded edge1 side edges.
		b2ClipVertex[] clipPoints1 = Arrays.InitializeWithDefaultInstances<b2ClipVertex>(2);
		b2ClipVertex[] clipPoints2 = Arrays.InitializeWithDefaultInstances<b2ClipVertex>(2);
		int np;

		// Clip to box side 1
		np = Utils.b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);

		if (np < Settings.b2_maxManifoldPoints)
		{
			return;
		}

		// Clip to negative box side 1
		np = Utils.b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);

		if (np < Settings.b2_maxManifoldPoints)
		{
			return;
		}

		// Now clipPoints2 contains the clipped points.
		if (primaryAxis.type == b2EPAxis.Type.e_edgeA)
		{
			manifold.localNormal = rf.normal;
			manifold.localPoint = rf.v1;
		}
		else
		{
			manifold.localNormal = polygonB.m_normals[rf.i1];
			manifold.localPoint = polygonB.m_vertices[rf.i1];
		}

		int pointCount = 0;
		for (int i = 0; i < Settings.b2_maxManifoldPoints; ++i)
		{
			float separation;

			separation = Utils.b2Dot(rf.normal, clipPoints2[i].v - rf.v1);

			if (separation <= m_radius)
			{
				b2ManifoldPoint cp = manifold.points[pointCount];

				if (primaryAxis.type == b2EPAxis.Type.e_edgeA)
				{
					cp.localPoint = Utils.b2MulT(m_xf, clipPoints2[i].v);
					cp.id = clipPoints2[i].id;
				}
				else
				{
					cp.localPoint = clipPoints2[i].v;
					cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
					cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
					cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
					cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
				}

				++pointCount;
			}
		}

		manifold.pointCount = pointCount;
	}
	public b2EPAxis ComputeEdgeSeparation()
	{
		b2EPAxis axis = new b2EPAxis();
		axis.type = b2EPAxis.Type.e_edgeA;
		axis.index = m_front ? 0 : 1;
		axis.separation = float.MaxValue;

		for (int i = 0; i < m_polygonB.count; ++i)
		{
			float s = Utils.b2Dot(m_normal, m_polygonB.vertices[i] - m_v1);
			if (s < axis.separation)
			{
				axis.separation = s;
			}
		}

		return axis;
	}
	public b2EPAxis ComputePolygonSeparation()
	{
		b2EPAxis axis = new b2EPAxis();
		axis.type = b2EPAxis.Type.e_unknown;
		axis.index = -1;
		axis.separation = -float.MaxValue;

		b2Vec2 perp = new b2Vec2(-m_normal.y, m_normal.x);

		for (int i = 0; i < m_polygonB.count; ++i)
		{
			b2Vec2 n = -m_polygonB.normals[i];

			float s1 = Utils.b2Dot(n, m_polygonB.vertices[i] - m_v1);
			float s2 = Utils.b2Dot(n, m_polygonB.vertices[i] - m_v2);
			float s = Utils.b2Min(s1, s2);

			if (s > m_radius)
			{
				// No collision
				axis.type = b2EPAxis.Type.e_edgeB;
				axis.index = i;
				axis.separation = s;
				return axis;
			}

			// Adjacency
			if (Utils.b2Dot(n, perp) >= 0.0f)
			{
				if (Utils.b2Dot(n - m_upperLimit, m_normal) < -Settings.b2_angularSlop)
				{
					continue;
				}
			}
			else
			{
				if (Utils.b2Dot(n - m_lowerLimit, m_normal) < -Settings.b2_angularSlop)
				{
					continue;
				}
			}

			if (s > axis.separation)
			{
				axis.type = b2EPAxis.Type.e_edgeB;
				axis.index = i;
				axis.separation = s;
			}
		}

		return axis;
	}

	public enum VertexType
	{
		e_isolated,
		e_concave,
		e_convex
	}

	public b2TempPolygon m_polygonB = new b2TempPolygon();

	public b2Transform m_xf = new b2Transform();
	public b2Vec2 m_centroidB = new b2Vec2();
	public b2Vec2 m_v0 = new b2Vec2();
	public b2Vec2 m_v1 = new b2Vec2();
	public b2Vec2 m_v2 = new b2Vec2();
	public b2Vec2 m_v3 = new b2Vec2();
	public b2Vec2 m_normal0 = new b2Vec2();
	public b2Vec2 m_normal1 = new b2Vec2();
	public b2Vec2 m_normal2 = new b2Vec2();
	public b2Vec2 m_normal = new b2Vec2();
	public VertexType m_type1;
	public VertexType m_type2;
	public b2Vec2 m_lowerLimit = new b2Vec2();
	public b2Vec2 m_upperLimit = new b2Vec2();
	public float m_radius;
	public bool m_front;
}