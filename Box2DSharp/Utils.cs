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

using System;
using System.Diagnostics;

public static class Utils
{
	internal static b2Vec2 ComputeCentroid(b2Vec2[] vs, int count)
	{
		Debug.Assert(count >= 3);

		b2Vec2 c = new b2Vec2();
		c.Set(0.0f, 0.0f);
		float area = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		b2Vec2 pRef = new b2Vec2(0.0f, 0.0f);
	#if false
	//	// This code would put the reference point inside the polygon.
	//	for (int32 i = 0; i < count; ++i)
	//	{
	//		pRef += vs[i];
	//	}
	//	pRef *= 1.0f / count;
	#endif

		float inv3 = 1.0f / 3.0f;

		for (int i = 0; i < count; ++i)
		{
			// Triangle vertices.
			b2Vec2 p1 = new b2Vec2(pRef);
			b2Vec2 p2 = vs[i];
			b2Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

			b2Vec2 e1 = p2 - p1;
			b2Vec2 e2 = p3 - p1;

			float D = b2Cross(e1, e2);

			float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			c += triangleArea * inv3 * (p1 + p2 + p3);
		}

		// Centroid
		Debug.Assert(area > float.Epsilon);
		c *= 1.0f / area;
		return c;
	}


	/// This is used to sort pairs.
	public static int b2PairLessThan(b2Pair pair1, b2Pair pair2)
	{
        if (pair1 != null && pair2 == null)
            return -1;

        if (pair2 != null && pair1 == null)
            return 1;

        if (pair1 == null && pair2 == null)
            return 0;

        if (pair1.proxyIdA < pair2.proxyIdA)
		{
			return 1;
		}

		if (pair1.proxyIdA == pair2.proxyIdA)
		{
			return pair1.proxyIdB < pair2.proxyIdB ? 1 : (pair1.proxyIdB > pair2.proxyIdB ? -1 : 0);
		}

		return -1;
	}

	public static void b2CollideCircles(b2Manifold manifold, b2CircleShape circleA, b2Transform xfA, b2CircleShape circleB, b2Transform xfB)
	{
		manifold.pointCount = 0;

		b2Vec2 pA = b2Mul(xfA, circleA.m_p);
		b2Vec2 pB = b2Mul(xfB, circleB.m_p);

		b2Vec2 d = pB - pA;
		float distSqr = b2Dot(d, d);
		float rA = circleA.m_radius;
		float rB = circleB.m_radius;
		float radius = rA + rB;
		if (distSqr > radius * radius)
		{
			return;
		}

		manifold.type = b2Manifold.Type.e_circles;
		manifold.localPoint = circleA.m_p;
		manifold.localNormal.SetZero();
		manifold.pointCount = 1;

		manifold.points[0].localPoint = circleB.m_p;
		manifold.points[0].id.key = 0;
	}

	public static void b2CollidePolygonAndCircle(b2Manifold manifold, b2PolygonShape polygonA, b2Transform xfA, b2CircleShape circleB, b2Transform xfB)
	{
		manifold.pointCount = 0;

		// Compute circle position in the frame of the polygon.
		b2Vec2 c = b2Mul(xfB, circleB.m_p);
		b2Vec2 cLocal = b2MulT(xfA, c);

		// Find the min separating edge.
		int normalIndex = 0;
		float separation = -float.MaxValue;
		float radius = polygonA.m_radius + circleB.m_radius;
		int vertexCount = polygonA.m_count;
		b2Vec2[] vertices = polygonA.m_vertices;
		b2Vec2[] normals = polygonA.m_normals;

		for (int i = 0; i < vertexCount; ++i)
		{
			float s = b2Dot(normals[i], cLocal - vertices[i]);

			if (s > radius)
			{
				// Early out.
				return;
			}

			if (s > separation)
			{
				separation = s;
				normalIndex = i;
			}
		}

		// Vertices that subtend the incident face.
		int vertIndex1 = normalIndex;
		int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		b2Vec2 v1 = vertices[vertIndex1];
		b2Vec2 v2 = vertices[vertIndex2];

		// If the center is inside the polygon ...
		if (separation < float.Epsilon)
		{
			manifold.pointCount = 1;
			manifold.type = b2Manifold.Type.e_faceA;
			manifold.localNormal = normals[normalIndex];
			manifold.localPoint = 0.5f * (v1 + v2);
			manifold.points[0].localPoint = circleB.m_p;
			manifold.points[0].id.key = 0;
			return;
		}

		// Compute barycentric coordinates
		float u1 = b2Dot(cLocal - v1, v2 - v1);
		float u2 = b2Dot(cLocal - v2, v1 - v2);
		if (u1 <= 0.0f)
		{
			if (b2DistanceSquared(cLocal, v1) > radius * radius)
			{
				return;
			}

			manifold.pointCount = 1;
			manifold.type = b2Manifold.Type.e_faceA;
			manifold.localNormal = cLocal - v1;
			manifold.localNormal.Normalize();
			manifold.localPoint = v1;
			manifold.points[0].localPoint = circleB.m_p;
			manifold.points[0].id.key = 0;
		}
		else if (u2 <= 0.0f)
		{
			if (b2DistanceSquared(cLocal, v2) > radius * radius)
			{
				return;
			}

			manifold.pointCount = 1;
			manifold.type = b2Manifold.Type.e_faceA;
			manifold.localNormal = cLocal - v2;
			manifold.localNormal.Normalize();
			manifold.localPoint = v2;
			manifold.points[0].localPoint = circleB.m_p;
			manifold.points[0].id.key = 0;
		}
		else
		{
			b2Vec2 faceCenter = 0.5f * (v1 + v2);
			float s = b2Dot(cLocal - faceCenter, normals[vertIndex1]);
			if (s > radius)
			{
				return;
			}

			manifold.pointCount = 1;
			manifold.type = b2Manifold.Type.e_faceA;
			manifold.localNormal = normals[vertIndex1];
			manifold.localPoint = faceCenter;
			manifold.points[0].localPoint = circleB.m_p;
			manifold.points[0].id.key = 0;
		}
	}

	// Compute contact points for edge versus circle.
	// This accounts for edge connectivity.
	public static void b2CollideEdgeAndCircle(b2Manifold manifold, b2EdgeShape edgeA, b2Transform xfA, b2CircleShape circleB, b2Transform xfB)
	{
		manifold.pointCount = 0;

		// Compute circle in frame of edge
		b2Vec2 Q = b2MulT(xfA, b2Mul(xfB, circleB.m_p));

		b2Vec2 A = new b2Vec2(edgeA.m_vertex1);
		b2Vec2 B = new b2Vec2(edgeA.m_vertex2);
		b2Vec2 e = B - A;

		// Barycentric coordinates
		float u = b2Dot(e, B - Q);
		float v = b2Dot(e, Q - A);

		float radius = edgeA.m_radius + circleB.m_radius;

		b2ContactFeature cf = new b2ContactFeature();
		cf.indexB = 0;
		cf.typeB = (int)b2ContactFeature.Type.e_vertex;

        b2Vec2 P;
        b2Vec2 d;
        float dd;

        // Region A
        if (v <= 0.0f)
		{
			P = new b2Vec2(A);
			d = Q - P;
			dd = b2Dot(d, d);
			if (dd > radius * radius)
			{
				return;
			}

			// Is there an edge connected to A?
			if (edgeA.m_hasVertex0)
			{
				b2Vec2 A1 = new b2Vec2(edgeA.m_vertex0);
				b2Vec2 B1 = new b2Vec2(A);
				b2Vec2 e1 = B1 - A1;
				float u1 = b2Dot(e1, B1 - Q);

				// Is the circle in Region AB of the previous edge?
				if (u1 > 0.0f)
				{
					return;
				}
			}

			cf.indexA = 0;
			cf.typeA = (int)b2ContactFeature.Type.e_vertex;
			manifold.pointCount = 1;
			manifold.type = b2Manifold.Type.e_circles;
			manifold.localNormal.SetZero();
			manifold.localPoint = P;
			manifold.points[0].id.key = 0;
			manifold.points[0].id.cf = cf;
			manifold.points[0].localPoint = circleB.m_p;
			return;
		}

		// Region B
		if (u <= 0.0f)
		{
			P = new b2Vec2(B);
			d = Q - P;
			dd = b2Dot(d, d);
			if (dd > radius * radius)
			{
				return;
			}

			// Is there an edge connected to B?
			if (edgeA.m_hasVertex3)
			{
				b2Vec2 B2 = new b2Vec2(edgeA.m_vertex3);
				b2Vec2 A2 = new b2Vec2(B);
				b2Vec2 e2 = B2 - A2;
				float v2 = b2Dot(e2, Q - A2);

				// Is the circle in Region AB of the next edge?
				if (v2 > 0.0f)
				{
					return;
				}
			}

			cf.indexA = 1;
			cf.typeA = (int)b2ContactFeature.Type.e_vertex;
			manifold.pointCount = 1;
			manifold.type = b2Manifold.Type.e_circles;
			manifold.localNormal.SetZero();
			manifold.localPoint = P;
			manifold.points[0].id.key = 0;
			manifold.points[0].id.cf = cf;
			manifold.points[0].localPoint = circleB.m_p;
			return;
		}

		// Region AB
		float den = b2Dot(e, e);
		Debug.Assert(den > 0.0f);
		P = (1.0f / den) * (u * A + v * B);
		d = Q - P;
		dd = b2Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b2Vec2 n = new b2Vec2(-e.y, e.x);
		if (b2Dot(n, Q - A) < 0.0f)
		{
			n.Set(-n.x, -n.y);
		}
		n.Normalize();

		cf.indexA = 0;
		cf.typeA = (int)b2ContactFeature.Type.e_face;
		manifold.pointCount = 1;
		manifold.type = b2Manifold.Type.e_faceA;
		manifold.localNormal = n;
		manifold.localPoint = A;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB.m_p;
	}

	public static void b2CollideEdgeAndPolygon(b2Manifold manifold, b2EdgeShape edgeA, b2Transform xfA, b2PolygonShape polygonB, b2Transform xfB)
	{
		b2EPCollider collider = new b2EPCollider();
		collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
	}

	// Find the max separation between poly1 and poly2 using edge normals from poly1.
	internal static float b2FindMaxSeparation(ref int edgeIndex, b2PolygonShape poly1, b2Transform xf1, b2PolygonShape poly2, b2Transform xf2)
	{
		int count1 = poly1.m_count;
		int count2 = poly2.m_count;
		b2Vec2[] n1s = poly1.m_normals;
		b2Vec2[] v1s = poly1.m_vertices;
		b2Vec2[] v2s = poly2.m_vertices;
		b2Transform xf = b2MulT(xf2, xf1);

		int bestIndex = 0;
		float maxSeparation = -float.MaxValue;
		for (int i = 0; i < count1; ++i)
		{
			// Get poly1 normal in frame2.
			b2Vec2 n = b2Mul(xf.q, n1s[i]);
			b2Vec2 v1 = b2Mul(xf, v1s[i]);

			// Find deepest point for normal i.
			float si = float.MaxValue;
			for (int j = 0; j < count2; ++j)
			{
				float sij = b2Dot(n, v2s[j] - v1);
				if (sij < si)
				{
					si = sij;
				}
			}

			if (si > maxSeparation)
			{
				maxSeparation = si;
				bestIndex = i;
			}
		}

		edgeIndex = bestIndex;
		return maxSeparation;
	}

	internal static void b2FindIncidentEdge(b2ClipVertex[] c, b2PolygonShape poly1, b2Transform xf1, int edge1, b2PolygonShape poly2, b2Transform xf2)
	{
		b2Vec2[] normals1 = poly1.m_normals;

		int count2 = poly2.m_count;
		b2Vec2[] vertices2 = poly2.m_vertices;
		b2Vec2[] normals2 = poly2.m_normals;

		Debug.Assert(0 <= edge1 && edge1 < poly1.m_count);

		// Get the normal of the reference edge in poly2's frame.
		b2Vec2 normal1 = b2MulT(xf2.q, b2Mul(xf1.q, normals1[edge1]));

		// Find the incident edge on poly2.
		int index = 0;
		float minDot = float.MaxValue;
		for (int i = 0; i < count2; ++i)
		{
			float dot = b2Dot(normal1, normals2[i]);
			if (dot < minDot)
			{
				minDot = dot;
				index = i;
			}
		}

		// Build the clip vertices for the incident edge.
		int i1 = index;
		int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

		c[0].v = b2Mul(xf2, vertices2[i1]);
		c[0].id.cf.indexA = (byte)edge1;
		c[0].id.cf.indexB = (byte)i1;
		c[0].id.cf.typeA = (int)b2ContactFeature.Type.e_face;
		c[0].id.cf.typeB = (int)b2ContactFeature.Type.e_vertex;

		c[1].v = b2Mul(xf2, vertices2[i2]);
		c[1].id.cf.indexA = (byte)edge1;
		c[1].id.cf.indexB = (byte)i2;
		c[1].id.cf.typeA = (int)b2ContactFeature.Type.e_face;
		c[1].id.cf.typeB = (int)b2ContactFeature.Type.e_vertex;
	}

	// Find edge normal of max separation on A - return if separating axis is found
	// Find edge normal of max separation on B - return if separation axis is found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip

	// The normal points from 1 to 2
	public static void b2CollidePolygons(b2Manifold manifold, b2PolygonShape polyA, b2Transform xfA, b2PolygonShape polyB, b2Transform xfB)
	{
		manifold.pointCount = 0;
		float totalRadius = polyA.m_radius + polyB.m_radius;

		int edgeA = 0;
		float separationA = b2FindMaxSeparation(ref edgeA, polyA, xfA, polyB, xfB);
		if (separationA > totalRadius)
		{
			return;
		}

		int edgeB = 0;
		float separationB = b2FindMaxSeparation(ref edgeB, polyB, xfB, polyA, xfA);
		if (separationB > totalRadius)
		{
			return;
		}

		b2PolygonShape poly1; // reference polygon
		b2PolygonShape poly2; // incident polygon
		b2Transform xf1 = new b2Transform();
		b2Transform xf2 = new b2Transform();
		int edge1; // reference edge
		byte flip;
		float k_tol = 0.1f * Settings.b2_linearSlop;

		if (separationB > separationA + k_tol)
		{
			poly1 = polyB;
			poly2 = polyA;
			xf1 = xfB;
			xf2 = xfA;
			edge1 = edgeB;
			manifold.type = b2Manifold.Type.e_faceB;
			flip = 1;
		}
		else
		{
			poly1 = polyA;
			poly2 = polyB;
			xf1 = xfA;
			xf2 = xfB;
			edge1 = edgeA;
			manifold.type = b2Manifold.Type.e_faceA;
			flip = 0;
		}

		b2ClipVertex[] incidentEdge = Arrays.InitializeWithDefaultInstances<b2ClipVertex>(2);
		b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		int count1 = poly1.m_count;
		b2Vec2[] vertices1 = poly1.m_vertices;

		int iv1 = edge1;
		int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

		b2Vec2 v11 = vertices1[iv1];
		b2Vec2 v12 = vertices1[iv2];

		b2Vec2 localTangent = v12 - v11;
		localTangent.Normalize();

		b2Vec2 localNormal = b2Cross(localTangent, 1.0f);
		b2Vec2 planePoint = 0.5f * (v11 + v12);

		b2Vec2 tangent = b2Mul(xf1.q, localTangent);
		b2Vec2 normal = b2Cross(tangent, 1.0f);



		v11 = b2Mul(xf1, v11);


		v12 = b2Mul(xf1, v12);

		// Face offset.
		float frontOffset = b2Dot(normal, v11);

		// Side offsets, extended by polytope skin thickness.
		float sideOffset1 = -b2Dot(tangent, v11) + totalRadius;
		float sideOffset2 = b2Dot(tangent, v12) + totalRadius;

		// Clip incident edge against extruded edge1 side edges.
		b2ClipVertex[] clipPoints1 = Arrays.InitializeWithDefaultInstances<b2ClipVertex>(2);
		b2ClipVertex[] clipPoints2 = Arrays.InitializeWithDefaultInstances<b2ClipVertex>(2);
		int np;

		// Clip to box side 1
		np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

		if (np < 2)
		{
			return;
		}

		// Clip to negative box side 1
		np = b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

		if (np < 2)
		{
			return;
		}

		// Now clipPoints2 contains the clipped points.
		manifold.localNormal = localNormal;
		manifold.localPoint = planePoint;

		int pointCount = 0;
		for (int i = 0; i < Settings.b2_maxManifoldPoints; ++i)
		{
			float separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;

			if (separation <= totalRadius)
			{
				b2ManifoldPoint cp = manifold.points[pointCount];
				cp.localPoint = b2MulT(xf2, clipPoints2[i].v);
				cp.id = clipPoints2[i].id;
				if (flip != 0)
				{
					// Swap features
					b2ContactFeature cf = new b2ContactFeature(cp.id.cf);
					cp.id.cf.indexA = cf.indexB;
					cp.id.cf.indexB = cf.indexA;
					cp.id.cf.typeA = cf.typeB;
					cp.id.cf.typeB = cf.typeA;
				}
				++pointCount;
			}
		}

		manifold.pointCount = pointCount;
	}


	public static readonly byte b2_nullFeature = byte.MaxValue;

	/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
	/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
	public static void b2GetPointStates(PointState[] state1, PointState[] state2, b2Manifold manifold1, b2Manifold manifold2)
	{
		for (int i = 0; i < Settings.b2_maxManifoldPoints; ++i)
		{
			state1[i] = PointState.nullState;
			state2[i] = PointState.nullState;
		}

		// Detect persists and removes.
		for (int i = 0; i < manifold1.pointCount; ++i)
		{
			b2ContactID id = manifold1.points[i].id;

			state1[i] = PointState.removeState;

			for (int j = 0; j < manifold2.pointCount; ++j)
			{
				if (manifold2.points[j].id.key == id.key)
				{
					state1[i] = PointState.persistState;
					break;
				}
			}
		}

		// Detect persists and adds.
		for (int i = 0; i < manifold2.pointCount; ++i)
		{
			b2ContactID id = manifold2.points[i].id;

			state2[i] = PointState.addState;

			for (int j = 0; j < manifold1.pointCount; ++j)
			{
				if (manifold1.points[j].id.key == id.key)
				{
					state2[i] = PointState.persistState;
					break;
				}
			}
		}
	}

	/// Compute the collision manifold between two circles.

	//void b2CollideCircles(b2Manifold manifold, b2CircleShape circleA, b2Transform xfA, b2CircleShape circleB, b2Transform xfB);

	/// Compute the collision manifold between a polygon and a circle.

	//void b2CollidePolygonAndCircle(b2Manifold manifold, b2PolygonShape polygonA, b2Transform xfA, b2CircleShape circleB, b2Transform xfB);

	/// Compute the collision manifold between two polygons.

	//void b2CollidePolygons(b2Manifold manifold, b2PolygonShape polygonA, b2Transform xfA, b2PolygonShape polygonB, b2Transform xfB);

	/// Compute the collision manifold between an edge and a circle.

	//void b2CollideEdgeAndCircle(b2Manifold manifold, b2EdgeShape polygonA, b2Transform xfA, b2CircleShape circleB, b2Transform xfB);

	/// Compute the collision manifold between an edge and a circle.

	//void b2CollideEdgeAndPolygon(b2Manifold manifold, b2EdgeShape edgeA, b2Transform xfA, b2PolygonShape circleB, b2Transform xfB);

// Sutherland-Hodgman clipping.

	/// Clipping for contact manifolds.
	public static int b2ClipSegmentToLine(b2ClipVertex[] vOut, b2ClipVertex[] vIn, b2Vec2 normal, float offset, int vertexIndexA)
	{
		// Start with no output points
		int numOut = 0;

		// Calculate the distance of end points to the line
		float distance0 = b2Dot(normal, vIn[0].v) - offset;
		float distance1 = b2Dot(normal, vIn[1].v) - offset;

		// If the points are behind the plane
		if (distance0 <= 0.0f)
		{
			vOut[numOut++] = vIn[0];
		}
		if (distance1 <= 0.0f)
		{
			vOut[numOut++] = vIn[1];
		}

		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0f)
		{
			// Find intersection point of edge and plane
			float interp = distance0 / (distance0 - distance1);


			vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

			// VertexA is hitting edgeB.
			vOut[numOut].id.cf.indexA = (byte)vertexIndexA;
			vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
			vOut[numOut].id.cf.typeA = (int)b2ContactFeature.Type.e_vertex;
			vOut[numOut].id.cf.typeB = (int)b2ContactFeature.Type.e_face;
			++numOut;
		}

		return numOut;
	}

	/// Determine if two generic shapes overlap.
	public static bool b2TestOverlap(b2Shape shapeA, int indexA, b2Shape shapeB, int indexB, b2Transform xfA, b2Transform xfB)
	{
		b2DistanceInput input = new b2DistanceInput();
		input.proxyA.Set(shapeA, indexA);
		input.proxyB.Set(shapeB, indexB);


		input.transformA = xfA;


		input.transformB = xfB;
		input.useRadii = true;

		b2SimplexCache cache = new b2SimplexCache();
		cache.count = 0;

		b2DistanceOutput output = new b2DistanceOutput();

		b2Distance(output, cache, input);

		return output.distance < 10.0f * float.Epsilon;
	}

	public static bool b2TestOverlap(b2AABB a, b2AABB b)
	{
		b2Vec2 d1 = new b2Vec2();
		b2Vec2 d2 = new b2Vec2();
		d1 = b.lowerBound - a.upperBound;
		d2 = a.lowerBound - b.upperBound;

		if (d1.x > 0.0f || d1.y > 0.0f)
		{
			return false;
		}

		if (d2.x > 0.0f || d2.y > 0.0f)
		{
			return false;
		}

		return true;
	}


	/// Compute the closest points between two shapes. Supports any combination of:
	/// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
	/// On the first call set b2SimplexCache.count to zero.
	public static void b2Distance(b2DistanceOutput output, b2SimplexCache cache, b2DistanceInput input)
	{
		++b2_gjkCalls;

		b2DistanceProxy proxyA = input.proxyA;
		b2DistanceProxy proxyB = input.proxyB;

		b2Transform transformA = new b2Transform(input.transformA);
		b2Transform transformB = new b2Transform(input.transformB);

		// Initialize the simplex.
		b2Simplex simplex = new b2Simplex();
		simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

		// Get simplex vertices as an array.
	    b2SimplexVertex[] vertices = new [] {simplex.m_v1, simplex.m_v2, simplex.m_v3};
		const int k_maxIters = 20;

		// These store the vertices of the last simplex so that we
		// can check for duplicates and prevent cycling.
		int[] saveA = new int[3];
		int[] saveB = new int[3];
		int saveCount = 0;

		// Main iteration loop.
		int iter = 0;
		while (iter < k_maxIters)
		{
			// Copy simplex so we can identify duplicates.
			saveCount = simplex.m_count;
			for (int i = 0; i < saveCount; ++i)
			{
				saveA[i] = vertices[i].indexA;
				saveB[i] = vertices[i].indexB;
			}

			switch (simplex.m_count)
			{
			case 1:
				break;

			case 2:
				simplex.Solve2();
				break;

			case 3:
				simplex.Solve3();
				break;

			default:
				Debug.Assert(false);
			break;
			}

			// If we have 3 points, then the origin is in the corresponding triangle.
			if (simplex.m_count == 3)
			{
				break;
			}

			// Get search direction.
			b2Vec2 d = simplex.GetSearchDirection();

			// Ensure the search direction is numerically fit.
			if (d.LengthSquared() < float.Epsilon * float.Epsilon)
			{
				// The origin is probably contained by a line segment
				// or triangle. Thus the shapes are overlapped.

				// We can't return zero here even though there may be overlap.
				// In case the simplex is a point, segment, or triangle it is difficult
				// to determine if the origin is contained in the CSO or very close to it.
				break;
			}

			// Compute a tentative new simplex vertex using support points.
			b2SimplexVertex vertex = vertices[simplex.m_count];
			vertex.indexA = proxyA.GetSupport(b2MulT(transformA.q, -d));
			vertex.wA = b2Mul(transformA, proxyA.GetVertex(vertex.indexA));
			//b2Vec2 wBLocal = new b2Vec2();
			vertex.indexB = proxyB.GetSupport(b2MulT(transformB.q, d));
			vertex.wB = b2Mul(transformB, proxyB.GetVertex(vertex.indexB));
			vertex.w = vertex.wB - vertex.wA;

			// Iteration count is equated to the number of support point calls.
			++iter;
			++b2_gjkIters;

			// Check for duplicate support points. This is the main termination criteria.
			bool duplicate = false;
			for (int i = 0; i < saveCount; ++i)
			{
				if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
				{
					duplicate = true;
					break;
				}
			}

			// If we found a duplicate support point we must exit to avoid cycling.
			if (duplicate)
			{
				break;
			}

			// New vertex is ok and needed.
			++simplex.m_count;
		}

		b2_gjkMaxIters = b2Max(b2_gjkMaxIters, iter);

		// Prepare output.
		simplex.GetWitnessPoints(output.pointA, output.pointB);
		output.distance = b2Distance(output.pointA, output.pointB);
		output.iterations = iter;

		// Cache the simplex.
		simplex.WriteCache(cache);

		// Apply radii if requested.
		if (input.useRadii)
		{
			float rA = proxyA.m_radius;
			float rB = proxyB.m_radius;

			if (output.distance > rA + rB && output.distance > float.Epsilon)
			{
				// Shapes are still no overlapped.
				// Move the witness points to the outer surface.
				output.distance -= rA + rB;
				b2Vec2 normal = output.pointB - output.pointA;
				normal.Normalize();
				output.pointA += rA * normal;
				output.pointB -= rB * normal;
			}
			else
			{
				// Shapes are overlapped when radii are considered.
				// Move the witness points to the middle.
				b2Vec2 p = 0.5f * (output.pointA + output.pointB);


				output.pointA = p;


				output.pointB = p;
				output.distance = 0.0f;
			}
		}
	}

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010

	/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
	public static bool b2ShapeCast(b2ShapeCastOutput output, b2ShapeCastInput input)
	{
		output.iterations = 0;
		output.lambda = 1.0f;
		output.normal.SetZero();
		output.point.SetZero();

		b2DistanceProxy proxyA = input.proxyA;
		b2DistanceProxy proxyB = input.proxyB;

		float radiusA = b2Max(proxyA.m_radius, (2.0f * Settings.b2_linearSlop));
		float radiusB = b2Max(proxyB.m_radius, (2.0f * Settings.b2_linearSlop));
		float radius = radiusA + radiusB;

		b2Transform xfA = new b2Transform(input.transformA);
		b2Transform xfB = new b2Transform(input.transformB);

		b2Vec2 r = new b2Vec2(input.translationB);
		b2Vec2 n = new b2Vec2(0.0f, 0.0f);
		float lambda = 0.0f;

		// Initial simplex
		b2Simplex simplex = new b2Simplex();
		simplex.m_count = 0;

		// Get simplex vertices as an array.
		b2SimplexVertex[] vertices = new[] { simplex.m_v1, simplex.m_v2, simplex.m_v3 };

		// Get support point in -r direction
		int indexA = proxyA.GetSupport(b2MulT(xfA.q, -r));
		b2Vec2 wA = b2Mul(xfA, proxyA.GetVertex(indexA));
		int indexB = proxyB.GetSupport(b2MulT(xfB.q, r));
		b2Vec2 wB = b2Mul(xfB, proxyB.GetVertex(indexB));
		b2Vec2 v = wA - wB;

        // Sigma is the target distance between polygons
        float sigma = Utils.b2Max(Settings.b2_polygonRadius, radius - Settings.b2_polygonRadius);
		float tolerance = 0.5f * Settings.b2_linearSlop;

		// Main iteration loop.
		const int k_maxIters = 20;
		int iter = 0;
		while (iter < k_maxIters && b2Abs(v.Length() - sigma) > tolerance)
		{
			Debug.Assert(simplex.m_count < 3);

			output.iterations += 1;

			// Support in direction -v (A - B)
			indexA = proxyA.GetSupport(b2MulT(xfA.q, -v));


			wA = b2Mul(xfA, proxyA.GetVertex(indexA));
			indexB = proxyB.GetSupport(b2MulT(xfB.q, v));


			wB = b2Mul(xfB, proxyB.GetVertex(indexB));
			b2Vec2 p = wA - wB;

			// -v is a normal at p
			v.Normalize();

			// Intersect ray with plane
			float vp = b2Dot(v, p);
			float vr = b2Dot(v, r);
			if (vp - sigma > lambda * vr)
			{
				if (vr <= 0.0f)
				{
					return false;
				}

				lambda = (vp - sigma) / vr;
				if (lambda > 1.0f)
				{
					return false;
				}



				n = -v;
				simplex.m_count = 0;
			}

			// Reverse simplex since it works with B - A.
			// Shift by lambda * r because we want the closest point to the current clip point.
			// Note that the support point p is not shifted because we want the plane equation
			// to be formed in unshifted space.
			b2SimplexVertex vertex = vertices[simplex.m_count];
			vertex.indexA = indexB;


			vertex.wA = wB + lambda * r;
			vertex.indexB = indexA;


			vertex.wB = wA;


			vertex.w = vertex.wB - vertex.wA;
			vertex.a = 1.0f;
			simplex.m_count += 1;

			switch (simplex.m_count)
			{
			case 1:
				break;

			case 2:
				simplex.Solve2();
				break;

			case 3:
				simplex.Solve3();
				break;

			default:
				Debug.Assert(false);
			break;
			}

			// If we have 3 points, then the origin is in the corresponding triangle.
			if (simplex.m_count == 3)
			{
				// Overlap
				return false;
			}

			// Get search direction.


			v = simplex.GetClosestPoint();

			// Iteration count is equated to the number of support point calls.
			++iter;
		}

		// Prepare output.
		b2Vec2 pointA = new b2Vec2();
		b2Vec2 pointB = new b2Vec2();
		simplex.GetWitnessPoints(pointB, pointA);

		if (v.LengthSquared() > 0.0f)
		{


			n = -v;
			n.Normalize();
		}



		output.point = pointA + radiusA * n;


		output.normal = n;
		output.lambda = lambda;
		output.iterations = iter;
		return true;
	}



	// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
	public static int b2_gjkCalls;
	public static int b2_gjkIters;
	public static int b2_gjkMaxIters;

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.

	/// Compute the upper bound on time before two shapes penetrate. Time is represented as
	/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
	/// non-tunneling collisions. If you change the time interval, you should call this function
	/// again.
	/// Note: use b2Distance to compute the contact point and normal at the time of impact.
	public static void b2TimeOfImpact(b2TOIOutput output, b2TOIInput input)
	{
		b2Timer timer = new b2Timer();

		++b2_toiCalls;

		output.state = b2TOIOutput.State.e_unknown;
		output.t = input.tMax;

		b2DistanceProxy proxyA = input.proxyA;
		b2DistanceProxy proxyB = input.proxyB;

		b2Sweep sweepA = input.sweepA;
		b2Sweep sweepB = input.sweepB;

		// Large rotations can make the root finder fail, so we normalize the
		// sweep angles.
		sweepA.Normalize();
		sweepB.Normalize();

		float tMax = input.tMax;

		float totalRadius = proxyA.m_radius + proxyB.m_radius;
		float target = b2Max(Settings.b2_linearSlop, totalRadius - 3.0f * Settings.b2_linearSlop);
		float tolerance = 0.25f * Settings.b2_linearSlop;
		Debug.Assert(target > tolerance);

		float t1 = 0.0f;
		const int k_maxIterations = 20; // TODO_ERIN b2Settings
		int iter = 0;

		// Prepare input for distance query.
		b2SimplexCache cache = new b2SimplexCache();
		cache.count = 0;
		b2DistanceInput distanceInput = new b2DistanceInput();


		distanceInput.proxyA = input.proxyA;


		distanceInput.proxyB = input.proxyB;
		distanceInput.useRadii = false;

		// The outer loop progressively attempts to compute new separating axes.
		// This loop terminates when an axis is repeated (no progress is made).
		for (;;)
		{
			b2Transform xfA = new b2Transform();
			b2Transform xfB = new b2Transform();
			sweepA.GetTransform(xfA, t1);
			sweepB.GetTransform(xfB, t1);

			// Get the distance between shapes. We can also use the results
			// to get a separating axis.


			distanceInput.transformA = xfA;


			distanceInput.transformB = xfB;
			b2DistanceOutput distanceOutput = new b2DistanceOutput();
			b2Distance(distanceOutput, cache, distanceInput);

			// If the shapes are overlapped, we give up on continuous collision.
			if (distanceOutput.distance <= 0.0f)
			{
				// Failure!
				output.state = b2TOIOutput.State.e_overlapped;
				output.t = 0.0f;
				break;
			}

			if (distanceOutput.distance < target + tolerance)
			{
				// Victory!
				output.state = b2TOIOutput.State.e_touching;
				output.t = t1;
				break;
			}

			// Initialize the separating axis.
			b2SeparationFunction fcn = new b2SeparationFunction();
			fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);
	#if false
	//		// Dump the curve seen by the root finder
	//		{
	//			const int32 N = 100;
	//			float32 dx = 1.0f / N;
	//			float32 xs[N+1];
	//			float32 fs[N+1];
	//
	//			float32 x = 0.0f;
	//
	//			for (int32 i = 0; i <= N; ++i)
	//			{
	//				sweepA.GetTransform(&xfA, x);
	//				sweepB.GetTransform(&xfB, x);
	//				float32 f = fcn.Evaluate(xfA, xfB) - target;
	//
	//				printf("%g %g\n", x, f);
	//
	//				xs[i] = x;
	//				fs[i] = f;
	//
	//				x += dx;
	//			}
	//		}
	#endif

			// Compute the TOI on the separating axis. We do this by successively
			// resolving the deepest point. This loop is bounded by the number of vertices.
			bool done = false;
			float t2 = tMax;
			int pushBackIter = 0;
			for (;;)
			{
				// Find the deepest point at t2. Store the witness point indices.
				int indexA = 0;
				int indexB = 0;
				float s2 = fcn.FindMinSeparation(ref indexA, ref indexB, t2);

				// Is the final configuration separated?
				if (s2 > target + tolerance)
				{
					// Victory!
					output.state = b2TOIOutput.State.e_separated;
					output.t = tMax;
					done = true;
					break;
				}

				// Has the separation reached tolerance?
				if (s2 > target - tolerance)
				{
					// Advance the sweeps
					t1 = t2;
					break;
				}

				// Compute the initial separation of the witness points.
				float s1 = fcn.Evaluate(indexA, indexB, t1);

				// Check for initial overlap. This might happen if the root finder
				// runs out of iterations.
				if (s1 < target - tolerance)
				{
					output.state = b2TOIOutput.State.e_failed;
					output.t = t1;
					done = true;
					break;
				}

				// Check for touching
				if (s1 <= target + tolerance)
				{
					// Victory! t1 should hold the TOI (could be 0.0).
					output.state = b2TOIOutput.State.e_touching;
					output.t = t1;
					done = true;
					break;
				}

				// Compute 1D root of: f(x) - target = 0
				int rootIterCount = 0;
				float a1 = t1;
				float a2 = t2;
				for (;;)
				{
					// Use a mix of the secant rule and bisection.
					float t;
					if ((rootIterCount & 1) != 0)
					{
						// Secant rule to improve convergence.
						t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
					}
					else
					{
						// Bisection to guarantee progress.
						t = 0.5f * (a1 + a2);
					}

					++rootIterCount;
					++b2_toiRootIters;

					float s = fcn.Evaluate(indexA, indexB, t);

					if (b2Abs(s - target) < tolerance)
					{
						// t2 holds a tentative value for t1
						t2 = t;
						break;
					}

					// Ensure we continue to bracket the root.
					if (s > target)
					{
						a1 = t;
						s1 = s;
					}
					else
					{
						a2 = t;
						s2 = s;
					}

					if (rootIterCount == 50)
					{
						break;
					}
				}

				b2_toiMaxRootIters = b2Max(b2_toiMaxRootIters, rootIterCount);

				++pushBackIter;

				if (pushBackIter == Settings.b2_maxPolygonVertices)
				{
					break;
				}
			}

			++iter;
			++b2_toiIters;

			if (done)
			{
				break;
			}

			if (iter == k_maxIterations)
			{
				// Root finder got stuck. Semi-victory.
				output.state = b2TOIOutput.State.e_failed;
				output.t = t1;
				break;
			}
		}

		b2_toiMaxIters = b2Max(b2_toiMaxIters, iter);

		float time = timer.GetMilliseconds();
		b2_toiMaxTime = b2Max(b2_toiMaxTime, time);
		b2_toiTime += time;
	}

	public static float b2_toiTime;
	public static float b2_toiMaxTime;
	public static int b2_toiCalls;
	public static int b2_toiIters;
	public static int b2_toiMaxIters;
	public static int b2_toiRootIters;
	public static int b2_toiMaxRootIters;
	public static readonly int b2_chunkSize = 16 * 1024;
	public static readonly int b2_maxBlockSize = 640;
	public static readonly int b2_blockSizes = 14;
	public static readonly int b2_chunkArrayIncrement = 128;

	/// This function is used to ensure that a floating point number is not a NaN or infinity.
	public static bool b2IsValid(float x)
	{
		return  !Double.IsInfinity(x) && !Double.IsNaN(x);
	}

	/// Useful constant

	/// Perform the dot product on two vectors.
	public static float b2Dot(b2Vec2 a, b2Vec2 b)
	{
		return a.x * b.x + a.y * b.y;
	}

	/// Perform the cross product on two vectors. In 2D this produces a scalar.
	public static float b2Cross(b2Vec2 a, b2Vec2 b)
	{
		return a.x * b.y - a.y * b.x;
	}

	/// Perform the cross product on a vector and a scalar. In 2D this produces
	/// a vector.
	public static b2Vec2 b2Cross(b2Vec2 a, float s)
	{
		return new b2Vec2(s * a.y, -s * a.x);
	}

	/// Perform the cross product on a scalar and a vector. In 2D this produces
	/// a vector.
	public static b2Vec2 b2Cross(float s, b2Vec2 a)
	{
		return new b2Vec2(-s * a.y, s * a.x);
	}

	/// Multiply a matrix times a vector. If a rotation matrix is provided,
	/// then this transforms the vector from one frame to another.
	public static b2Vec2 b2Mul(b2Mat22 A, b2Vec2 v)
	{
		return new b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
	}

	/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
	/// then this transforms the vector from one frame to another (inverse transform).
	public static b2Vec2 b2MulT(b2Mat22 A, b2Vec2 v)
	{
		return new b2Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey));
	}

	public static float b2Distance(b2Vec2 a, b2Vec2 b)
	{
		b2Vec2 c = a - b;
		return c.Length();
	}

	public static float b2DistanceSquared(b2Vec2 a, b2Vec2 b)
	{
		b2Vec2 c = a - b;
		return b2Dot(c, c);
	}

	/// Perform the dot product on two vectors.
	public static float b2Dot(b2Vec3 a, b2Vec3 b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	/// Perform the cross product on two vectors.
	public static b2Vec3 b2Cross(b2Vec3 a, b2Vec3 b)
	{
		return new b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	// A * B
	public static b2Mat22 b2Mul(b2Mat22 A, b2Mat22 B)
	{
		return new b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey));
	}

	// A^T * B
	public static b2Mat22 b2MulT(b2Mat22 A, b2Mat22 B)
	{
		b2Vec2 c1 = new b2Vec2(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
		b2Vec2 c2 = new b2Vec2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
		return new b2Mat22(c1, c2);
	}

	/// Multiply a matrix times a vector.
	public static b2Vec3 b2Mul(b2Mat33 A, b2Vec3 v)
	{
		return v.x * A.ex + v.y * A.ey + v.z * A.ez;
	}

	/// Multiply a matrix times a vector.
	public static b2Vec2 b2Mul22(b2Mat33 A, b2Vec2 v)
	{
		return new b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
	}

	/// Multiply two rotations: q * r
	public static b2Rot b2Mul(b2Rot q, b2Rot r)
	{
		// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
		// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
		// s = qs * rc + qc * rs
		// c = qc * rc - qs * rs
		b2Rot qr = new b2Rot();
		qr.s = q.s * r.c + q.c * r.s;
		qr.c = q.c * r.c - q.s * r.s;
		return qr;
	}

	/// Transpose multiply two rotations: qT * r
	public static b2Rot b2MulT(b2Rot q, b2Rot r)
	{
		// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
		// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
		// s = qc * rs - qs * rc
		// c = qc * rc + qs * rs
		b2Rot qr = new b2Rot();
		qr.s = q.c * r.s - q.s * r.c;
		qr.c = q.c * r.c + q.s * r.s;
		return qr;
	}

	/// Rotate a vector
	public static b2Vec2 b2Mul(b2Rot q, b2Vec2 v)
	{
		return new b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
	}

	/// Inverse rotate a vector
	public static b2Vec2 b2MulT(b2Rot q, b2Vec2 v)
	{
		return new b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
	}

	public static b2Vec2 b2Mul(b2Transform T, b2Vec2 v)
	{
		float x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
		float y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

		return new b2Vec2(x, y);
	}

	public static b2Vec2 b2MulT(b2Transform T, b2Vec2 v)
	{
		float px = v.x - T.p.x;
		float py = v.y - T.p.y;
		float x = (T.q.c * px + T.q.s * py);
		float y = (-T.q.s * px + T.q.c * py);

		return new b2Vec2(x, y);
	}

	// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
	//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
	public static b2Transform b2Mul(b2Transform A, b2Transform B)
	{
		b2Transform C = new b2Transform();
		C.q = b2Mul(A.q, B.q);


		C.p = b2Mul(A.q, B.p) + A.p;
		return C;
	}

	// v2 = A.q' * (B.q * v1 + B.p - A.p)
	//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
	public static b2Transform b2MulT(b2Transform A, b2Transform B)
	{
		b2Transform C = new b2Transform();
		C.q = b2MulT(A.q, B.q);


		C.p = b2MulT(A.q, B.p - A.p);
		return C;
	}

    public static float b2Abs(float a)
	{
		return a > 0 ? a : -a;
	}

    public static int b2Abs(int a)
    {
        return a > 0 ? a : -a;
    }

    public static b2Vec2 b2Abs(b2Vec2 a)
	{
		return new b2Vec2(b2Abs(a.x), b2Abs(a.y));
	}

	public static b2Mat22 b2Abs(b2Mat22 A)
	{
		return new b2Mat22(b2Abs(A.ex), b2Abs(A.ey));
	}

    public static float b2Min(float a, float b)
	{
		return a < b ? a : b;
	}

    public static int b2Min(int a, int b)
    {
        return a < b ? a : b;
    }

    public static b2Vec2 b2Min(b2Vec2 a, b2Vec2 b)
	{
		return new b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
	}

    public static float b2Max(float a, float b)
	{
		return a > b ? a : b;
	}

    public static int b2Max(int a, int b)
    {
        return a > b ? a : b;
    }

    public static b2Vec2 b2Max(b2Vec2 a, b2Vec2 b)
	{
		return new b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
	}

	public static float b2Clamp(float a, float low, float high)
	{
		return b2Max(low, b2Min(a, high));
	}

    public static int b2Clamp(int a, int low, int high)
    {
        return b2Max(low, b2Min(a, high));
    }

    public static b2Vec2 b2Clamp(b2Vec2 a, b2Vec2 low, b2Vec2 high)
	{
		return b2Max(low, b2Min(a, high));
	}

	public static void b2Swap<T>(ref T a, ref T b)
	{
		T tmp = a;
		a = b;
		b = tmp;
	}

	/// "Next Largest Power of 2
	/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	/// largest power of 2. For a 32-bit value:"
	public static uint b2NextPowerOfTwo(uint x)
	{
		x |= (x >> 1);
		x |= (x >> 2);
		x |= (x >> 4);
		x |= (x >> 8);
		x |= (x >> 16);
		return x + 1;
	}

	public static bool b2IsPowerOfTwo(uint x)
	{
		bool result = x > 0 && (x & (x - 1)) == 0;
		return result;
	}

	/// @file
	/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
	///

	// Collision

	/// The maximum number of contact points between two convex shapes. Do
	/// not change this value.

	/// The maximum number of vertices on a convex polygon. You cannot increase
	/// this too much because b2BlockAllocator has a maximum object size.

	/// This is used to fatten AABBs in the dynamic tree. This allows proxies
	/// to move by a small amount without triggering a tree adjustment.
	/// This is in meters.

	/// This is used to fatten AABBs in the dynamic tree. This is used to predict
	/// the future position based on the current displacement.
	/// This is a dimensionless multiplier.

	/// A small length used as a collision and constraint tolerance. Usually it is
	/// chosen to be numerically significant, but visually insignificant.

	/// A small angle used as a collision and constraint tolerance. Usually it is
	/// chosen to be numerically significant, but visually insignificant.
	
	

	/// The radius of the polygon/edge shape skin. This should not be modified. Making
	/// this smaller means polygons will have an insufficient buffer for continuous collision.
	/// Making it larger may create artifacts for vertex collision.
	
	

	/// Maximum number of sub-steps per contact in continuous physics simulation.


	// Dynamics

	/// Maximum number of contacts to be handled to solve a TOI impact.

	/// A velocity threshold for elastic collisions. Any collision with a relative linear
	/// velocity below this threshold will be treated as inelastic.

	/// The maximum linear position correction used when solving constraints. This helps to
	/// prevent overshoot.

	/// The maximum angular position correction used when solving constraints. This helps to
	/// prevent overshoot.
	
	

	/// The maximum linear velocity of a body. This limit is very large and is used
	/// to prevent numerical problems. You shouldn't need to adjust this.
	
	

	/// The maximum angular velocity of a body. This limit is very large and is used
	/// to prevent numerical problems. You shouldn't need to adjust this.
	
	
	
	

	/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
	/// that overlap is removed in one time step. However using values close to 1 often lead
	/// to overshoot.


	// Sleep

	/// The time that a body must be still before it will go to sleep.

	/// A body cannot sleep if its linear velocity is above this tolerance.

	/// A body cannot sleep if its angular velocity is above this tolerance.
	
	

    // You can modify this to use your logging facility.

	/// Logging function.
	public static void b2Log(string _string, params object[] LegacyParamArray)
	{
	    Console.Write(_string, LegacyParamArray);
	}

	/// Current version.
	public static b2Version b2_version = new b2Version(2, 3, 2);

	public static readonly int b2_stackSize = 100 * 1024; // 100k
	public static readonly int b2_maxStackEntries = 32;

	/// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
	/// For example, anything slides on ice.
	public static float b2MixFriction(float friction1, float friction2)
	{
		return (float)Math.Sqrt(friction1 * friction2);
	}

	/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
	/// For example, a superball bounces on anything.
	public static float b2MixRestitution(float restitution1, float restitution2)
	{
		return restitution1 > restitution2 ? restitution1 : restitution2;
	}

	// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.

	public static bool g_blockSolve = true;
	public static readonly float b2_minPulleyLength = 2.0f;

	public static b2ContactFilter b2_defaultFilter = new b2ContactFilter();
	public static b2ContactListener b2_defaultListener = new b2ContactListener();

}