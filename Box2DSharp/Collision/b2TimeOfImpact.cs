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

/// Input parameters for b2TimeOfImpact
public class b2TOIInput
{
	public b2DistanceProxy proxyA = new b2DistanceProxy();
	public b2DistanceProxy proxyB = new b2DistanceProxy();
	public b2Sweep sweepA = new b2Sweep();
	public b2Sweep sweepB = new b2Sweep();
	public float tMax; // defines sweep interval [0, tMax]
}

/// Output parameters for b2TimeOfImpact.
public class b2TOIOutput
{
	public enum State
	{
		e_unknown,
		e_failed,
		e_overlapped,
		e_touching,
		e_separated
	}

	public State state;
	public float t;
}

//
public class b2SeparationFunction
{
	public enum Type
	{
		e_points,
		e_faceA,
		e_faceB
	}

	// TODO_ERIN might not need to return the separation

	public float Initialize(b2SimplexCache cache, b2DistanceProxy proxyA, b2Sweep sweepA, b2DistanceProxy proxyB, b2Sweep sweepB, float t1)
	{
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		int count = cache.count;
		Debug.Assert(0 < count && count < 3);

		m_sweepA = sweepA;
		m_sweepB = sweepB;

		b2Transform xfA = new b2Transform();
		b2Transform xfB = new b2Transform();
		m_sweepA.GetTransform(xfA, t1);
		m_sweepB.GetTransform(xfB, t1);

		if (count == 1)
		{
			m_type = Type.e_points;
			b2Vec2 localPointA = m_proxyA.GetVertex(cache.indexA[0]);
			b2Vec2 localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			b2Vec2 pointA = Utils.b2Mul(xfA, localPointA);
			b2Vec2 pointB = Utils.b2Mul(xfB, localPointB);
			m_axis = pointB - pointA;
			float s = m_axis.Normalize();
			return s;
		}
		else if (cache.indexA[0] == cache.indexA[1])
		{
			// Two points on B and one on A.
			m_type = Type.e_faceB;
			b2Vec2 localPointB1 = proxyB.GetVertex(cache.indexB[0]);
			b2Vec2 localPointB2 = proxyB.GetVertex(cache.indexB[1]);

			m_axis = Utils.b2Cross(localPointB2 - localPointB1, 1.0f);
			m_axis.Normalize();
			b2Vec2 normal = Utils.b2Mul(xfB.q, m_axis);

			m_localPoint = 0.5f * (localPointB1 + localPointB2);
			b2Vec2 pointB = Utils.b2Mul(xfB, m_localPoint);

			b2Vec2 localPointA = proxyA.GetVertex(cache.indexA[0]);
			b2Vec2 pointA = Utils.b2Mul(xfA, localPointA);

			float s = Utils.b2Dot(pointA - pointB, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
				s = -s;
			}
			return s;
		}
		else
		{
			// Two points on A and one or two points on B.
			m_type = Type.e_faceA;
			b2Vec2 localPointA1 = m_proxyA.GetVertex(cache.indexA[0]);
			b2Vec2 localPointA2 = m_proxyA.GetVertex(cache.indexA[1]);

			m_axis = Utils.b2Cross(localPointA2 - localPointA1, 1.0f);
			m_axis.Normalize();
			b2Vec2 normal = Utils.b2Mul(xfA.q, m_axis);

			m_localPoint = 0.5f * (localPointA1 + localPointA2);
			b2Vec2 pointA = Utils.b2Mul(xfA, m_localPoint);

			b2Vec2 localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			b2Vec2 pointB = Utils.b2Mul(xfB, localPointB);

			float s = Utils.b2Dot(pointB - pointA, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
				s = -s;
			}
			return s;
		}
	}

	//
	public float FindMinSeparation(ref int indexA, ref int indexB, float t)
	{
		b2Transform xfA = new b2Transform();
		b2Transform xfB = new b2Transform();
		m_sweepA.GetTransform(xfA, t);
		m_sweepB.GetTransform(xfB, t);

		switch (m_type)
		{
		case Type.e_points:
		{
				b2Vec2 axisA = Utils.b2MulT(xfA.q, m_axis);
				b2Vec2 axisB = Utils.b2MulT(xfB.q, -m_axis);

				indexA = m_proxyA.GetSupport(axisA);
				indexB = m_proxyB.GetSupport(axisB);

				b2Vec2 localPointA = m_proxyA.GetVertex(indexA);
				b2Vec2 localPointB = m_proxyB.GetVertex(indexB);

				b2Vec2 pointA = Utils.b2Mul(xfA, localPointA);
				b2Vec2 pointB = Utils.b2Mul(xfB, localPointB);

				float separation = Utils.b2Dot(pointB - pointA, m_axis);
				return separation;
		}

		case Type.e_faceA:
		{
				b2Vec2 normal = Utils.b2Mul(xfA.q, m_axis);
				b2Vec2 pointA = Utils.b2Mul(xfA, m_localPoint);

				b2Vec2 axisB = Utils.b2MulT(xfB.q, -normal);

				indexA = -1;
				indexB = m_proxyB.GetSupport(axisB);

				b2Vec2 localPointB = m_proxyB.GetVertex(indexB);
				b2Vec2 pointB = Utils.b2Mul(xfB, localPointB);

				float separation = Utils.b2Dot(pointB - pointA, normal);
				return separation;
		}

		case Type.e_faceB:
		{
				b2Vec2 normal = Utils.b2Mul(xfB.q, m_axis);
				b2Vec2 pointB = Utils.b2Mul(xfB, m_localPoint);

				b2Vec2 axisA = Utils.b2MulT(xfA.q, -normal);

				indexB = -1;
				indexA = m_proxyA.GetSupport(axisA);

				b2Vec2 localPointA = m_proxyA.GetVertex(indexA);
				b2Vec2 pointA = Utils.b2Mul(xfA, localPointA);

				float separation = Utils.b2Dot(pointA - pointB, normal);
				return separation;
		}

		default:
			Debug.Assert(false);
			indexA = -1;
			indexB = -1;
			return 0.0f;
		}
	}

	//
	public float Evaluate(int indexA, int indexB, float t)
	{
		b2Transform xfA = new b2Transform();
		b2Transform xfB = new b2Transform();
		m_sweepA.GetTransform(xfA, t);
		m_sweepB.GetTransform(xfB, t);

		switch (m_type)
		{
		case Type.e_points:
		{
				b2Vec2 localPointA = m_proxyA.GetVertex(indexA);
				b2Vec2 localPointB = m_proxyB.GetVertex(indexB);

				b2Vec2 pointA = Utils.b2Mul(xfA, localPointA);
				b2Vec2 pointB = Utils.b2Mul(xfB, localPointB);
				float separation = Utils.b2Dot(pointB - pointA, m_axis);

				return separation;
		}

		case Type.e_faceA:
		{
				b2Vec2 normal = Utils.b2Mul(xfA.q, m_axis);
				b2Vec2 pointA = Utils.b2Mul(xfA, m_localPoint);

				b2Vec2 localPointB = m_proxyB.GetVertex(indexB);
				b2Vec2 pointB = Utils.b2Mul(xfB, localPointB);

				float separation = Utils.b2Dot(pointB - pointA, normal);
				return separation;
		}

		case Type.e_faceB:
		{
				b2Vec2 normal = Utils.b2Mul(xfB.q, m_axis);
				b2Vec2 pointB = Utils.b2Mul(xfB, m_localPoint);

				b2Vec2 localPointA = m_proxyA.GetVertex(indexA);
				b2Vec2 pointA = Utils.b2Mul(xfA, localPointA);

				float separation = Utils.b2Dot(pointA - pointB, normal);
				return separation;
		}

		default:
			Debug.Assert(false);
			return 0.0f;
		}
	}

	public b2DistanceProxy m_proxyA;
	public b2DistanceProxy m_proxyB;
	public b2Sweep m_sweepA = new b2Sweep();
	public b2Sweep m_sweepB = new b2Sweep();
	public Type m_type;
	public b2Vec2 m_localPoint = new b2Vec2();
	public b2Vec2 m_axis = new b2Vec2();
}
