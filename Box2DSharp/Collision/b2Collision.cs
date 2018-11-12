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

using System.Runtime.InteropServices;

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
public struct b2ContactFeature
{
    public b2ContactFeature(b2ContactFeature other)
    {
        indexA = other.indexA;
        indexB = other.indexB;
        typeA = other.typeA;
        typeB = other.typeB;
    }

    public enum Type
	{
		e_vertex = 0,
		e_face = 1
	}

	public byte indexA; ///< Feature index on shapeA
	public byte indexB; ///< Feature index on shapeB
	public byte typeA; ///< The feature type on shapeA
	public byte typeB; ///< The feature type on shapeB
}

/// Contact ids to facilitate warm starting.
[StructLayout(LayoutKind.Explicit, Size=4)]
public struct b2ContactID
{
    [FieldOffset(0)]
    public b2ContactFeature cf;

    [FieldOffset(0)]
    public uint key; ///< Used to quickly compare contact ids.
};

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
public class b2ManifoldPoint
{
	public b2Vec2 localPoint = new b2Vec2(); ///< usage depends on manifold type
	public float normalImpulse; ///< the non-penetration impulse
	public float tangentImpulse; ///< the friction impulse
	public b2ContactID id = new b2ContactID(); ///< uniquely identifies a contact point between two shapes
}

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
public class b2Manifold
{
	public enum Type
	{
		e_circles,
		e_faceA,
		e_faceB
	}

	public b2ManifoldPoint[] points = Arrays.InitializeWithDefaultInstances<b2ManifoldPoint>(DefineConstants.b2_maxManifoldPoints); ///< the points of contact
	public b2Vec2 localNormal = new b2Vec2(); ///< not use for Type::e_points
	public b2Vec2 localPoint = new b2Vec2(); ///< usage depends on manifold type
	public Type type;
	public int pointCount; ///< the number of manifold points
}

/// This is used to compute the current state of a contact manifold.
public class b2WorldManifold
{
	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
	public void Initialize(b2Manifold manifold, b2Transform xfA, float radiusA, b2Transform xfB, float radiusB)
	{
		if (manifold.pointCount == 0)
		{
			return;
		}

		switch (manifold.type)
		{
		case b2Manifold.Type.e_circles:
		{
				normal.Set(1.0f, 0.0f);
				b2Vec2 pointA = GlobalMembers.b2Mul(xfA, manifold.localPoint);
				b2Vec2 pointB = GlobalMembers.b2Mul(xfB, manifold.points[0].localPoint);
				if (GlobalMembers.b2DistanceSquared(pointA, pointB) > float.Epsilon * float.Epsilon)
				{
					normal = pointB - pointA;
					normal.Normalize();
				}

				b2Vec2 cA = pointA + radiusA * normal;
				b2Vec2 cB = pointB - radiusB * normal;
				points[0] = 0.5f * (cA + cB);
				separations[0] = GlobalMembers.b2Dot(cB - cA, normal);
		}
			break;

		case b2Manifold.Type.e_faceA:
		{
				normal = GlobalMembers.b2Mul(xfA.q, manifold.localNormal);
				b2Vec2 planePoint = GlobalMembers.b2Mul(xfA, manifold.localPoint);

				for (int i = 0; i < manifold.pointCount; ++i)
				{
					b2Vec2 clipPoint = GlobalMembers.b2Mul(xfB, manifold.points[i].localPoint);
					b2Vec2 cA = clipPoint + (radiusA - GlobalMembers.b2Dot(clipPoint - planePoint, normal)) * normal;
					b2Vec2 cB = clipPoint - radiusB * normal;
					points[i] = 0.5f * (cA + cB);
					separations[i] = GlobalMembers.b2Dot(cB - cA, normal);
				}
		}
			break;

		case b2Manifold.Type.e_faceB:
		{
			normal = GlobalMembers.b2Mul(xfB.q, manifold.localNormal);
			b2Vec2 planePoint = GlobalMembers.b2Mul(xfB, manifold.localPoint);

			for (int i = 0; i < manifold.pointCount; ++i)
			{
				b2Vec2 clipPoint = GlobalMembers.b2Mul(xfA, manifold.points[i].localPoint);
				b2Vec2 cB = clipPoint + (radiusB - GlobalMembers.b2Dot(clipPoint - planePoint, normal)) * normal;
				b2Vec2 cA = clipPoint - radiusA * normal;
				points[i] = 0.5f * (cA + cB);
				separations[i] = GlobalMembers.b2Dot(cA - cB, normal);
			}

			// Ensure normal points from A to B.
			normal = -normal;
		}
			break;
		}
	}

	public b2Vec2 normal = new b2Vec2(); ///< world vector pointing from A to B
	public b2Vec2[] points = Arrays.InitializeWithDefaultInstances<b2Vec2>(DefineConstants.b2_maxManifoldPoints); ///< world contact point (point of intersection)
	public float[] separations = Arrays.InitializeWithDefaultInstances<float>(DefineConstants.b2_maxManifoldPoints); ///< a negative value indicates overlap, in meters
}

/// This is used for determining the state of contact points.
public enum PointState
{
	nullState, /// point does not exist
	addState, /// point was added in the update
	persistState, /// point persisted across the update
	removeState /// point was removed in the update
}

/// Used for computing contact manifolds.
public class b2ClipVertex
{
	public b2Vec2 v = new b2Vec2();
	public b2ContactID id;
}

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
public class b2RayCastInput
{
	public b2Vec2 p1 = new b2Vec2();
	public b2Vec2 p2 = new b2Vec2();
	public float maxFraction;
}

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
public class b2RayCastOutput
{
	public b2Vec2 normal = new b2Vec2();
	public float fraction;
}

/// An axis aligned bounding box.
public struct b2AABB
{
	/// Verify that the bounds are sorted.
	public bool IsValid()
	{
		b2Vec2 d = upperBound - lowerBound;
		bool valid = d.x >= 0.0f && d.y >= 0.0f;
		valid = valid && lowerBound.IsValid() && upperBound.IsValid();
		return valid;
	}

	/// Get the center of the AABB.
	public b2Vec2 GetCenter()
	{
		return 0.5f * (lowerBound + upperBound);
	}

	/// Get the extents of the AABB (half-widths).
	public b2Vec2 GetExtents()
	{
		return 0.5f * (upperBound - lowerBound);
	}

	/// Get the perimeter length
	public float GetPerimeter()
	{
		float wx = upperBound.x - lowerBound.x;
		float wy = upperBound.y - lowerBound.y;
		return 2.0f * (wx + wy);
	}

	/// Combine an AABB into this one.
	public void Combine(b2AABB aabb)
	{
		lowerBound = GlobalMembers.b2Min(lowerBound, aabb.lowerBound);
		upperBound = GlobalMembers.b2Max(upperBound, aabb.upperBound);
	}

	/// Combine two AABBs into this one.
	public void Combine(b2AABB aabb1, b2AABB aabb2)
	{
		lowerBound = GlobalMembers.b2Min(aabb1.lowerBound, aabb2.lowerBound);
		upperBound = GlobalMembers.b2Max(aabb1.upperBound, aabb2.upperBound);
	}

	/// Does this aabb contain the provided AABB.
	public bool Contains(b2AABB aabb)
	{
		bool result = true;
		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;
		return result;
	}


	// From Real-time Collision Detection, p179.
	public bool RayCast(b2RayCastOutput output, b2RayCastInput input)
	{
		float tmin = -float.MaxValue;
		float tmax = float.MaxValue;

		b2Vec2 p = new b2Vec2(input.p1);
		b2Vec2 d = input.p2 - input.p1;
		b2Vec2 absD = GlobalMembers.b2Abs(d);

		b2Vec2 normal = new b2Vec2();

		for (int i = 0; i < 2; ++i)
		{
			if (absD[i] < float.Epsilon)
			{
				// Parallel.
				if (p[i] < lowerBound[i] || upperBound[i] < p[i])
				{
					return false;
				}
			}
			else
			{
				float inv_d = 1.0f / d[i];
				float t1 = (lowerBound[i] - p[i]) * inv_d;
				float t2 = (upperBound[i] - p[i]) * inv_d;

				// Sign of the normal vector.
				float s = -1.0f;

				if (t1 > t2)
				{
					GlobalMembers.b2Swap(ref t1, ref t2);
					s = 1.0f;
				}

				// Push the min up
				if (t1 > tmin)
				{
					normal.SetZero();
					normal[i] = s;
					tmin = t1;
				}

				// Pull the max down
				tmax = GlobalMembers.b2Min(tmax, t2);

				if (tmin > tmax)
				{
					return false;
				}
			}
		}

		// Does the ray start inside the box?
		// Does the ray intersect beyond the max fraction?
		if (tmin < 0.0f || input.maxFraction < tmin)
		{
			return false;
		}

		// Intersection.
		output.fraction = tmin;


		output.normal = normal;
		return true;
	}

	public b2Vec2 lowerBound; ///< the lower vertex
	public b2Vec2 upperBound; ///< the upper vertex
}