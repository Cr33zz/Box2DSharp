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

/// This holds the mass data computed for a shape.
public class b2MassData
{
	/// The mass of the shape, usually in kilograms.
	public float mass;

	/// The position of the shape's centroid relative to the shape's origin.
	public b2Vec2 center = new b2Vec2();

	/// The rotational inertia of the shape about the local origin.
	public float I;
}

/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.
public abstract class b2Shape : System.IDisposable
{

	public enum Type
	{
		e_circle = 0,
		e_edge = 1,
		e_polygon = 2,
		e_chain = 3,
		e_typeCount = 4
	}

	public virtual void Dispose()
	{
	}

	/// Clone the concrete shape using the provided allocator.
	public abstract b2Shape Clone();

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	public b2Shape.Type GetType()
	{
		return m_type;
	}

	/// Get the number of child primitives.
	public abstract int GetChildCount();

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	public abstract bool TestPoint(b2Transform xf, b2Vec2 p);

	/// Cast a ray against a child shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	/// @param transform the transform to be applied to the shape.
	/// @param childIndex the child shape index
	public abstract bool RayCast(b2RayCastOutput output, b2RayCastInput input, b2Transform transform, int childIndex);

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	public abstract void ComputeAABB(b2AABB aabb, b2Transform xf, int childIndex);

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @param massData returns the mass data for this shape.
	/// @param density the density in kilograms per meter squared.
	public abstract void ComputeMass(b2MassData massData, float density);

	public Type m_type;

	/// Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
	/// making rounded polygons.
	public float m_radius;
}

