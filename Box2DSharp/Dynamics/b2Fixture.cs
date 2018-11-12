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
using System.Diagnostics;

/// This holds contact filtering data.
public class b2Filter
{
	public b2Filter()
	{
		categoryBits = 0x0001;
		maskBits = 0xFFFF;
		groupIndex = 0;
	}

	/// The collision category bits. Normally you would just set one bit.
	public ushort categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	public ushort maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	public short groupIndex;
}

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
public class b2FixtureDef
{
	/// The constructor sets the default fixture definition values.
	public b2FixtureDef()
	{
		shape = null;
		userData = null;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		isSensor = false;
	}

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	public b2Shape shape;

	/// Use this to store application specific fixture data.
	public object userData;

	/// The friction coefficient, usually in the range [0,1].
	public float friction;

	/// The restitution (elasticity) usually in the range [0,1].
	public float restitution;

	/// The density, usually in kg/m^2.
	public float density;

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	public bool isSensor;

	/// Contact filtering data.
	public b2Filter filter = new b2Filter();
}

/// This proxy is used internally to connect fixtures to the broad-phase.
public class b2FixtureProxy
{
	public b2AABB aabb = new b2AABB();
	public b2Fixture fixture;
	public int childIndex;
	public int proxyId;
}

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
public class b2Fixture
{
	/// Get the type of the child shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	public new b2Shape.Type GetType()
	{
		return m_shape.GetType();
	}

	/// Get the child shape. You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	public b2Shape GetShape()
	{
		return m_shape;
	}

	/// Set if this fixture is a sensor.
	public void SetSensor(bool sensor)
	{
		if (sensor != m_isSensor)
		{
			m_body.SetAwake(true);
			m_isSensor = sensor;
		}
	}

	/// Is this fixture a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	public bool IsSensor()
	{
		return m_isSensor;
	}

	/// Set the contact filtering data. This will not update contacts until the next time
	/// step when either parent body is active and awake.
	/// This automatically calls Refilter.
	public void SetFilterData(b2Filter filter)
	{
		m_filter = filter;

		Refilter();
	}

	/// Get the contact filtering data.
	public b2Filter GetFilterData()
	{
		return m_filter;
	}

	/// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
	public void Refilter()
	{
		if (m_body == null)
		{
			return;
		}

		// Flag associated contacts for filtering.
		b2ContactEdge edge = m_body.GetContactList();
		while (edge != null)
		{
			b2Contact contact = edge.contact;
			b2Fixture fixtureA = contact.GetFixtureA();
			b2Fixture fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
			{
				contact.FlagForFiltering();
			}

			edge = edge.next;
		}

		b2World world = m_body.GetWorld();

		if (world == null)
		{
			return;
		}

		// Touch each proxy so that new pairs may be created
		b2BroadPhase broadPhase = world.m_contactManager.m_broadPhase;
		for (int i = 0; i < m_proxyCount; ++i)
		{
			broadPhase.TouchProxy(m_proxies[i].proxyId);
		}
	}

	/// Get the parent body of this fixture. This is nullptr if the fixture is not attached.
	/// @return the parent body.
	public b2Body GetBody()
	{
		return m_body;
	}

	/// Get the next fixture in the parent body's fixture list.
	/// @return the next shape.
	public b2Fixture GetNext()
	{
		return m_next;
	}

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	public object GetUserData()
	{
		return m_userData;
	}

	/// Set the user data. Use this to store your application specific data.
	public void SetUserData(object data)
	{
		m_userData = data;
	}

	/// Test a point for containment in this fixture.
	/// @param p a point in world coordinates.
	public bool TestPoint(b2Vec2 p)
	{
		return m_shape.TestPoint(m_body.GetTransform(), p);
	}

	/// Cast a ray against this shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	public bool RayCast(b2RayCastOutput output, b2RayCastInput input, int childIndex)
	{
		return m_shape.RayCast(output, input, m_body.GetTransform(), childIndex);
	}

	/// Get the mass data for this fixture. The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	public void GetMassData(b2MassData massData)
	{
		m_shape.ComputeMass(massData, m_density);
	}

	/// Set the density of this fixture. This will _not_ automatically adjust the mass
	/// of the body. You must call b2Body::ResetMassData to update the body's mass.
	public void SetDensity(float density)
	{
		Debug.Assert(GlobalMembers.b2IsValid(density) && density >= 0.0f);
		m_density = density;
	}

	/// Get the density of this fixture.


	public float GetDensity()
	{
		return m_density;
	}

	/// Get the coefficient of friction.
	public float GetFriction()
	{
		return m_friction;
	}

	/// Set the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	public void SetFriction(float friction)
	{
		m_friction = friction;
	}

	/// Get the coefficient of restitution.
	public float GetRestitution()
	{
		return m_restitution;
	}

	/// Set the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	public void SetRestitution(float restitution)
	{
		m_restitution = restitution;
	}

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	public b2AABB GetAABB(int childIndex)
	{
		Debug.Assert(0 <= childIndex && childIndex < m_proxyCount);
		return m_proxies[childIndex].aabb;
	}

	/// Dump this fixture to the log file.
	public void Dump(int bodyIndex)
	{
		GlobalMembers.b2Log("    b2FixtureDef fd;\n");
		GlobalMembers.b2Log("    fd.friction = %.15lef;\n", m_friction);
		GlobalMembers.b2Log("    fd.restitution = %.15lef;\n", m_restitution);
		GlobalMembers.b2Log("    fd.density = %.15lef;\n", m_density);
		GlobalMembers.b2Log("    fd.isSensor = bool(%d);\n", m_isSensor);
		GlobalMembers.b2Log("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
		GlobalMembers.b2Log("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
		GlobalMembers.b2Log("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);

		switch (m_shape.m_type)
		{
		case b2Shape.Type.e_circle:
		{
				b2CircleShape s = (b2CircleShape)m_shape;
				GlobalMembers.b2Log("    b2CircleShape shape;\n");
				GlobalMembers.b2Log("    shape.m_radius = %.15lef;\n", s.m_radius);
				GlobalMembers.b2Log("    shape.m_p.Set(%.15lef, %.15lef);\n", s.m_p.x, s.m_p.y);
		}
			break;

		case b2Shape.Type.e_edge:
		{
				b2EdgeShape s = (b2EdgeShape)m_shape;
				GlobalMembers.b2Log("    b2EdgeShape shape;\n");
				GlobalMembers.b2Log("    shape.m_radius = %.15lef;\n", s.m_radius);
				GlobalMembers.b2Log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s.m_vertex0.x, s.m_vertex0.y);
				GlobalMembers.b2Log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s.m_vertex1.x, s.m_vertex1.y);
				GlobalMembers.b2Log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s.m_vertex2.x, s.m_vertex2.y);
				GlobalMembers.b2Log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s.m_vertex3.x, s.m_vertex3.y);
				GlobalMembers.b2Log("    shape.m_hasVertex0 = bool(%d);\n", s.m_hasVertex0);
				GlobalMembers.b2Log("    shape.m_hasVertex3 = bool(%d);\n", s.m_hasVertex3);
		}
			break;

		case b2Shape.Type.e_polygon:
		{
				b2PolygonShape s = (b2PolygonShape)m_shape;
				GlobalMembers.b2Log("    b2PolygonShape shape;\n");
				GlobalMembers.b2Log("    b2Vec2 vs[%d];\n", DefineConstants.b2_maxPolygonVertices);
				for (int i = 0; i < s.m_count; ++i)
				{
					GlobalMembers.b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].x, s.m_vertices[i].y);
				}
				GlobalMembers.b2Log("    shape.Set(vs, %d);\n", s.m_count);
		}
			break;

		case b2Shape.Type.e_chain:
		{
				b2ChainShape s = (b2ChainShape)m_shape;
				GlobalMembers.b2Log("    b2ChainShape shape;\n");
				GlobalMembers.b2Log("    b2Vec2 vs[%d];\n", s.m_count);
				for (int i = 0; i < s.m_count; ++i)
				{
					GlobalMembers.b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].x, s.m_vertices[i].y);
				}
				GlobalMembers.b2Log("    shape.CreateChain(vs, %d);\n", s.m_count);
				GlobalMembers.b2Log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s.m_prevVertex.x, s.m_prevVertex.y);
				GlobalMembers.b2Log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s.m_nextVertex.x, s.m_nextVertex.y);
				GlobalMembers.b2Log("    shape.m_hasPrevVertex = bool(%d);\n", s.m_hasPrevVertex);
				GlobalMembers.b2Log("    shape.m_hasNextVertex = bool(%d);\n", s.m_hasNextVertex);
		}
			break;

		default:
			return;
		}

		GlobalMembers.b2Log("\n");
		GlobalMembers.b2Log("    fd.shape = &shape;\n");
		GlobalMembers.b2Log("\n");
		GlobalMembers.b2Log("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
	}

    internal b2Fixture()
	{
		m_userData = null;
		m_body = null;
		m_next = null;
		m_proxies = null;
		m_proxyCount = 0;
		m_shape = null;
		m_density = 0.0f;
	}

    // We need separation create/destroy functions from the constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments allowed by C++).
    internal void Create(b2Body body, b2FixtureDef def)
	{
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;

		m_body = body;
		m_next = null;

		m_filter = def.filter;

		m_isSensor = def.isSensor;

		m_shape = def.shape.Clone();

		// Reserve proxy space
		int childCount = m_shape.GetChildCount();
		m_proxies = Arrays.InitializeWithDefaultInstances<b2FixtureProxy>(childCount);
		for (int i = 0; i < childCount; ++i)
		{
            m_proxies[i] = new b2FixtureProxy();
            m_proxies[i].fixture = null;
			m_proxies[i].proxyId = (int)b2BroadPhase.AnonymousEnum.e_nullProxy;
		}
		m_proxyCount = 0;

		m_density = def.density;
	}
    internal void Destroy()
	{
		// The proxies must be destroyed before calling this.
		Debug.Assert(m_proxyCount == 0);

		// Free the proxy array.
		m_proxies = null;
		m_shape = null;
	}

	// These support body activation/deactivation.
	internal void CreateProxies(b2BroadPhase broadPhase, b2Transform xf)
	{
		Debug.Assert(m_proxyCount == 0);

		// Create proxies in the broad-phase.
		m_proxyCount = m_shape.GetChildCount();

		for (int i = 0; i < m_proxyCount; ++i)
		{
			b2FixtureProxy proxy = m_proxies[i];
			m_shape.ComputeAABB(proxy.aabb, xf, i);
			proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, proxy);
			proxy.fixture = this;
			proxy.childIndex = i;
		}
	}
    internal void DestroyProxies(b2BroadPhase broadPhase)
	{
		// Destroy proxies in the broad-phase.
		for (int i = 0; i < m_proxyCount; ++i)
		{
			b2FixtureProxy proxy = m_proxies[i];
			broadPhase.DestroyProxy(proxy.proxyId);
			proxy.proxyId = (int)b2BroadPhase.AnonymousEnum.e_nullProxy;
		}

		m_proxyCount = 0;
	}

    internal void Synchronize(b2BroadPhase broadPhase, b2Transform transform1, b2Transform transform2)
	{
		if (m_proxyCount == 0)
		{
			return;
		}

		for (int i = 0; i < m_proxyCount; ++i)
		{
			b2FixtureProxy proxy = m_proxies[i];

			// Compute an AABB that covers the swept shape (may miss some rotation effect).
			b2AABB aabb1 = new b2AABB();
			b2AABB aabb2 = new b2AABB();
			m_shape.ComputeAABB(aabb1, transform1, proxy.childIndex);
			m_shape.ComputeAABB(aabb2, transform2, proxy.childIndex);

			proxy.aabb.Combine(aabb1, aabb2);

			b2Vec2 displacement = transform2.p - transform1.p;

			broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
		}
	}

    internal float m_density;

	internal b2Fixture m_next;
    internal b2Body m_body;

    internal b2Shape m_shape;

	internal float m_friction;
	internal float m_restitution;

	internal b2FixtureProxy[] m_proxies;
    internal int m_proxyCount;

    internal b2Filter m_filter = new b2Filter();

    internal bool m_isSensor;

    internal object m_userData;
}


