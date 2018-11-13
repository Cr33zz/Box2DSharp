using System.Diagnostics;

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

public delegate b2Contact b2ContactCreateFcn(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB);
public delegate void b2ContactDestroyFcn(ref b2Contact contact);

public class b2ContactRegister
{
	public b2ContactCreateFcn createFcn;
	public b2ContactDestroyFcn destroyFcn;
	public bool primary;
}

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
public class b2ContactEdge
{
	public b2Body other; ///< provides quick access to the other body attached.
	public b2Contact contact; ///< the contact
	public b2ContactEdge prev; ///< the previous contact edge in the body's contact list
	public b2ContactEdge next; ///< the next contact edge in the body's contact list
}

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
public abstract class b2Contact : System.IDisposable
{

	/// Get the contact manifold. Do not modify the manifold unless you understand the
	/// internals of Box2D.
	public b2Manifold GetManifold()
	{
		return m_manifold;
	}

	/// Get the world manifold.
	public void GetWorldManifold(b2WorldManifold worldManifold)
	{
		b2Body bodyA = m_fixtureA.GetBody();
		b2Body bodyB = m_fixtureB.GetBody();
		b2Shape shapeA = m_fixtureA.GetShape();
		b2Shape shapeB = m_fixtureB.GetShape();

		worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}

	/// Is this contact touching?
	public bool IsTouching()
	{
		return (m_flags & ContactFlags.e_touchingFlag) == ContactFlags.e_touchingFlag;
	}

	/// Enable/disable this contact. This can be used inside the pre-solve
	/// contact listener. The contact is only disabled for the current
	/// time step (or sub-step in continuous collisions).
	public void SetEnabled(bool flag)
	{
		if (flag)
		{
			m_flags |= ContactFlags.e_enabledFlag;
		}
		else
		{
			m_flags &= ~ContactFlags.e_enabledFlag;
		}
	}

	/// Has this contact been disabled?
	public bool IsEnabled()
	{
		return (m_flags & ContactFlags.e_enabledFlag) == ContactFlags.e_enabledFlag;
	}

	/// Get the next contact in the world's contact list.
	public b2Contact GetNext()
	{
		return m_next;
	}

	/// Get fixture A in this contact.
	public b2Fixture GetFixtureA()
	{
		return m_fixtureA;
	}

	/// Get the child primitive index for fixture A.
	public int GetChildIndexA()
	{
		return m_indexA;
	}

	/// Get fixture B in this contact.
	public b2Fixture GetFixtureB()
	{
		return m_fixtureB;
	}

	/// Get the child primitive index for fixture B.
	public int GetChildIndexB()
	{
		return m_indexB;
	}

	/// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
	/// This value persists until set or reset.
	public void SetFriction(float friction)
	{
		m_friction = friction;
	}

	/// Get the friction.
	public float GetFriction()
	{
		return m_friction;
	}

	/// Reset the friction mixture to the default value.
	public void ResetFriction()
	{
		m_friction = Utils.b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
	}

	/// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
	/// The value persists until you set or reset.
	public void SetRestitution(float restitution)
	{
		m_restitution = restitution;
	}

	/// Get the restitution.
	public float GetRestitution()
	{
		return m_restitution;
	}

	/// Reset the restitution to the default value.
	public void ResetRestitution()
	{
		m_restitution = Utils.b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
	}

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	public void SetTangentSpeed(float speed)
	{
		m_tangentSpeed = speed;
	}

	/// Get the desired tangent speed. In meters per second.
	public float GetTangentSpeed()
	{
		return m_tangentSpeed;
	}

	/// Evaluate this contact with your own manifold and transforms.
	public abstract void Evaluate(b2Manifold manifold, b2Transform xfA, b2Transform xfB);

    // Flags stored in m_flags
    internal enum ContactFlags
	{
		// Used when crawling contact graph when forming islands.
		e_islandFlag = 0x0001,

		// Set when the shapes are touching.
		e_touchingFlag = 0x0002,

		// This contact can be disabled (by user)
		e_enabledFlag = 0x0004,

		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag = 0x0008,

		// This bullet contact had a TOI event
		e_bulletHitFlag = 0x0010,

		// This contact has a valid TOI in m_toi
		e_toiFlag = 0x0020
	}

    /// Flag this contact for filtering. Filtering will occur the next time step.
    internal void FlagForFiltering()
	{
		m_flags |= ContactFlags.e_filterFlag;
	}

	protected static void AddType(b2ContactCreateFcn createFcn, b2ContactDestroyFcn destoryFcn, b2Shape.Type type1, b2Shape.Type type2)
	{
		Debug.Assert(0 <= ((int)type1) && type1 < b2Shape.Type.e_typeCount);
		Debug.Assert(0 <= ((int)type2) && type2 < b2Shape.Type.e_typeCount);

        s_registers[(int)type1, (int)type2] = new b2ContactRegister();
        s_registers[(int)type1, (int)type2].createFcn = createFcn;
		s_registers[(int)type1, (int)type2].destroyFcn = destoryFcn;
		s_registers[(int)type1, (int)type2].primary = true;

		if (type1 != type2)
		{
            s_registers[(int)type2, (int)type1] = new b2ContactRegister();
            s_registers[(int)type2, (int)type1].createFcn = createFcn;
			s_registers[(int)type2, (int)type1].destroyFcn = destoryFcn;
			s_registers[(int)type2, (int)type1].primary = false;
		}
	}
	protected static void InitializeRegisters()
	{
		AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.Type.e_circle, b2Shape.Type.e_circle);
		AddType(b2PolygonAndCircleContact.Create, b2PolygonAndCircleContact.Destroy, b2Shape.Type.e_polygon, b2Shape.Type.e_circle);
		AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.Type.e_polygon, b2Shape.Type.e_polygon);
		AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.Type.e_edge, b2Shape.Type.e_circle);
		AddType(b2EdgeAndPolygonContact.Create, b2EdgeAndPolygonContact.Destroy, b2Shape.Type.e_edge, b2Shape.Type.e_polygon);
		AddType(b2ChainAndCircleContact.Create, b2ChainAndCircleContact.Destroy, b2Shape.Type.e_chain, b2Shape.Type.e_circle);
		AddType(b2ChainAndPolygonContact.Create, b2ChainAndPolygonContact.Destroy, b2Shape.Type.e_chain, b2Shape.Type.e_polygon);
	}
    internal static b2Contact Create(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB)
	{
		if (s_initialized == false)
		{
			InitializeRegisters();
			s_initialized = true;
		}

		b2Shape.Type type1 = fixtureA.GetType();
		b2Shape.Type type2 = fixtureB.GetType();

		Debug.Assert(0 <= ((int)type1) && type1 < b2Shape.Type.e_typeCount);
		Debug.Assert(0 <= ((int)type2) && type2 < b2Shape.Type.e_typeCount);

		b2ContactCreateFcn createFcn = s_registers[(int)type1, (int)type2].createFcn;
		if (createFcn != null)
		{
			if (s_registers[(int)type1, (int)type2].primary)
			{
				return createFcn(fixtureA, indexA, fixtureB, indexB);
			}
			else
			{
				return createFcn(fixtureB, indexB, fixtureA, indexA);
			}
		}
		else
		{
			return null;
		}
	}

    internal static void Destroy(ref b2Contact contact)
	{
		Debug.Assert(s_initialized == true);

		b2Fixture fixtureA = contact.m_fixtureA;
		b2Fixture fixtureB = contact.m_fixtureB;

		if (contact.m_manifold.pointCount > 0 && fixtureA.IsSensor() == false && fixtureB.IsSensor() == false)
		{
			fixtureA.GetBody().SetAwake(true);
			fixtureB.GetBody().SetAwake(true);
		}

		b2Shape.Type typeA = fixtureA.GetType();
		b2Shape.Type typeB = fixtureB.GetType();

		Debug.Assert(0 <= ((int)typeA) && typeB < b2Shape.Type.e_typeCount);
		Debug.Assert(0 <= ((int)typeA) && typeB < b2Shape.Type.e_typeCount);

		b2ContactDestroyFcn destroyFcn = s_registers[(int)typeA, (int)typeB].destroyFcn;
		destroyFcn(ref contact);
	}

	protected b2Contact()
	{
		this.m_fixtureA = null;
		this.m_fixtureB = null;
	}
	protected b2Contact(b2Fixture fA, int indexA, b2Fixture fB, int indexB)
	{
		m_flags = ContactFlags.e_enabledFlag;

		m_fixtureA = fA;
		m_fixtureB = fB;

		m_indexA = indexA;
		m_indexB = indexB;

		m_manifold.pointCount = 0;

		m_prev = null;
		m_next = null;

		m_nodeA.contact = null;
		m_nodeA.prev = null;
		m_nodeA.next = null;
		m_nodeA.other = null;

		m_nodeB.contact = null;
		m_nodeB.prev = null;
		m_nodeB.next = null;
		m_nodeB.other = null;

		m_toiCount = 0;

		m_friction = Utils.b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
		m_restitution = Utils.b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);

		m_tangentSpeed = 0.0f;
	}
	public virtual void Dispose()
	{
	}


    // Update the contact manifold and touching status.
    // Note: do not assume the fixture AABBs are overlapping or are valid.
    internal void Update(b2ContactListener listener)
	{
		b2Manifold oldManifold = m_manifold;

		// Re-enable this contact.
		m_flags |= ContactFlags.e_enabledFlag;

		bool touching = false;
		bool wasTouching = (m_flags & ContactFlags.e_touchingFlag) == ContactFlags.e_touchingFlag;

		bool sensorA = m_fixtureA.IsSensor();
		bool sensorB = m_fixtureB.IsSensor();
		bool sensor = sensorA || sensorB;

		b2Body bodyA = m_fixtureA.GetBody();
		b2Body bodyB = m_fixtureB.GetBody();
		b2Transform xfA = bodyA.GetTransform();
		b2Transform xfB = bodyB.GetTransform();

		// Is this contact a sensor?
		if (sensor)
		{
			b2Shape shapeA = m_fixtureA.GetShape();
			b2Shape shapeB = m_fixtureB.GetShape();
			touching = Utils.b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

			// Sensors don't generate manifolds.
			m_manifold.pointCount = 0;
		}
		else
		{
			Evaluate(m_manifold, xfA, xfB);
			touching = m_manifold.pointCount > 0;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (int i = 0; i < m_manifold.pointCount; ++i)
			{
				b2ManifoldPoint mp2 = m_manifold.points[i];
				mp2.normalImpulse = 0.0f;
				mp2.tangentImpulse = 0.0f;
				b2ContactID id2 = mp2.id;

				for (int j = 0; j < oldManifold.pointCount; ++j)
				{
					b2ManifoldPoint mp1 = oldManifold.points[j];

					if (mp1.id.key == id2.key)
					{
						mp2.normalImpulse = mp1.normalImpulse;
						mp2.tangentImpulse = mp1.tangentImpulse;
						break;
					}
				}
			}

			if (touching != wasTouching)
			{
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}

		if (touching)
		{
			m_flags |= ContactFlags.e_touchingFlag;
		}
		else
		{
			m_flags &= ~ContactFlags.e_touchingFlag;
		}

		if (wasTouching == false && touching == true && listener != null)
		{
			listener.BeginContact(this);
		}

		if (wasTouching == true && touching == false && listener != null)
		{
			listener.EndContact(this);
		}

		if (sensor == false && touching && listener != null)
		{
			listener.PreSolve(this, oldManifold);
		}
	}

	protected static b2ContactRegister[,] s_registers = new b2ContactRegister[(int)b2Shape.Type.e_typeCount, (int)b2Shape.Type.e_typeCount];
    protected static bool s_initialized = false;

    internal ContactFlags m_flags;

    // World pool and list pointers.
    internal b2Contact m_prev;
    internal b2Contact m_next;

    // Nodes for connecting bodies.
    internal b2ContactEdge m_nodeA = new b2ContactEdge();
    internal b2ContactEdge m_nodeB = new b2ContactEdge();

	internal b2Fixture m_fixtureA;
    internal b2Fixture m_fixtureB;

    internal int m_indexA;
    internal int m_indexB;

    internal b2Manifold m_manifold = new b2Manifold();

    internal int m_toiCount;
    internal float m_toi;

    internal float m_friction;
    internal float m_restitution;

    internal float m_tangentSpeed;
}




