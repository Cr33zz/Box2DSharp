using System;
using System.Diagnostics;

/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
public enum BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody,
	b2_dynamicBody

	// TODO_ERIN
	//b2_bulletBody,
}

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
public class b2BodyDef
{
	/// This constructor sets the body definition default values.
	public b2BodyDef()
	{
		userData = null;
		position.Set(0.0f, 0.0f);
		angle = 0.0f;
		linearVelocity.Set(0.0f, 0.0f);
		angularVelocity = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		allowSleep = true;
		awake = true;
		fixedRotation = false;
		bullet = false;
		type = BodyType.b2_staticBody;
		active = true;
		gravityScale = 1.0f;
	}

	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	public BodyType type;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	public b2Vec2 position = new b2Vec2();

	/// The world angle of the body in radians.
	public float angle;

	/// The linear velocity of the body's origin in world co-ordinates.
	public b2Vec2 linearVelocity = new b2Vec2();

	/// The angular velocity of the body.
	public float angularVelocity;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	public float linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	public float angularDamping;

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	public bool allowSleep;

	/// Is this body initially awake or sleeping?
	public bool awake;

	/// Should this body be prevented from rotating? Useful for characters.
	public bool fixedRotation;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	public bool bullet;

	/// Does this body start out active?
	public bool active;

	/// Use this to store application specific body data.
	public object userData;

	/// Scale the gravity applied to this body.
	public float gravityScale;
}

/// A rigid body. These are created via b2World::CreateBody.
public class b2Body : System.IDisposable
{
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	public b2Fixture CreateFixture(b2FixtureDef def)
	{
		Debug.Assert(m_world.IsLocked() == false);
		if (m_world.IsLocked() == true)
		{
			return null;
		}

		b2Fixture fixture = new b2Fixture();
        fixture.Create(this, def);

		if ((m_flags & BodyFlags.e_activeFlag) != 0)
		{
			b2BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			fixture.CreateProxies(broadPhase, m_xf);
		}

		fixture.m_next = m_fixtureList;
		m_fixtureList = fixture;
		++m_fixtureCount;

		fixture.m_body = this;

		// Adjust mass properties if needed.
		if (fixture.m_density > 0.0f)
		{
			ResetMassData();
		}

		// Let the world know we have a new fixture. This will cause new contacts
		// to be created at the beginning of the next time step.
		m_world.m_flags |= b2World.WorldFlags.e_newFixture;

		return fixture;
	}

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use b2FixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// @param shape the shape to be cloned.
	/// @param density the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	public b2Fixture CreateFixture(b2Shape shape, float density)
	{
		b2FixtureDef def = new b2FixtureDef();
		def.shape = shape;
		def.density = density;

		return CreateFixture(def);
	}

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	public void DestroyFixture(b2Fixture fixture)
	{
		if (fixture == null)
		{
			return;
		}

		Debug.Assert(m_world.IsLocked() == false);
		if (m_world.IsLocked() == true)
		{
			return;
		}

		Debug.Assert(fixture.m_body == this);

		// Remove the fixture from this body's singly linked list.
		Debug.Assert(m_fixtureCount > 0);
		b2Fixture node = m_fixtureList;
		bool found = false;
		while (node != null)
		{
			if (node == fixture)
			{
				node = fixture.m_next;
				found = true;
				break;
			}

			node = node.m_next;
		}

		// You tried to remove a shape that is not attached to this body.
		Debug.Assert(found);

		// Destroy any contacts associated with the fixture.
		b2ContactEdge edge = m_contactList;
		while (edge != null)
		{
			b2Contact c = edge.contact;
			edge = edge.next;

			b2Fixture fixtureA = c.GetFixtureA();
			b2Fixture fixtureB = c.GetFixtureB();

			if (fixture == fixtureA || fixture == fixtureB)
			{
				// This destroys the contact and removes it from
				// this body's contact list.
				m_world.m_contactManager.Destroy(c);
			}
		}

		if ((m_flags & BodyFlags.e_activeFlag) != 0)
		{
			b2BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			fixture.DestroyProxies(broadPhase);
		}

		fixture.m_body = null;
		fixture.m_next = null;
		fixture.Destroy();

		--m_fixtureCount;

		// Reset the mass data.
		ResetMassData();
	}

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to b2World::Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	public void SetTransform(b2Vec2 position, float angle)
	{
		Debug.Assert(m_world.IsLocked() == false);
		if (m_world.IsLocked() == true)
		{
			return;
		}

		m_xf.q.Set(angle);
		m_xf.p = position;

		m_sweep.c = Utils.b2Mul(m_xf, m_sweep.localCenter);
		m_sweep.a = angle;

		m_sweep.c0 = m_sweep.c;
		m_sweep.a0 = angle;

		b2BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
		for (b2Fixture f = m_fixtureList; f != null; f = f.m_next)
		{
			f.Synchronize(broadPhase, m_xf, m_xf);
		}
	}

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	public b2Transform GetTransform()
	{
		return m_xf;
	}

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	public b2Vec2 GetPosition()
	{
		return m_xf.p;
	}

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	public float GetAngle()
	{
		return m_sweep.a;
	}

	/// Get the world position of the center of mass.
	public b2Vec2 GetWorldCenter()
	{
		return m_sweep.c;
	}

	/// Get the local position of the center of mass.
	public b2Vec2 GetLocalCenter()
	{
		return m_sweep.localCenter;
	}

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	public void SetLinearVelocity(b2Vec2 v)
	{
		if (m_type == BodyType.b2_staticBody)
		{
			return;
		}

		if (Utils.b2Dot(v, v) > 0.0f)
		{
			SetAwake(true);
		}

		m_linearVelocity = v;
	}

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	public b2Vec2 GetLinearVelocity()
	{
		return m_linearVelocity;
	}

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	public void SetAngularVelocity(float w)
	{
		if (m_type == BodyType.b2_staticBody)
		{
			return;
		}

		if (w * w > 0.0f)
		{
			SetAwake(true);
		}

		m_angularVelocity = w;
	}

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	public float GetAngularVelocity()
	{
		return m_angularVelocity;
	}

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	public void ApplyForce(b2Vec2 force, b2Vec2 point, bool wake)
	{
		if (m_type != BodyType.b2_dynamicBody)
		{
			return;
		}

		if (wake && (m_flags & BodyFlags.e_awakeFlag) == 0)
		{
			SetAwake(true);
		}

		// Don't accumulate a force if the body is sleeping.
		if ((m_flags & BodyFlags.e_awakeFlag) != 0)
		{
			m_force += force;
			m_torque += Utils.b2Cross(point - m_sweep.c, force);
		}
	}

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	public void ApplyForceToCenter(b2Vec2 force, bool wake)
	{
		if (m_type != BodyType.b2_dynamicBody)
		{
			return;
		}

		if (wake && (m_flags & BodyFlags.e_awakeFlag) == 0)
		{
			SetAwake(true);
		}

		// Don't accumulate a force if the body is sleeping
		if ((m_flags & BodyFlags.e_awakeFlag) != 0)
		{
			m_force += force;
		}
	}

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	public void ApplyTorque(float torque, bool wake)
	{
		if (m_type != BodyType.b2_dynamicBody)
		{
			return;
		}

		if (wake && (m_flags & BodyFlags.e_awakeFlag) == 0)
		{
			SetAwake(true);
		}

		// Don't accumulate a force if the body is sleeping
		if ((m_flags & BodyFlags.e_awakeFlag) != 0)
		{
			m_torque += torque;
		}
	}

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	public void ApplyLinearImpulse(b2Vec2 impulse, b2Vec2 point, bool wake)
	{
		if (m_type != BodyType.b2_dynamicBody)
		{
			return;
		}

		if (wake && (m_flags & BodyFlags.e_awakeFlag) == 0)
		{
			SetAwake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if ((m_flags & BodyFlags.e_awakeFlag) != 0)
		{
			m_linearVelocity += m_invMass * impulse;
			m_angularVelocity += m_invI * Utils.b2Cross(point - m_sweep.c, impulse);
		}
	}

	/// Apply an impulse to the center of mass. This immediately modifies the velocity.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param wake also wake up the body
	public void ApplyLinearImpulseToCenter(b2Vec2 impulse, bool wake)
	{
		if (m_type != BodyType.b2_dynamicBody)
		{
			return;
		}

		if (wake && (m_flags & BodyFlags.e_awakeFlag) == 0)
		{
			SetAwake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if ((m_flags & BodyFlags.e_awakeFlag) != 0)
		{
			m_linearVelocity += m_invMass * impulse;
		}
	}

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	public void ApplyAngularImpulse(float impulse, bool wake)
	{
		if (m_type != BodyType.b2_dynamicBody)
		{
			return;
		}

		if (wake && (m_flags & BodyFlags.e_awakeFlag) == 0)
		{
			SetAwake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if ((m_flags & BodyFlags.e_awakeFlag) != 0)
		{
			m_angularVelocity += m_invI * impulse;
		}
	}

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	public float GetMass()
	{
		return m_mass;
	}

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	public float GetInertia()
	{
		return m_I + m_mass * Utils.b2Dot(m_sweep.localCenter, m_sweep.localCenter);
	}

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	public void GetMassData(b2MassData data)
	{
		data.mass = m_mass;
		data.I = m_I + m_mass * Utils.b2Dot(m_sweep.localCenter, m_sweep.localCenter);
		data.center = m_sweep.localCenter;
	}

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	public void SetMassData(b2MassData massData)
	{
		Debug.Assert(m_world.IsLocked() == false);
		if (m_world.IsLocked() == true)
		{
			return;
		}

		if (m_type != BodyType.b2_dynamicBody)
		{
			return;
		}

		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		m_mass = massData.mass;
		if (m_mass <= 0.0f)
		{
			m_mass = 1.0f;
		}

		m_invMass = 1.0f / m_mass;

		if (massData.I > 0.0f && (m_flags & BodyFlags.e_fixedRotationFlag) == 0)
		{
			m_I = massData.I - m_mass * Utils.b2Dot(massData.center, massData.center);
			Debug.Assert(m_I > 0.0f);
			m_invI = 1.0f / m_I;
		}

		// Move center of mass.
		b2Vec2 oldCenter = new b2Vec2(m_sweep.c);
		m_sweep.localCenter = massData.center;
		m_sweep.c0 = m_sweep.c = Utils.b2Mul(m_xf, m_sweep.localCenter);

		// Update center of mass velocity.
		m_linearVelocity += Utils.b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
	}

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	public void ResetMassData()
	{
		// Compute mass data from shapes. Each shape has its own density.
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;
		m_sweep.localCenter.SetZero();

		// Static and kinematic bodies have zero mass.
		if (m_type == BodyType.b2_staticBody || m_type == BodyType.b2_kinematicBody)
		{
			m_sweep.c0 = m_xf.p;
			m_sweep.c = m_xf.p;
			m_sweep.a0 = m_sweep.a;
			return;
		}

		Debug.Assert(m_type == BodyType.b2_dynamicBody);

		// Accumulate mass over all fixtures.
		b2Vec2 localCenter = new b2Vec2();
		for (b2Fixture f = m_fixtureList; f != null; f = f.m_next)
		{
			if (f.m_density == 0.0f)
			{
				continue;
			}

			b2MassData massData = new b2MassData();
			f.GetMassData(massData);
			m_mass += massData.mass;
			localCenter += massData.mass * massData.center;
			m_I += massData.I;
		}

		// Compute center of mass.
		if (m_mass > 0.0f)
		{
			m_invMass = 1.0f / m_mass;
			localCenter *= m_invMass;
		}
		else
		{
			// Force all dynamic bodies to have a positive mass.
			m_mass = 1.0f;
			m_invMass = 1.0f;
		}

		if (m_I > 0.0f && (m_flags & BodyFlags.e_fixedRotationFlag) == 0)
		{
			// Center the inertia about the center of mass.
			m_I -= m_mass * Utils.b2Dot(localCenter, localCenter);
			Debug.Assert(m_I > 0.0f);
			m_invI = 1.0f / m_I;

		}
		else
		{
			m_I = 0.0f;
			m_invI = 0.0f;
		}

		// Move center of mass.


		b2Vec2 oldCenter = new b2Vec2(m_sweep.c);


		m_sweep.localCenter = localCenter;
		m_sweep.c0 = m_sweep.c = Utils.b2Mul(m_xf, m_sweep.localCenter);

		// Update center of mass velocity.
		m_linearVelocity += Utils.b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
	}

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.


	public b2Vec2 GetWorldPoint(b2Vec2 localPoint)
	{
		return Utils.b2Mul(m_xf, localPoint);
	}

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.


	public b2Vec2 GetWorldVector(b2Vec2 localVector)
	{
		return Utils.b2Mul(m_xf.q, localVector);
	}

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.


	public b2Vec2 GetLocalPoint(b2Vec2 worldPoint)
	{
		return Utils.b2MulT(m_xf, worldPoint);
	}

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.


	public b2Vec2 GetLocalVector(b2Vec2 worldVector)
	{
		return Utils.b2MulT(m_xf.q, worldVector);
	}

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.


	public b2Vec2 GetLinearVelocityFromWorldPoint(b2Vec2 worldPoint)
	{
		return m_linearVelocity + Utils.b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
	}

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.


	public b2Vec2 GetLinearVelocityFromLocalPoint(b2Vec2 localPoint)
	{
		return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
	}

	/// Get the linear damping of the body.


	public float GetLinearDamping()
	{
		return m_linearDamping;
	}

	/// Set the linear damping of the body.
	public void SetLinearDamping(float linearDamping)
	{
		m_linearDamping = linearDamping;
	}

	/// Get the angular damping of the body.


	public float GetAngularDamping()
	{
		return m_angularDamping;
	}

	/// Set the angular damping of the body.
	public void SetAngularDamping(float angularDamping)
	{
		m_angularDamping = angularDamping;
	}

	/// Get the gravity scale of the body.


	public float GetGravityScale()
	{
		return m_gravityScale;
	}

	/// Set the gravity scale of the body.
	public void SetGravityScale(float scale)
	{
		m_gravityScale = scale;
	}

	/// Set the type of this body. This may alter the mass and velocity.
	public void SetType(BodyType type)
	{
		Debug.Assert(m_world.IsLocked() == false);
		if (m_world.IsLocked() == true)
		{
			return;
		}

		if (m_type == type)
		{
			return;
		}

		m_type = type;

		ResetMassData();

		if (m_type == BodyType.b2_staticBody)
		{
			m_linearVelocity.SetZero();
			m_angularVelocity = 0.0f;
			m_sweep.a0 = m_sweep.a;


			m_sweep.c0 = m_sweep.c;
			SynchronizeFixtures();
		}

		SetAwake(true);

		m_force.SetZero();
		m_torque = 0.0f;

		// Delete the attached contacts.
		b2ContactEdge ce = m_contactList;
		while (ce != null)
		{
			b2ContactEdge ce0 = ce;
			ce = ce.next;
			m_world.m_contactManager.Destroy(ce0.contact);
		}
		m_contactList = null;

		// Touch the proxies so that new contacts will be created (when appropriate)
		b2BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
		for (b2Fixture f = m_fixtureList; f != null; f = f.m_next)
		{
			int proxyCount = f.m_proxyCount;
			for (int i = 0; i < proxyCount; ++i)
			{
				broadPhase.TouchProxy(f.m_proxies[i].proxyId);
			}
		}
	}

	/// Get the type of this body.
	public new BodyType GetType()
	{
		return m_type;
	}

	/// Should this body be treated like a bullet for continuous collision detection?
	public void SetBullet(bool flag)
	{
		if (flag)
		{
			m_flags |= BodyFlags.e_bulletFlag;
		}
		else
		{
			m_flags &= ~BodyFlags.e_bulletFlag;
		}
	}

	/// Is this body treated like a bullet for continuous collision detection?


	public bool IsBullet()
	{
		return (m_flags & BodyFlags.e_bulletFlag) == BodyFlags.e_bulletFlag;
	}

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	public void SetSleepingAllowed(bool flag)
	{
		if (flag)
		{
			m_flags |= BodyFlags.e_autoSleepFlag;
		}
		else
		{
			m_flags &= ~BodyFlags.e_autoSleepFlag;
			SetAwake(true);
		}
	}

	/// Is this body allowed to sleep


	public bool IsSleepingAllowed()
	{
		return (m_flags & BodyFlags.e_autoSleepFlag) == BodyFlags.e_autoSleepFlag;
	}

	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	public void SetAwake(bool flag)
	{
		if (flag)
		{
			m_flags |= BodyFlags.e_awakeFlag;
			m_sleepTime = 0.0f;
		}
		else
		{
			m_flags &= ~BodyFlags.e_awakeFlag;
			m_sleepTime = 0.0f;
			m_linearVelocity.SetZero();
			m_angularVelocity = 0.0f;
			m_force.SetZero();
			m_torque = 0.0f;
		}
	}

	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	public bool IsAwake()
	{
		return (m_flags & BodyFlags.e_awakeFlag) == BodyFlags.e_awakeFlag;
	}

	/// Set the active state of the body. An inactive body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on inactive bodies.
	/// Fixtures on an inactive body are implicitly inactive and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to an inactive body are implicitly inactive.
	/// An inactive body is still owned by a b2World object and remains
	/// in the body list.
	public void SetActive(bool flag)
	{
		Debug.Assert(m_world.IsLocked() == false);

		if (flag == IsActive())
		{
			return;
		}

		if (flag)
		{
			m_flags |= BodyFlags.e_activeFlag;

			// Create all proxies.
			b2BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			for (b2Fixture f = m_fixtureList; f != null; f = f.m_next)
			{
				f.CreateProxies(broadPhase, m_xf);
			}

			// Contacts are created the next time step.
		}
		else
		{
			m_flags &= ~BodyFlags.e_activeFlag;

			// Destroy all proxies.
			b2BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			for (b2Fixture f = m_fixtureList; f != null; f = f.m_next)
			{
				f.DestroyProxies(broadPhase);
			}

			// Destroy the attached contacts.
			b2ContactEdge ce = m_contactList;
			while (ce != null)
			{
				b2ContactEdge ce0 = ce;
				ce = ce.next;
				m_world.m_contactManager.Destroy(ce0.contact);
			}
			m_contactList = null;
		}
	}

	/// Get the active state of the body.


	public bool IsActive()
	{
		return (m_flags & BodyFlags.e_activeFlag) == BodyFlags.e_activeFlag;
	}

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	public void SetFixedRotation(bool flag)
	{
		bool status = (m_flags & BodyFlags.e_fixedRotationFlag) == BodyFlags.e_fixedRotationFlag;
		if (status == flag)
		{
			return;
		}

		if (flag)
		{
			m_flags |= BodyFlags.e_fixedRotationFlag;
		}
		else
		{
			m_flags &= ~BodyFlags.e_fixedRotationFlag;
		}

		m_angularVelocity = 0.0f;

		ResetMassData();
	}

	/// Does this body have fixed rotation?
	public bool IsFixedRotation()
	{
		return (m_flags & BodyFlags.e_fixedRotationFlag) == BodyFlags.e_fixedRotationFlag;
	}

	/// Get the list of all fixtures attached to this body.
	public b2Fixture GetFixtureList()
	{
		return m_fixtureList;
	}

	/// Get the list of all joints attached to this body.
	public b2JointEdge GetJointList()
	{
		return m_jointList;
	}

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use b2ContactListener.
	public b2ContactEdge GetContactList()
	{
		return m_contactList;
	}

	/// Get the next body in the world's body list.
	public b2Body GetNext()
	{
		return m_next;
	}

	/// Get the user data pointer that was provided in the body definition.
	public object GetUserData()
	{
		return m_userData;
	}

	/// Set the user data. Use this to store your application specific data.
	public void SetUserData(object data)
	{
		m_userData = data;
	}

	/// Get the parent world of this body.
	public b2World GetWorld()
	{
		return m_world;
	}

	/// Dump this body to a log file
	public void Dump()
	{
		int bodyIndex = m_islandIndex;

		Utils.b2Log("{\n");
		Utils.b2Log("  b2BodyDef bd;\n");
		Utils.b2Log("  bd.type = b2BodyType(%d);\n", m_type);
		Utils.b2Log("  bd.position.Set(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
		Utils.b2Log("  bd.angle = %.15lef;\n", m_sweep.a);
		Utils.b2Log("  bd.linearVelocity.Set(%.15lef, %.15lef);\n", m_linearVelocity.x, m_linearVelocity.y);
		Utils.b2Log("  bd.angularVelocity = %.15lef;\n", m_angularVelocity);
		Utils.b2Log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
		Utils.b2Log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
		Utils.b2Log("  bd.allowSleep = bool(%d);\n", m_flags & BodyFlags.e_autoSleepFlag);
		Utils.b2Log("  bd.awake = bool(%d);\n", m_flags & BodyFlags.e_awakeFlag);
		Utils.b2Log("  bd.fixedRotation = bool(%d);\n", m_flags & BodyFlags.e_fixedRotationFlag);
		Utils.b2Log("  bd.bullet = bool(%d);\n", m_flags & BodyFlags.e_bulletFlag);
		Utils.b2Log("  bd.active = bool(%d);\n", m_flags & BodyFlags.e_activeFlag);
		Utils.b2Log("  bd.gravityScale = %.15lef;\n", m_gravityScale);
		Utils.b2Log("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
		Utils.b2Log("\n");
		for (b2Fixture f = m_fixtureList; f != null; f = f.m_next)
		{
			Utils.b2Log("  {\n");
			f.Dump(bodyIndex);
			Utils.b2Log("  }\n");
		}
		Utils.b2Log("}\n");
	}



    //	friend class b2World;

    //	friend class b2Island;

    //	friend class b2ContactManager;

    //	friend class b2ContactSolver;

    //	friend class b2Contact;


    //	friend class b2DistanceJoint;

    //	friend class b2FrictionJoint;

    //	friend class b2GearJoint;

    //	friend class b2MotorJoint;

    //	friend class b2MouseJoint;

    //	friend class b2PrismaticJoint;

    //	friend class b2PulleyJoint;

    //	friend class b2RevoluteJoint;

    //	friend class b2RopeJoint;

    //	friend class b2WeldJoint;

    //	friend class b2WheelJoint;

    // m_flags

    internal enum BodyFlags
    {
		e_islandFlag = 0x0001,
		e_awakeFlag = 0x0002,
		e_autoSleepFlag = 0x0004,
		e_bulletFlag = 0x0008,
		e_fixedRotationFlag = 0x0010,
		e_activeFlag = 0x0020,
		e_toiFlag = 0x0040
	}

    internal b2Body(b2BodyDef bd, b2World world)
	{
		Debug.Assert(bd.position.IsValid());
		Debug.Assert(bd.linearVelocity.IsValid());
		Debug.Assert(Utils.b2IsValid(bd.angle));
		Debug.Assert(Utils.b2IsValid(bd.angularVelocity));
		Debug.Assert(Utils.b2IsValid(bd.angularDamping) && bd.angularDamping >= 0.0f);
		Debug.Assert(Utils.b2IsValid(bd.linearDamping) && bd.linearDamping >= 0.0f);

		m_flags = 0;

		if (bd.bullet)
		{
			m_flags |= BodyFlags.e_bulletFlag;
		}
		if (bd.fixedRotation)
		{
			m_flags |= BodyFlags.e_fixedRotationFlag;
		}
		if (bd.allowSleep)
		{
			m_flags |= BodyFlags.e_autoSleepFlag;
		}
		if (bd.awake)
		{
			m_flags |= BodyFlags.e_awakeFlag;
		}
		if (bd.active)
		{
			m_flags |= BodyFlags.e_activeFlag;
		}

		m_world = world;



		m_xf.p = bd.position;
		m_xf.q.Set(bd.angle);

		m_sweep.localCenter.SetZero();

		m_sweep.c0 = m_xf.p;
		m_sweep.c = m_xf.p;
		m_sweep.a0 = bd.angle;
		m_sweep.a = bd.angle;
		m_sweep.alpha0 = 0.0f;

		m_jointList = null;
		m_contactList = null;
		m_prev = null;
		m_next = null;

		m_linearVelocity = bd.linearVelocity;
		m_angularVelocity = bd.angularVelocity;

		m_linearDamping = bd.linearDamping;
		m_angularDamping = bd.angularDamping;
		m_gravityScale = bd.gravityScale;

		m_force.SetZero();
		m_torque = 0.0f;

		m_sleepTime = 0.0f;

		m_type = bd.type;

		if (m_type == BodyType.b2_dynamicBody)
		{
			m_mass = 1.0f;
			m_invMass = 1.0f;
		}
		else
		{
			m_mass = 0.0f;
			m_invMass = 0.0f;
		}

		m_I = 0.0f;
		m_invI = 0.0f;

		m_userData = bd.userData;

		m_fixtureList = null;
		m_fixtureCount = 0;
	}
	public void Dispose()
	{
		// shapes and joints are destroyed in b2World::Destroy
	}

    internal void SynchronizeFixtures()
	{
		b2Transform xf1 = new b2Transform();
		xf1.q.Set(m_sweep.a0);
		xf1.p = m_sweep.c0 - Utils.b2Mul(xf1.q, m_sweep.localCenter);

		b2BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
		for (b2Fixture f = m_fixtureList; f != null; f = f.m_next)
		{
			f.Synchronize(broadPhase, xf1, m_xf);
		}
	}
    internal void SynchronizeTransform()
	{
		m_xf.q.Set(m_sweep.a);
		m_xf.p = m_sweep.c - Utils.b2Mul(m_xf.q, m_sweep.localCenter);
	}

    // This is used to prevent connected bodies from colliding.
    // It may lie, depending on the collideConnected flag.
    internal bool ShouldCollide(b2Body other)
	{
		// At least one body should be dynamic.
		if (m_type != BodyType.b2_dynamicBody && other.m_type != BodyType.b2_dynamicBody)
		{
			return false;
		}

		// Does a joint prevent collision?
		for (b2JointEdge jn = m_jointList; jn != null; jn = jn.next)
		{
			if (jn.other == other)
			{
				if (jn.joint.m_collideConnected == false)
				{
					return false;
				}
			}
		}

		return true;
	}

    internal void Advance(float alpha)
	{
		// Advance to the new safe time. This doesn't sync the broad-phase.
		m_sweep.Advance(alpha);
		m_sweep.c = m_sweep.c0;
		m_sweep.a = m_sweep.a0;
		m_xf.q.Set(m_sweep.a);
		m_xf.p = m_sweep.c - Utils.b2Mul(m_xf.q, m_sweep.localCenter);
	}

    internal BodyType m_type;

    internal BodyFlags m_flags;

    internal int m_islandIndex;

    internal b2Transform m_xf = new b2Transform(); // the body origin transform
    internal b2Sweep m_sweep = new b2Sweep(); // the swept motion for CCD

    internal b2Vec2 m_linearVelocity = new b2Vec2();
    internal float m_angularVelocity;

    internal b2Vec2 m_force = new b2Vec2();
    internal float m_torque;

    internal b2World m_world;
    internal b2Body m_prev;
    internal b2Body m_next;

    internal b2Fixture m_fixtureList;
    internal int m_fixtureCount;

	internal b2JointEdge m_jointList;
    internal b2ContactEdge m_contactList;

    internal float m_mass;
    internal float m_invMass;

    // Rotational inertia about the center of mass.
    internal float m_I;
    internal float m_invI;

    internal float m_linearDamping;
    internal float m_angularDamping;
    internal float m_gravityScale;

    internal float m_sleepTime;

    internal object m_userData;
}


