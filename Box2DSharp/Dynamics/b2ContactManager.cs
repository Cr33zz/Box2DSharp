﻿/*
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

// Delegate of b2World.
public class b2ContactManager
{
	public b2ContactManager()
	{
		m_contactList = null;
		m_contactCount = 0;
		m_contactFilter = Utils.b2_defaultFilter;
		m_contactListener = Utils.b2_defaultListener;
	    m_broadphaseCollision = AddPair;
	}

	// Broad-phase callback.
	public void AddPair(object proxyUserDataA, object proxyUserDataB)
	{
		b2FixtureProxy proxyA = (b2FixtureProxy)proxyUserDataA;
		b2FixtureProxy proxyB = (b2FixtureProxy)proxyUserDataB;

		b2Fixture fixtureA = proxyA.fixture;
		b2Fixture fixtureB = proxyB.fixture;

		int indexA = proxyA.childIndex;
		int indexB = proxyB.childIndex;

		b2Body bodyA = fixtureA.GetBody();
		b2Body bodyB = fixtureB.GetBody();

		// Are the fixtures on the same body?
		if (bodyA == bodyB)
		{
			return;
		}

		// TODO_ERIN use a hash table to remove a potential bottleneck when both
		// bodies have a lot of contacts.
		// Does a contact already exist?
		b2ContactEdge edge = bodyB.GetContactList();
		while (edge != null)
		{
			if (edge.other == bodyA)
			{
				b2Fixture fA = edge.contact.GetFixtureA();
				b2Fixture fB = edge.contact.GetFixtureB();
				int iA = edge.contact.GetChildIndexA();
				int iB = edge.contact.GetChildIndexB();

				if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
				{
					// A contact already exists.
					return;
				}

				if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
				{
					// A contact already exists.
					return;
				}
			}

			edge = edge.next;
		}

		// Does a joint override collision? Is at least one body dynamic?
		if (bodyB.ShouldCollide(bodyA) == false)
		{
			return;
		}

		// Check user filtering.
		if (m_contactFilter != null && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
		{
			return;
		}

		// Call the factory.
		b2Contact c = b2Contact.Create(fixtureA, indexA, fixtureB, indexB);
		if (c == null)
		{
			return;
		}

		// Contact creation may swap fixtures.
		fixtureA = c.GetFixtureA();
		fixtureB = c.GetFixtureB();
		indexA = c.GetChildIndexA();
		indexB = c.GetChildIndexB();
		bodyA = fixtureA.GetBody();
		bodyB = fixtureB.GetBody();

		// Insert into the world.
		c.m_prev = null;
		c.m_next = m_contactList;
		if (m_contactList != null)
		{
			m_contactList.m_prev = c;
		}
		m_contactList = c;

		// Connect to island graph.

		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;

		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null)
		{
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;

		// Connect to body B
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;

		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null)
		{
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;

		// Wake up the bodies
		if (fixtureA.IsSensor() == false && fixtureB.IsSensor() == false)
		{
			bodyA.SetAwake(true);
			bodyB.SetAwake(true);
		}

		++m_contactCount;
	}

	public void FindNewContacts()
	{
		m_broadPhase.UpdatePairs(m_broadphaseCollision);
	}

	public void Destroy(b2Contact c)
	{
		b2Fixture fixtureA = c.GetFixtureA();
		b2Fixture fixtureB = c.GetFixtureB();
		b2Body bodyA = fixtureA.GetBody();
		b2Body bodyB = fixtureB.GetBody();

		if (m_contactListener != null && c.IsTouching())
		{
			m_contactListener.EndContact(c);
		}

		// Remove from the world.
		if (c.m_prev != null)
		{
			c.m_prev.m_next = c.m_next;
		}

		if (c.m_next != null)
		{
			c.m_next.m_prev = c.m_prev;
		}

		if (c == m_contactList)
		{
			m_contactList = c.m_next;
		}

		// Remove from body 1
		if (c.m_nodeA.prev != null)
		{
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}

		if (c.m_nodeA.next != null)
		{
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}

		if (c.m_nodeA == bodyA.m_contactList)
		{
			bodyA.m_contactList = c.m_nodeA.next;
		}

		// Remove from body 2
		if (c.m_nodeB.prev != null)
		{
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}

		if (c.m_nodeB.next != null)
		{
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}

		if (c.m_nodeB == bodyB.m_contactList)
		{
			bodyB.m_contactList = c.m_nodeB.next;
		}

		// Call the factory.
		b2Contact.Destroy(ref c);
		--m_contactCount;
	}


	// This is the top level collision call for the time step. Here
	// all the narrow phase collision is processed for the world
	// contact list.
	public void Collide()
	{
		// Update awake contacts.
		b2Contact c = m_contactList;
		while (c != null)
		{
			b2Fixture fixtureA = c.GetFixtureA();
			b2Fixture fixtureB = c.GetFixtureB();
			int indexA = c.GetChildIndexA();
			int indexB = c.GetChildIndexB();
			b2Body bodyA = fixtureA.GetBody();
			b2Body bodyB = fixtureB.GetBody();

			// Is this contact flagged for filtering?
			if ((c.m_flags & b2Contact.ContactFlags.e_filterFlag) != 0)
			{
				// Should these bodies collide?
				if (bodyB.ShouldCollide(bodyA) == false)
				{
					b2Contact cNuke = c;
					c = cNuke.GetNext();
					Destroy(cNuke);
					continue;
				}

				// Check user filtering.
				if (m_contactFilter != null && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
				{
					b2Contact cNuke = c;
					c = cNuke.GetNext();
					Destroy(cNuke);
					continue;
				}

				// Clear the filtering flag.
				c.m_flags &= ~b2Contact.ContactFlags.e_filterFlag;
			}

			bool activeA = bodyA.IsAwake() && bodyA.m_type != BodyType.b2_staticBody;
			bool activeB = bodyB.IsAwake() && bodyB.m_type != BodyType.b2_staticBody;

			// At least one body must be awake and it must be dynamic or kinematic.
			if (activeA == false && activeB == false)
			{
				c = c.GetNext();
				continue;
			}

			int proxyIdA = fixtureA.m_proxies[indexA].proxyId;
			int proxyIdB = fixtureB.m_proxies[indexB].proxyId;
			bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (overlap == false)
			{
				b2Contact cNuke = c;
				c = cNuke.GetNext();
				Destroy(cNuke);
				continue;
			}

			// The contact persists.
			c.Update(m_contactListener);
			c = c.GetNext();
		}
	}

	public b2BroadPhase m_broadPhase = new b2BroadPhase();
	public b2Contact m_contactList;
	public int m_contactCount;
	public b2ContactFilter m_contactFilter;
	public b2ContactListener m_contactListener;
    public b2BroadphaseCallback m_broadphaseCollision;
}