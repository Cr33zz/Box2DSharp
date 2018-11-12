﻿using System.Diagnostics;

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

public class b2EdgeAndPolygonContact : b2Contact
{
	public new static b2Contact Create(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB)
	{
		return new b2EdgeAndPolygonContact(fixtureA, fixtureB);
	}
	public new static void Destroy(ref b2Contact contact)
	{
	    contact = null;
	}

	public b2EdgeAndPolygonContact(b2Fixture fixtureA, b2Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0)
	{
		Debug.Assert(m_fixtureA.GetType() == b2Shape.Type.e_edge);
		Debug.Assert(m_fixtureB.GetType() == b2Shape.Type.e_polygon);
	}
	public new void Dispose()
	{
		base.Dispose();
	}

	public override void Evaluate(b2Manifold manifold, b2Transform xfA, b2Transform xfB)
	{
		GlobalMembers.b2CollideEdgeAndPolygon(manifold, (b2EdgeShape)m_fixtureA.GetShape(), xfA, (b2PolygonShape)m_fixtureB.GetShape(), xfB);
	}
}
