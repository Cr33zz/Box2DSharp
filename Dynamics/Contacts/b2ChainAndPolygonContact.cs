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

public class b2ChainAndPolygonContact : b2Contact
{
	public static b2Contact Create(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB)
	{
		return new b2ChainAndPolygonContact(fixtureA, indexA, fixtureB, indexB);
	}
	public static void Destroy(ref b2Contact contact)
	{
	    contact = null;
	}

	public b2ChainAndPolygonContact(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB) : base(fixtureA, indexA, fixtureB, indexB)
	{
		Debug.Assert(m_fixtureA.GetType() == b2Shape.Type.e_chain);
		Debug.Assert(m_fixtureB.GetType() == b2Shape.Type.e_polygon);
	}
	public new void Dispose()
	{
		base.Dispose();
	}

	public override void Evaluate(b2Manifold manifold, b2Transform xfA, b2Transform xfB)
	{
		b2ChainShape chain = (b2ChainShape)m_fixtureA.GetShape();
		b2EdgeShape edge = new b2EdgeShape();
		chain.GetChildEdge(edge, m_indexA);
		GlobalMembers.b2CollideEdgeAndPolygon(manifold, edge, xfA, (b2PolygonShape)m_fixtureB.GetShape(), xfB);
	}
}
