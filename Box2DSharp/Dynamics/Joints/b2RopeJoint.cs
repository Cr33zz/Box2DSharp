﻿/*
* Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in b2JointDef.
public class b2RopeJointDef : b2JointDef
{
	public b2RopeJointDef()
	{
		type = b2JointType.e_ropeJoint;
		localAnchorA.Set(-1.0f, 0.0f);
		localAnchorB.Set(1.0f, 0.0f);
		maxLength = 0.0f;
	}

	/// The local anchor point relative to bodyA's origin.
	public b2Vec2 localAnchorA = new b2Vec2();

	/// The local anchor point relative to bodyB's origin.
	public b2Vec2 localAnchorB = new b2Vec2();

	/// The maximum length of the rope.
	/// Warning: this must be larger than b2_linearSlop or
	/// the joint will have no effect.
	public float maxLength;
}

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See b2DistanceJoint if you want to dynamically
/// control length.
public class b2RopeJoint : b2Joint
{


	public override b2Vec2 GetAnchorA()
	{
		return m_bodyA.GetWorldPoint(m_localAnchorA);
	}


	public override b2Vec2 GetAnchorB()
	{
		return m_bodyB.GetWorldPoint(m_localAnchorB);
	}



	public override b2Vec2 GetReactionForce(float inv_dt)
	{
		b2Vec2 F = (inv_dt * m_impulse) * m_u;
		return F;
	}


	public override float GetReactionTorque(float inv_dt)
	{
		
		return 0.0f;
	}

	/// The local anchor point relative to bodyA's origin.


	public b2Vec2 GetLocalAnchorA()
	{
		return m_localAnchorA;
	}

	/// The local anchor point relative to bodyB's origin.


	public b2Vec2 GetLocalAnchorB()
	{
		return m_localAnchorB;
	}

	/// Set/Get the maximum length of the rope.
	public void SetMaxLength(float length)
	{
		m_maxLength = length;
	}


	public float GetMaxLength()
	{
		return m_maxLength;
	}



	public b2LimitState GetLimitState()
	{
		return m_state;
	}

	/// Dump joint to dmLog
	public override void Dump()
	{
		int indexA = m_bodyA.m_islandIndex;
		int indexB = m_bodyB.m_islandIndex;

		GlobalMembers.b2Log("  b2RopeJointDef jd;\n");
		GlobalMembers.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
		GlobalMembers.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
		GlobalMembers.b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		GlobalMembers.b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
		GlobalMembers.b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
		GlobalMembers.b2Log("  jd.maxLength = %.15lef;\n", m_maxLength);
		GlobalMembers.b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
	}



    //	friend class b2Joint;

    // Limit:
    // C = norm(pB - pA) - L
    // u = (pB - pA) / norm(pB - pA)
    // Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
    // J = [-u -cross(rA, u) u cross(rB, u)]
    // K = J * invM * JT
    //   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

    internal b2RopeJoint(b2RopeJointDef def) : base(def)
	{


		m_localAnchorA = def.localAnchorA;


		m_localAnchorB = def.localAnchorB;

		m_maxLength = def.maxLength;

		m_mass = 0.0f;
		m_impulse = 0.0f;
		m_state = b2LimitState.e_inactiveLimit;
		m_length = 0.0f;
	}

    internal override void InitVelocityConstraints(b2SolverData data)
	{
		m_indexA = m_bodyA.m_islandIndex;
		m_indexB = m_bodyB.m_islandIndex;


		m_localCenterA = m_bodyA.m_sweep.localCenter;


		m_localCenterB = m_bodyB.m_sweep.localCenter;
		m_invMassA = m_bodyA.m_invMass;
		m_invMassB = m_bodyB.m_invMass;
		m_invIA = m_bodyA.m_invI;
		m_invIB = m_bodyB.m_invI;

		b2Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		b2Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;

		b2Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		b2Rot qA = new b2Rot(aA);
		b2Rot qB = new b2Rot(aB);



		m_rA = GlobalMembers.b2Mul(qA, m_localAnchorA - m_localCenterA);


		m_rB = GlobalMembers.b2Mul(qB, m_localAnchorB - m_localCenterB);


		m_u = cB + m_rB - cA - m_rA;

		m_length = m_u.Length();

		float C = m_length - m_maxLength;
		if (C > 0.0f)
		{
			m_state = b2LimitState.e_atUpperLimit;
		}
		else
		{
			m_state = b2LimitState.e_inactiveLimit;
		}

		if (m_length > DefineConstants.b2_linearSlop)
		{
			m_u *= 1.0f / m_length;
		}
		else
		{
			m_u.SetZero();
			m_mass = 0.0f;
			m_impulse = 0.0f;
			return;
		}

		// Compute effective mass.
		float crA = GlobalMembers.b2Cross(m_rA, m_u);
		float crB = GlobalMembers.b2Cross(m_rB, m_u);
		float invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

		m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

		if (data.step.warmStarting)
		{
			// Scale the impulse to support a variable time step.
			m_impulse *= data.step.dtRatio;

			b2Vec2 P = m_impulse * m_u;
			vA -= m_invMassA * P;
			wA -= m_invIA * GlobalMembers.b2Cross(m_rA, P);
			vB += m_invMassB * P;
			wB += m_invIB * GlobalMembers.b2Cross(m_rB, P);
		}
		else
		{
			m_impulse = 0.0f;
		}



		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;


		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
    internal override void SolveVelocityConstraints(b2SolverData data)
	{
		b2Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		// Cdot = dot(u, v + cross(w, r))
		b2Vec2 vpA = vA + GlobalMembers.b2Cross(wA, m_rA);
		b2Vec2 vpB = vB + GlobalMembers.b2Cross(wB, m_rB);
		float C = m_length - m_maxLength;
		float Cdot = GlobalMembers.b2Dot(m_u, vpB - vpA);

		// Predictive constraint.
		if (C < 0.0f)
		{
			Cdot += data.step.inv_dt * C;
		}

		float impulse = -m_mass * Cdot;
		float oldImpulse = m_impulse;
		m_impulse = GlobalMembers.b2Min(0.0f, m_impulse + impulse);
		impulse = m_impulse - oldImpulse;

		b2Vec2 P = impulse * m_u;
		vA -= m_invMassA * P;
		wA -= m_invIA * GlobalMembers.b2Cross(m_rA, P);
		vB += m_invMassB * P;
		wB += m_invIB * GlobalMembers.b2Cross(m_rB, P);



		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;


		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
    internal override bool SolvePositionConstraints(b2SolverData data)
	{
		b2Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		b2Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;

		b2Rot qA = new b2Rot(aA);
		b2Rot qB = new b2Rot(aB);

		b2Vec2 rA = GlobalMembers.b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = GlobalMembers.b2Mul(qB, m_localAnchorB - m_localCenterB);
		b2Vec2 u = cB + rB - cA - rA;

		float length = u.Normalize();
		float C = length - m_maxLength;

		C = GlobalMembers.b2Clamp(C, 0.0f, DefineConstants.b2_maxLinearCorrection);

		float impulse = -m_mass * C;
		b2Vec2 P = impulse * u;

		cA -= m_invMassA * P;
		aA -= m_invIA * GlobalMembers.b2Cross(rA, P);
		cB += m_invMassB * P;
		aB += m_invIB * GlobalMembers.b2Cross(rB, P);



		data.positions[m_indexA].c = cA;
		data.positions[m_indexA].a = aA;


		data.positions[m_indexB].c = cB;
		data.positions[m_indexB].a = aB;

		return length - m_maxLength < DefineConstants.b2_linearSlop;
	}

	// Solver shared
	protected b2Vec2 m_localAnchorA = new b2Vec2();
	protected b2Vec2 m_localAnchorB = new b2Vec2();
	protected float m_maxLength;
	protected float m_length;
	protected float m_impulse;

	// Solver temp
	protected int m_indexA;
	protected int m_indexB;
	protected b2Vec2 m_u = new b2Vec2();
	protected b2Vec2 m_rA = new b2Vec2();
	protected b2Vec2 m_rB = new b2Vec2();
	protected b2Vec2 m_localCenterA = new b2Vec2();
	protected b2Vec2 m_localCenterB = new b2Vec2();
	protected float m_invMassA;
	protected float m_invMassB;
	protected float m_invIA;
	protected float m_invIB;
	protected float m_mass;
	protected b2LimitState m_state;
}
