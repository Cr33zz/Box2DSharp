using System;
using System.Diagnostics;

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

/// Friction joint definition.
public class b2FrictionJointDef : b2JointDef
{
	public b2FrictionJointDef()
	{
		type = b2JointType.e_frictionJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		maxForce = 0.0f;
		maxTorque = 0.0f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.

	// Point-to-point constraint
	// Cdot = v2 - v1
	//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
	// J = [-I -r1_skew I r2_skew ]
	// Identity used:
	// w k % (rx i + ry j) = w * (-ry i + rx j)

	// Angle constraint
	// Cdot = w2 - w1
	// J = [0 0 -1 0 0 1]
	// K = invI1 + invI2

	public void Initialize(b2Body bA, b2Body bB, b2Vec2 anchor)
	{
		bodyA = bA;
		bodyB = bB;


		localAnchorA = bodyA.GetLocalPoint(anchor);


		localAnchorB = bodyB.GetLocalPoint(anchor);
	}

	/// The local anchor point relative to bodyA's origin.
	public b2Vec2 localAnchorA = new b2Vec2();

	/// The local anchor point relative to bodyB's origin.
	public b2Vec2 localAnchorB = new b2Vec2();

	/// The maximum friction force in N.
	public float maxForce;

	/// The maximum friction torque in N-m.
	public float maxTorque;
}

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
public class b2FrictionJoint : b2Joint
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
		return inv_dt * m_linearImpulse;
	}


	public override float GetReactionTorque(float inv_dt)
	{
		return inv_dt * m_angularImpulse;
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

	/// Set the maximum friction force in N.
	public void SetMaxForce(float force)
	{
		Debug.Assert(GlobalMembers.b2IsValid(force) && force >= 0.0f);
		m_maxForce = force;
	}

	/// Get the maximum friction force in N.
	public float GetMaxForce()
	{
		return m_maxForce;
	}

	/// Set the maximum friction torque in N*m.
	public void SetMaxTorque(float torque)
	{
		Debug.Assert(GlobalMembers.b2IsValid(torque) && torque >= 0.0f);
		m_maxTorque = torque;
	}

	/// Get the maximum friction torque in N*m.
	public float GetMaxTorque()
	{
		return m_maxTorque;
	}

	/// Dump joint to dmLog
	public override void Dump()
	{
		int indexA = m_bodyA.m_islandIndex;
		int indexB = m_bodyB.m_islandIndex;

		GlobalMembers.b2Log("  b2FrictionJointDef jd;\n");
		GlobalMembers.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
		GlobalMembers.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
		GlobalMembers.b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		GlobalMembers.b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
		GlobalMembers.b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
		GlobalMembers.b2Log("  jd.maxForce = %.15lef;\n", m_maxForce);
		GlobalMembers.b2Log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
		GlobalMembers.b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
	}



    internal b2FrictionJoint(b2FrictionJointDef def) : base(def)
	{
		m_localAnchorA = def.localAnchorA;
		m_localAnchorB = def.localAnchorB;

		m_linearImpulse.SetZero();
		m_angularImpulse = 0.0f;

		m_maxForce = def.maxForce;
		m_maxTorque = def.maxTorque;
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

		float aA = data.positions[m_indexA].a;
		b2Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;

		float aB = data.positions[m_indexB].a;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		b2Rot qA = new b2Rot(aA);
		b2Rot qB = new b2Rot(aB);

		// Compute the effective mass matrix.
		m_rA = GlobalMembers.b2Mul(qA, m_localAnchorA - m_localCenterA);
		m_rB = GlobalMembers.b2Mul(qB, m_localAnchorB - m_localCenterB);

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		float mA = m_invMassA;
		float mB = m_invMassB;
		float iA = m_invIA;
		float iB = m_invIB;

		b2Mat22 K = new b2Mat22();
		K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
		K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

		m_linearMass = K.GetInverse();

		m_angularMass = iA + iB;
		if (m_angularMass > 0.0f)
		{
			m_angularMass = 1.0f / m_angularMass;
		}

		if (data.step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			m_linearImpulse *= data.step.dtRatio;
			m_angularImpulse *= data.step.dtRatio;

			b2Vec2 P = new b2Vec2(m_linearImpulse.x, m_linearImpulse.y);
			vA -= mA * P;
			wA -= iA * (GlobalMembers.b2Cross(m_rA, P) + m_angularImpulse);
			vB += mB * P;
			wB += iB * (GlobalMembers.b2Cross(m_rB, P) + m_angularImpulse);
		}
		else
		{
			m_linearImpulse.SetZero();
			m_angularImpulse = 0.0f;
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

		float mA = m_invMassA;
		float mB = m_invMassB;
		float iA = m_invIA;
		float iB = m_invIB;

		float h = data.step.dt;

		{
		// Solve angular friction
			float Cdot = wB - wA;
			float impulse = -m_angularMass * Cdot;

			float oldImpulse = m_angularImpulse;
			float maxImpulse = h * m_maxTorque;
			m_angularImpulse = GlobalMembers.b2Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		{
		// Solve linear friction
			b2Vec2 Cdot = vB + GlobalMembers.b2Cross(wB, m_rB) - vA - GlobalMembers.b2Cross(wA, m_rA);

			b2Vec2 impulse = -GlobalMembers.b2Mul(m_linearMass, Cdot);


			b2Vec2 oldImpulse = new b2Vec2(m_linearImpulse);
			m_linearImpulse += impulse;

			float maxImpulse = h * m_maxForce;

			if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				m_linearImpulse.Normalize();
				m_linearImpulse *= maxImpulse;
			}

			impulse = m_linearImpulse - oldImpulse;

			vA -= mA * impulse;
			wA -= iA * GlobalMembers.b2Cross(m_rA, impulse);

			vB += mB * impulse;
			wB += iB * GlobalMembers.b2Cross(m_rB, impulse);
		}

		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
    internal override bool SolvePositionConstraints(b2SolverData data)
	{
		return true;
	}

	protected b2Vec2 m_localAnchorA = new b2Vec2();
	protected b2Vec2 m_localAnchorB = new b2Vec2();

	// Solver shared
	protected b2Vec2 m_linearImpulse = new b2Vec2();
	protected float m_angularImpulse;
	protected float m_maxForce;
	protected float m_maxTorque;

	// Solver temp
	protected int m_indexA;
	protected int m_indexB;
	protected b2Vec2 m_rA = new b2Vec2();
	protected b2Vec2 m_rB = new b2Vec2();
	protected b2Vec2 m_localCenterA = new b2Vec2();
	protected b2Vec2 m_localCenterB = new b2Vec2();
	protected float m_invMassA;
	protected float m_invMassB;
	protected float m_invIA;
	protected float m_invIB;
	protected b2Mat22 m_linearMass = new b2Mat22();
	protected float m_angularMass;
}
