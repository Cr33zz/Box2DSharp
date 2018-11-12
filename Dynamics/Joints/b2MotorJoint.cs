using System;
using System.Diagnostics;

/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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





























/// Motor joint definition.
public class b2MotorJointDef : b2JointDef
{
	public b2MotorJointDef()
	{
		type = b2JointType.e_motorJoint;
		linearOffset.SetZero();
		angularOffset = 0.0f;
		maxForce = 1.0f;
		maxTorque = 1.0f;
		correctionFactor = 0.3f;
	}

	/// Initialize the bodies and offsets using the current transforms.

	// Point-to-point constraint
	// Cdot = v2 - v1
	//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
	// J = [-I -r1_skew I r2_skew ]
	// Identity used:
	// w k % (rx i + ry j) = w * (-ry i + rx j)
	//
	// r1 = offset - c1
	// r2 = -c2

	// Angle constraint
	// Cdot = w2 - w1
	// J = [0 0 -1 0 0 1]
	// K = invI1 + invI2

	public void Initialize(b2Body bA, b2Body bB)
	{
		bodyA = bA;
		bodyB = bB;
		b2Vec2 xB = bodyB.GetPosition();


		linearOffset = bodyA.GetLocalPoint(xB);

		float angleA = bodyA.GetAngle();
		float angleB = bodyB.GetAngle();
		angularOffset = angleB - angleA;
	}

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	public b2Vec2 linearOffset = new b2Vec2();

	/// The bodyB angle minus bodyA angle in radians.
	public float angularOffset;

	/// The maximum motor force in N.
	public float maxForce;

	/// The maximum motor torque in N-m.
	public float maxTorque;

	/// Position correction factor in the range [0,1].
	public float correctionFactor;
}

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
public class b2MotorJoint : b2Joint
{


	public override b2Vec2 GetAnchorA()
	{
		return m_bodyA.GetPosition();
	}


	public override b2Vec2 GetAnchorB()
	{
		return m_bodyB.GetPosition();
	}



	public override b2Vec2 GetReactionForce(float inv_dt)
	{
		return inv_dt * m_linearImpulse;
	}


	public override float GetReactionTorque(float inv_dt)
	{
		return inv_dt * m_angularImpulse;
	}

	/// Set/get the target linear offset, in frame A, in meters.
	public void SetLinearOffset(b2Vec2 linearOffset)
	{
		if (linearOffset.x != m_linearOffset.x || linearOffset.y != m_linearOffset.y)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);


			m_linearOffset = linearOffset;
		}
	}


	public b2Vec2 GetLinearOffset()
	{
		return m_linearOffset;
	}

	/// Set/get the target angular offset, in radians.
	public void SetAngularOffset(float angularOffset)
	{
		if (angularOffset != m_angularOffset)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_angularOffset = angularOffset;
		}
	}


	public float GetAngularOffset()
	{
		return m_angularOffset;
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

	/// Set the position correction factor in the range [0,1].
	public void SetCorrectionFactor(float factor)
	{
		Debug.Assert(GlobalMembers.b2IsValid(factor) && 0.0f <= factor && factor <= 1.0f);
		m_correctionFactor = factor;
	}

	/// Get the position correction factor in the range [0,1].


	public float GetCorrectionFactor()
	{
		return m_correctionFactor;
	}

	/// Dump to Console.Write
	public override void Dump()
	{
		int indexA = m_bodyA.m_islandIndex;
		int indexB = m_bodyB.m_islandIndex;

		Console.Write("  b2MotorJointDef jd;\n");
		Console.Write("  jd.bodyA = bodies[%d];\n", indexA);
		Console.Write("  jd.bodyB = bodies[%d];\n", indexB);
		Console.Write("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		Console.Write("  jd.linearOffset.Set(%.15lef, %.15lef);\n", m_linearOffset.x, m_linearOffset.y);
		Console.Write("  jd.angularOffset = %.15lef;\n", m_angularOffset);
		Console.Write("  jd.maxForce = %.15lef;\n", m_maxForce);
		Console.Write("  jd.maxTorque = %.15lef;\n", m_maxTorque);
		Console.Write("  jd.correctionFactor = %.15lef;\n", m_correctionFactor);
		Console.Write("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
	}



    //	friend class b2Joint;

    internal b2MotorJoint(b2MotorJointDef def) : base(def)
	{


		m_linearOffset = def.linearOffset;
		m_angularOffset = def.angularOffset;

		m_linearImpulse.SetZero();
		m_angularImpulse = 0.0f;

		m_maxForce = def.maxForce;
		m_maxTorque = def.maxTorque;
		m_correctionFactor = def.correctionFactor;
	}

	protected override void InitVelocityConstraints(b2SolverData data)
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

		// Compute the effective mass matrix.


		m_rA = GlobalMembers.b2Mul(qA, m_linearOffset - m_localCenterA);


		m_rB = GlobalMembers.b2Mul(qB, -m_localCenterB);

		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]



		float mA = m_invMassA;
		float mB = m_invMassB;
		float iA = m_invIA;
		float iB = m_invIB;

		// Upper 2 by 2 of K for point to point
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



		m_linearError = cB + m_rB - cA - m_rA;
		m_angularError = aB - aA - m_angularOffset;

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
	protected override void SolveVelocityConstraints(b2SolverData data)
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
		float inv_h = data.step.inv_dt;

		{
		// Solve angular friction
			float Cdot = wB - wA + inv_h * m_correctionFactor * m_angularError;
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
			b2Vec2 Cdot = vB + GlobalMembers.b2Cross(wB, m_rB) - vA - GlobalMembers.b2Cross(wA, m_rA) + inv_h * m_correctionFactor * m_linearError;

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
	protected override bool SolvePositionConstraints(b2SolverData data)
	{
		

		return true;
	}

	// Solver shared
	protected b2Vec2 m_linearOffset = new b2Vec2();
	protected float m_angularOffset;
	protected b2Vec2 m_linearImpulse = new b2Vec2();
	protected float m_angularImpulse;
	protected float m_maxForce;
	protected float m_maxTorque;
	protected float m_correctionFactor;

	// Solver temp
	protected int m_indexA;
	protected int m_indexB;
	protected b2Vec2 m_rA = new b2Vec2();
	protected b2Vec2 m_rB = new b2Vec2();
	protected b2Vec2 m_localCenterA = new b2Vec2();
	protected b2Vec2 m_localCenterB = new b2Vec2();
	protected b2Vec2 m_linearError = new b2Vec2();
	protected float m_angularError;
	protected float m_invMassA;
	protected float m_invMassB;
	protected float m_invIA;
	protected float m_invIB;
	protected b2Mat22 m_linearMass = new b2Mat22();
	protected float m_angularMass;
}
