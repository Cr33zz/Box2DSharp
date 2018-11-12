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
/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
public class b2DistanceJointDef : b2JointDef
{
	public b2DistanceJointDef()
	{
		type = b2JointType.e_distanceJoint;
		localAnchorA.Set(0.0f, 0.0f);
		localAnchorB.Set(0.0f, 0.0f);
		length = 1.0f;
		frequencyHz = 0.0f;
		dampingRatio = 0.0f;
	}

	/// Initialize the bodies, anchors, and length using the world
	/// anchors.

	// 1-D constrained system
	// m (v2 - v1) = lambda
	// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
	// x2 = x1 + h * v2

	// 1-D mass-damper-spring system
	// m (v2 - v1) + h * d * v2 + h * k * 

	// C = norm(p2 - p1) - L
	// u = (p2 - p1) / norm(p2 - p1)
	// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
	// J = [-u -cross(r1, u) u cross(r2, u)]
	// K = J * invM * JT
	//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

	public void Initialize(b2Body b1, b2Body b2, b2Vec2 anchor1, b2Vec2 anchor2)
	{
		bodyA = b1;
		bodyB = b2;
		localAnchorA = bodyA.GetLocalPoint(anchor1);
		localAnchorB = bodyB.GetLocalPoint(anchor2);
		b2Vec2 d = anchor2 - anchor1;
		length = d.Length();
	}

	/// The local anchor point relative to bodyA's origin.
	public b2Vec2 localAnchorA = new b2Vec2();

	/// The local anchor point relative to bodyB's origin.
	public b2Vec2 localAnchorB = new b2Vec2();

	/// The natural length between the anchor points.
	public float length;

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	public float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	public float dampingRatio;
}

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
public class b2DistanceJoint : b2Joint
{

	public override b2Vec2 GetAnchorA()
	{
		return m_bodyA.GetWorldPoint(m_localAnchorA);
	}
	public override b2Vec2 GetAnchorB()
	{
		return m_bodyB.GetWorldPoint(m_localAnchorB);
	}

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	public override b2Vec2 GetReactionForce(float inv_dt)
	{
		b2Vec2 F = (inv_dt * m_impulse) * m_u;
		return F;
	}

	/// Get the reaction torque given the inverse time step.
	/// Unit is N*m. This is always zero for a distance joint.
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

	/// Set/get the natural length.
	/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
	public void SetLength(float length)
	{
		m_length = length;
	}
	public float GetLength()
	{
		return m_length;
	}

	/// Set/get frequency in Hz.
	public void SetFrequency(float hz)
	{
		m_frequencyHz = hz;
	}
	public float GetFrequency()
	{
		return m_frequencyHz;
	}

	/// Set/get damping ratio.
	public void SetDampingRatio(float ratio)
	{
		m_dampingRatio = ratio;
	}
	public float GetDampingRatio()
	{
		return m_dampingRatio;
	}

	/// Dump joint to dmLog
	public override void Dump()
	{
		int indexA = m_bodyA.m_islandIndex;
		int indexB = m_bodyB.m_islandIndex;

		GlobalMembers.b2Log("  b2DistanceJointDef jd;\n");
		GlobalMembers.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
		GlobalMembers.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
		GlobalMembers.b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		GlobalMembers.b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
		GlobalMembers.b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
		GlobalMembers.b2Log("  jd.length = %.15lef;\n", m_length);
		GlobalMembers.b2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
		GlobalMembers.b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
		GlobalMembers.b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
	}

	internal b2DistanceJoint(b2DistanceJointDef def) : base(def)
	{


		m_localAnchorA = def.localAnchorA;


		m_localAnchorB = def.localAnchorB;
		m_length = def.length;
		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;
		m_impulse = 0.0f;
		m_gamma = 0.0f;
		m_bias = 0.0f;
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

		// Handle singularity.
		float length = m_u.Length();
		if (length > DefineConstants.b2_linearSlop)
		{
			m_u *= 1.0f / length;
		}
		else
		{
			m_u.Set(0.0f, 0.0f);
		}

		float crAu = GlobalMembers.b2Cross(m_rA, m_u);
		float crBu = GlobalMembers.b2Cross(m_rB, m_u);
		float invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

		// Compute the effective mass matrix.
		m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

		if (m_frequencyHz > 0.0f)
		{
			float C = length - m_length;

			// Frequency
			float omega = 2.0f * DefineConstants.b2_pi * m_frequencyHz;

			// Damping coefficient
			float d = 2.0f * m_mass * m_dampingRatio * omega;

			// Spring stiffness
			float k = m_mass * omega * omega;

			// magic formulas
			float h = data.step.dt;
			m_gamma = h * (d + h * k);
			m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
			m_bias = C * h * k * m_gamma;

			invMass += m_gamma;
			m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
		}
		else
		{
			m_gamma = 0.0f;
			m_bias = 0.0f;
		}

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
		float Cdot = GlobalMembers.b2Dot(m_u, vpB - vpA);

		float impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
		m_impulse += impulse;

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
		if (m_frequencyHz > 0.0f)
		{
			// There is no position correction for soft distance constraints.
			return true;
		}

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
		float C = length - m_length;
		C = GlobalMembers.b2Clamp(C, -DefineConstants.b2_maxLinearCorrection, DefineConstants.b2_maxLinearCorrection);

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

		return GlobalMembers.b2Abs(C) < DefineConstants.b2_linearSlop;
	}

	protected float m_frequencyHz;
	protected float m_dampingRatio;
	protected float m_bias;

	// Solver shared
	protected b2Vec2 m_localAnchorA = new b2Vec2();
	protected b2Vec2 m_localAnchorB = new b2Vec2();
	protected float m_gamma;
	protected float m_impulse;
	protected float m_length;

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
}
