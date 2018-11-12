using System;
using System.Diagnostics;

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
public class b2PulleyJointDef : b2JointDef
{
	public b2PulleyJointDef()
	{
		type = b2JointType.e_pulleyJoint;
		groundAnchorA.Set(-1.0f, 1.0f);
		groundAnchorB.Set(1.0f, 1.0f);
		localAnchorA.Set(-1.0f, 0.0f);
		localAnchorB.Set(1.0f, 0.0f);
		lengthA = 0.0f;
		lengthB = 0.0f;
		ratio = 1.0f;
		collideConnected = true;
	}

	/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.

	// Pulley:
	// length1 = norm(p1 - s1)
	// length2 = norm(p2 - s2)
	// C0 = (length1 + ratio * length2)_initial
	// C = C0 - (length1 + ratio * length2)
	// u1 = (p1 - s1) / norm(p1 - s1)
	// u2 = (p2 - s2) / norm(p2 - s2)
	// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
	// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
	// K = J * invM * JT
	//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

	public void Initialize(b2Body bA, b2Body bB, b2Vec2 groundA, b2Vec2 groundB, b2Vec2 anchorA, b2Vec2 anchorB, float r)
	{
		bodyA = bA;
		bodyB = bB;


		groundAnchorA = groundA;


		groundAnchorB = groundB;


		localAnchorA = bodyA.GetLocalPoint(anchorA);


		localAnchorB = bodyB.GetLocalPoint(anchorB);
		b2Vec2 dA = anchorA - groundA;
		lengthA = dA.Length();
		b2Vec2 dB = anchorB - groundB;
		lengthB = dB.Length();
		ratio = r;
		Debug.Assert(ratio > float.Epsilon);
	}

	/// The first ground anchor in world coordinates. This point never moves.
	public b2Vec2 groundAnchorA = new b2Vec2();

	/// The second ground anchor in world coordinates. This point never moves.
	public b2Vec2 groundAnchorB = new b2Vec2();

	/// The local anchor point relative to bodyA's origin.
	public b2Vec2 localAnchorA = new b2Vec2();

	/// The local anchor point relative to bodyB's origin.
	public b2Vec2 localAnchorB = new b2Vec2();

	/// The a reference length for the segment attached to bodyA.
	public float lengthA;

	/// The a reference length for the segment attached to bodyB.
	public float lengthB;

	/// The pulley ratio, used to simulate a block-and-tackle.
	public float ratio;
}

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
public class b2PulleyJoint : b2Joint
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
		b2Vec2 P = m_impulse * m_uB;
		return inv_dt * P;
	}


	public override float GetReactionTorque(float inv_dt)
	{
		return 0.0f;
	}

	/// Get the first ground anchor.


	public b2Vec2 GetGroundAnchorA()
	{
		return m_groundAnchorA;
	}

	/// Get the second ground anchor.


	public b2Vec2 GetGroundAnchorB()
	{
		return m_groundAnchorB;
	}

	/// Get the current length of the segment attached to bodyA.


	public float GetLengthA()
	{
		return m_lengthA;
	}

	/// Get the current length of the segment attached to bodyB.


	public float GetLengthB()
	{
		return m_lengthB;
	}

	/// Get the pulley ratio.


	public float GetRatio()
	{
		return m_ratio;
	}

	/// Get the current length of the segment attached to bodyA.


	public float GetCurrentLengthA()
	{
		b2Vec2 p = m_bodyA.GetWorldPoint(m_localAnchorA);


		b2Vec2 s = new b2Vec2(m_groundAnchorA);
		b2Vec2 d = p - s;
		return d.Length();
	}

	/// Get the current length of the segment attached to bodyB.


	public float GetCurrentLengthB()
	{
		b2Vec2 p = m_bodyB.GetWorldPoint(m_localAnchorB);


		b2Vec2 s = new b2Vec2(m_groundAnchorB);
		b2Vec2 d = p - s;
		return d.Length();
	}

	/// Dump joint to dmLog
	public override void Dump()
	{
		int indexA = m_bodyA.m_islandIndex;
		int indexB = m_bodyB.m_islandIndex;

		Console.Write("  b2PulleyJointDef jd;\n");
		Console.Write("  jd.bodyA = bodies[%d];\n", indexA);
		Console.Write("  jd.bodyB = bodies[%d];\n", indexB);
		Console.Write("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		Console.Write("  jd.groundAnchorA.Set(%.15lef, %.15lef);\n", m_groundAnchorA.x, m_groundAnchorA.y);
		Console.Write("  jd.groundAnchorB.Set(%.15lef, %.15lef);\n", m_groundAnchorB.x, m_groundAnchorB.y);
		Console.Write("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
		Console.Write("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
		Console.Write("  jd.lengthA = %.15lef;\n", m_lengthA);
		Console.Write("  jd.lengthB = %.15lef;\n", m_lengthB);
		Console.Write("  jd.ratio = %.15lef;\n", m_ratio);
		Console.Write("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
	}

	/// Implement b2Joint::ShiftOrigin
	public override void ShiftOrigin(b2Vec2 newOrigin)
	{
		m_groundAnchorA -= newOrigin;
		m_groundAnchorB -= newOrigin;
	}



    internal b2PulleyJoint(b2PulleyJointDef def) : base(def)
	{
		m_groundAnchorA = def.groundAnchorA;


		m_groundAnchorB = def.groundAnchorB;


		m_localAnchorA = def.localAnchorA;


		m_localAnchorB = def.localAnchorB;

		m_lengthA = def.lengthA;
		m_lengthB = def.lengthB;

		Debug.Assert(def.ratio != 0.0f);
		m_ratio = def.ratio;

		m_constant = def.lengthA + m_ratio * def.lengthB;

		m_impulse = 0.0f;
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



		m_rA = GlobalMembers.b2Mul(qA, m_localAnchorA - m_localCenterA);


		m_rB = GlobalMembers.b2Mul(qB, m_localAnchorB - m_localCenterB);

		// Get the pulley axes.


		m_uA = cA + m_rA - m_groundAnchorA;


		m_uB = cB + m_rB - m_groundAnchorB;

		float lengthA = m_uA.Length();
		float lengthB = m_uB.Length();

		if (lengthA > 10.0f * DefineConstants.b2_linearSlop)
		{
			m_uA *= 1.0f / lengthA;
		}
		else
		{
			m_uA.SetZero();
		}

		if (lengthB > 10.0f * DefineConstants.b2_linearSlop)
		{
			m_uB *= 1.0f / lengthB;
		}
		else
		{
			m_uB.SetZero();
		}

		// Compute effective mass.
		float ruA = GlobalMembers.b2Cross(m_rA, m_uA);
		float ruB = GlobalMembers.b2Cross(m_rB, m_uB);

		float mA = m_invMassA + m_invIA * ruA * ruA;
		float mB = m_invMassB + m_invIB * ruB * ruB;

		m_mass = mA + m_ratio * m_ratio * mB;

		if (m_mass > 0.0f)
		{
			m_mass = 1.0f / m_mass;
		}

		if (data.step.warmStarting)
		{
			// Scale impulses to support variable time steps.
			m_impulse *= data.step.dtRatio;

			// Warm starting.
			b2Vec2 PA = -(m_impulse) * m_uA;
			b2Vec2 PB = (-m_ratio * m_impulse) * m_uB;

			vA += m_invMassA * PA;
			wA += m_invIA * GlobalMembers.b2Cross(m_rA, PA);
			vB += m_invMassB * PB;
			wB += m_invIB * GlobalMembers.b2Cross(m_rB, PB);
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
	protected override void SolveVelocityConstraints(b2SolverData data)
	{
		b2Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		b2Vec2 vpA = vA + GlobalMembers.b2Cross(wA, m_rA);
		b2Vec2 vpB = vB + GlobalMembers.b2Cross(wB, m_rB);

		float Cdot = -GlobalMembers.b2Dot(m_uA, vpA) - m_ratio * GlobalMembers.b2Dot(m_uB, vpB);
		float impulse = -m_mass * Cdot;
		m_impulse += impulse;

		b2Vec2 PA = -impulse * m_uA;
		b2Vec2 PB = -m_ratio * impulse * m_uB;
		vA += m_invMassA * PA;
		wA += m_invIA * GlobalMembers.b2Cross(m_rA, PA);
		vB += m_invMassB * PB;
		wB += m_invIB * GlobalMembers.b2Cross(m_rB, PB);



		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;


		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
	protected override bool SolvePositionConstraints(b2SolverData data)
	{
		b2Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		b2Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;

		b2Rot qA = new b2Rot(aA);
		b2Rot qB = new b2Rot(aB);

		b2Vec2 rA = GlobalMembers.b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = GlobalMembers.b2Mul(qB, m_localAnchorB - m_localCenterB);

		// Get the pulley axes.
		b2Vec2 uA = cA + rA - m_groundAnchorA;
		b2Vec2 uB = cB + rB - m_groundAnchorB;

		float lengthA = uA.Length();
		float lengthB = uB.Length();

		if (lengthA > 10.0f * DefineConstants.b2_linearSlop)
		{
			uA *= 1.0f / lengthA;
		}
		else
		{
			uA.SetZero();
		}

		if (lengthB > 10.0f * DefineConstants.b2_linearSlop)
		{
			uB *= 1.0f / lengthB;
		}
		else
		{
			uB.SetZero();
		}

		// Compute effective mass.
		float ruA = GlobalMembers.b2Cross(rA, uA);
		float ruB = GlobalMembers.b2Cross(rB, uB);

		float mA = m_invMassA + m_invIA * ruA * ruA;
		float mB = m_invMassB + m_invIB * ruB * ruB;

		float mass = mA + m_ratio * m_ratio * mB;

		if (mass > 0.0f)
		{
			mass = 1.0f / mass;
		}

		float C = m_constant - lengthA - m_ratio * lengthB;
		float linearError = GlobalMembers.b2Abs(C);

		float impulse = -mass * C;

		b2Vec2 PA = -impulse * uA;
		b2Vec2 PB = -m_ratio * impulse * uB;

		cA += m_invMassA * PA;
		aA += m_invIA * GlobalMembers.b2Cross(rA, PA);
		cB += m_invMassB * PB;
		aB += m_invIB * GlobalMembers.b2Cross(rB, PB);



		data.positions[m_indexA].c = cA;
		data.positions[m_indexA].a = aA;


		data.positions[m_indexB].c = cB;
		data.positions[m_indexB].a = aB;

		return linearError < DefineConstants.b2_linearSlop;
	}

	protected b2Vec2 m_groundAnchorA = new b2Vec2();
	protected b2Vec2 m_groundAnchorB = new b2Vec2();
	protected float m_lengthA;
	protected float m_lengthB;

	// Solver shared
	protected b2Vec2 m_localAnchorA = new b2Vec2();
	protected b2Vec2 m_localAnchorB = new b2Vec2();
	protected float m_constant;
	protected float m_ratio;
	protected float m_impulse;

	// Solver temp
	protected int m_indexA;
	protected int m_indexB;
	protected b2Vec2 m_uA = new b2Vec2();
	protected b2Vec2 m_uB = new b2Vec2();
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
