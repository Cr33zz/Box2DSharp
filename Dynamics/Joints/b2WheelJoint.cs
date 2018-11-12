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
/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
public class b2WheelJointDef : b2JointDef
{
	public b2WheelJointDef()
	{
		type = b2JointType.e_wheelJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		localAxisA.Set(1.0f, 0.0f);
		enableMotor = false;
		maxMotorTorque = 0.0f;
		motorSpeed = 0.0f;
		frequencyHz = 2.0f;
		dampingRatio = 0.7f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.

	// Linear constraint (point-to-line)
	// d = pB - pA = xB + rB - xA - rA
	// C = dot(ay, d)
	// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
	//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
	// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

	// Spring linear constraint
	// C = dot(ax, d)
	// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
	// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

	// Motor rotational constraint
	// Cdot = wB - wA
	// J = [0 0 -1 0 0 1]

	public void Initialize(b2Body bA, b2Body bB, b2Vec2 anchor, b2Vec2 axis)
	{
		bodyA = bA;
		bodyB = bB;


		localAnchorA = bodyA.GetLocalPoint(anchor);


		localAnchorB = bodyB.GetLocalPoint(anchor);


		localAxisA = bodyA.GetLocalVector(axis);
	}

	/// The local anchor point relative to bodyA's origin.
	public b2Vec2 localAnchorA = new b2Vec2();

	/// The local anchor point relative to bodyB's origin.
	public b2Vec2 localAnchorB = new b2Vec2();

	/// The local translation axis in bodyA.
	public b2Vec2 localAxisA = new b2Vec2();

	/// Enable/disable the joint motor.
	public bool enableMotor;

	/// The maximum motor torque, usually in N-m.
	public float maxMotorTorque;

	/// The desired motor speed in radians per second.
	public float motorSpeed;

	/// Suspension frequency, zero indicates no suspension
	public float frequencyHz;

	/// Suspension damping ratio, one indicates critical damping
	public float dampingRatio;
}

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper.
/// This joint is designed for vehicle suspensions.
public class b2WheelJoint : b2Joint
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
		return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
	}


	public override float GetReactionTorque(float inv_dt)
	{
		return inv_dt * m_motorImpulse;
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

	/// The local joint axis relative to bodyA.


	public b2Vec2 GetLocalAxisA()
	{
		return m_localXAxisA;
	}

	/// Get the current joint translation, usually in meters.


	public float GetJointTranslation()
	{
		b2Body bA = m_bodyA;
		b2Body bB = m_bodyB;

		b2Vec2 pA = bA.GetWorldPoint(m_localAnchorA);
		b2Vec2 pB = bB.GetWorldPoint(m_localAnchorB);
		b2Vec2 d = pB - pA;
		b2Vec2 axis = bA.GetWorldVector(m_localXAxisA);

		float translation = GlobalMembers.b2Dot(d, axis);
		return translation;
	}

	/// Get the current joint linear speed, usually in meters per second.


	public float GetJointLinearSpeed()
	{
		b2Body bA = m_bodyA;
		b2Body bB = m_bodyB;

		b2Vec2 rA = GlobalMembers.b2Mul(bA.m_xf.q, m_localAnchorA - bA.m_sweep.localCenter);
		b2Vec2 rB = GlobalMembers.b2Mul(bB.m_xf.q, m_localAnchorB - bB.m_sweep.localCenter);
		b2Vec2 p1 = bA.m_sweep.c + rA;
		b2Vec2 p2 = bB.m_sweep.c + rB;
		b2Vec2 d = p2 - p1;
		b2Vec2 axis = GlobalMembers.b2Mul(bA.m_xf.q, m_localXAxisA);



		b2Vec2 vA = new b2Vec2(bA.m_linearVelocity);


		b2Vec2 vB = new b2Vec2(bB.m_linearVelocity);
		float wA = bA.m_angularVelocity;
		float wB = bB.m_angularVelocity;

		float speed = GlobalMembers.b2Dot(d, GlobalMembers.b2Cross(wA, axis)) + GlobalMembers.b2Dot(axis, vB + GlobalMembers.b2Cross(wB, rB) - vA - GlobalMembers.b2Cross(wA, rA));
		return speed;
	}

	/// Get the current joint angle in radians.


	public float GetJointAngle()
	{
		b2Body bA = m_bodyA;
		b2Body bB = m_bodyB;
		return bB.m_sweep.a - bA.m_sweep.a;
	}

	/// Get the current joint angular speed in radians per second.


	public float GetJointAngularSpeed()
	{
		float wA = m_bodyA.m_angularVelocity;
		float wB = m_bodyB.m_angularVelocity;
		return wB - wA;
	}

	/// Is the joint motor enabled?


	public bool IsMotorEnabled()
	{
		return m_enableMotor;
	}

	/// Enable/disable the joint motor.
	public void EnableMotor(bool flag)
	{
		if (flag != m_enableMotor)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_enableMotor = flag;
		}
	}

	/// Set the motor speed, usually in radians per second.
	public void SetMotorSpeed(float speed)
	{
		if (speed != m_motorSpeed)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_motorSpeed = speed;
		}
	}

	/// Get the motor speed, usually in radians per second.


	public float GetMotorSpeed()
	{
		return m_motorSpeed;
	}

	/// Set/Get the maximum motor force, usually in N-m.
	public void SetMaxMotorTorque(float torque)
	{
		if (torque != m_maxMotorTorque)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_maxMotorTorque = torque;
		}
	}


	public float GetMaxMotorTorque()
	{
		return m_maxMotorTorque;
	}

	/// Get the current motor torque given the inverse time step, usually in N-m.


	public float GetMotorTorque(float inv_dt)
	{
		return inv_dt * m_motorImpulse;
	}

	/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
	public void SetSpringFrequencyHz(float hz)
	{
		m_frequencyHz = hz;
	}


	public float GetSpringFrequencyHz()
	{
		return m_frequencyHz;
	}

	/// Set/Get the spring damping ratio
	public void SetSpringDampingRatio(float ratio)
	{
		m_dampingRatio = ratio;
	}


	public float GetSpringDampingRatio()
	{
		return m_dampingRatio;
	}

	/// Dump to Console.Write
	public override void Dump()
	{
		int indexA = m_bodyA.m_islandIndex;
		int indexB = m_bodyB.m_islandIndex;

		Console.Write("  b2WheelJointDef jd;\n");
		Console.Write("  jd.bodyA = bodies[%d];\n", indexA);
		Console.Write("  jd.bodyB = bodies[%d];\n", indexB);
		Console.Write("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		Console.Write("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
		Console.Write("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
		Console.Write("  jd.localAxisA.Set(%.15lef, %.15lef);\n", m_localXAxisA.x, m_localXAxisA.y);
		Console.Write("  jd.enableMotor = bool(%d);\n", m_enableMotor);
		Console.Write("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
		Console.Write("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
		Console.Write("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
		Console.Write("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
		Console.Write("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
	}



    //	friend class b2Joint;
    internal b2WheelJoint(b2WheelJointDef def) : base(def)
	{


		m_localAnchorA = def.localAnchorA;


		m_localAnchorB = def.localAnchorB;


		m_localXAxisA = def.localAxisA;
		m_localYAxisA = GlobalMembers.b2Cross(1.0f, m_localXAxisA);

		m_mass = 0.0f;
		m_impulse = 0.0f;
		m_motorMass = 0.0f;
		m_motorImpulse = 0.0f;
		m_springMass = 0.0f;
		m_springImpulse = 0.0f;

		m_maxMotorTorque = def.maxMotorTorque;
		m_motorSpeed = def.motorSpeed;
		m_enableMotor = def.enableMotor;

		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;

		m_bias = 0.0f;
		m_gamma = 0.0f;

		m_ax.SetZero();
		m_ay.SetZero();
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

		float mA = m_invMassA;
		float mB = m_invMassB;
		float iA = m_invIA;
		float iB = m_invIB;

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

		// Compute the effective masses.
		b2Vec2 rA = GlobalMembers.b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = GlobalMembers.b2Mul(qB, m_localAnchorB - m_localCenterB);
		b2Vec2 d = cB + rB - cA - rA;

		{
		// Point to line constraint


			m_ay = GlobalMembers.b2Mul(qA, m_localYAxisA);
			m_sAy = GlobalMembers.b2Cross(d + rA, m_ay);
			m_sBy = GlobalMembers.b2Cross(rB, m_ay);

			m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

			if (m_mass > 0.0f)
			{
				m_mass = 1.0f / m_mass;
			}
		}

		// Spring constraint
		m_springMass = 0.0f;
		m_bias = 0.0f;
		m_gamma = 0.0f;
		if (m_frequencyHz > 0.0f)
		{


			m_ax = GlobalMembers.b2Mul(qA, m_localXAxisA);
			m_sAx = GlobalMembers.b2Cross(d + rA, m_ax);
			m_sBx = GlobalMembers.b2Cross(rB, m_ax);

			float invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

			if (invMass > 0.0f)
			{
				m_springMass = 1.0f / invMass;

				float C = GlobalMembers.b2Dot(d, m_ax);

				// Frequency
				float omega = 2.0f * DefineConstants.b2_pi * m_frequencyHz;

				// Damping coefficient
				float damp = 2.0f * m_springMass * m_dampingRatio * omega;

				// Spring stiffness
				float k = m_springMass * omega * omega;

				// magic formulas
				float h = data.step.dt;
				m_gamma = h * (damp + h * k);
				if (m_gamma > 0.0f)
				{
					m_gamma = 1.0f / m_gamma;
				}

				m_bias = C * h * k * m_gamma;

				m_springMass = invMass + m_gamma;
				if (m_springMass > 0.0f)
				{
					m_springMass = 1.0f / m_springMass;
				}
			}
		}
		else
		{
			m_springImpulse = 0.0f;
		}

		// Rotational motor
		if (m_enableMotor)
		{
			m_motorMass = iA + iB;
			if (m_motorMass > 0.0f)
			{
				m_motorMass = 1.0f / m_motorMass;
			}
		}
		else
		{
			m_motorMass = 0.0f;
			m_motorImpulse = 0.0f;
		}

		if (data.step.warmStarting)
		{
			// Account for variable time step.
			m_impulse *= data.step.dtRatio;
			m_springImpulse *= data.step.dtRatio;
			m_motorImpulse *= data.step.dtRatio;

			b2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
			float LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
			float LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

			vA -= m_invMassA * P;
			wA -= m_invIA * LA;

			vB += m_invMassB * P;
			wB += m_invIB * LB;
		}
		else
		{
			m_impulse = 0.0f;
			m_springImpulse = 0.0f;
			m_motorImpulse = 0.0f;
		}



		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;


		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
	protected override void SolveVelocityConstraints(b2SolverData data)
	{
		float mA = m_invMassA;
		float mB = m_invMassB;
		float iA = m_invIA;
		float iB = m_invIB;

		b2Vec2 vA = data.velocities[m_indexA].v;
		float wA = data.velocities[m_indexA].w;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		{
		// Solve spring constraint
			float Cdot = GlobalMembers.b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
			float impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
			m_springImpulse += impulse;

			b2Vec2 P = impulse * m_ax;
			float LA = impulse * m_sAx;
			float LB = impulse * m_sBx;

			vA -= mA * P;
			wA -= iA * LA;

			vB += mB * P;
			wB += iB * LB;
		}

		{
		// Solve rotational motor constraint
			float Cdot = wB - wA - m_motorSpeed;
			float impulse = -m_motorMass * Cdot;

			float oldImpulse = m_motorImpulse;
			float maxImpulse = data.step.dt * m_maxMotorTorque;
			m_motorImpulse = GlobalMembers.b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_motorImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		{
		// Solve point to line constraint
			float Cdot = GlobalMembers.b2Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
			float impulse = -m_mass * Cdot;
			m_impulse += impulse;

			b2Vec2 P = impulse * m_ay;
			float LA = impulse * m_sAy;
			float LB = impulse * m_sBy;

			vA -= mA * P;
			wA -= iA * LA;

			vB += mB * P;
			wB += iB * LB;
		}



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
		b2Vec2 d = (cB - cA) + rB - rA;

		b2Vec2 ay = GlobalMembers.b2Mul(qA, m_localYAxisA);

		float sAy = GlobalMembers.b2Cross(d + rA, ay);
		float sBy = GlobalMembers.b2Cross(rB, ay);

		float C = GlobalMembers.b2Dot(d, ay);

		float k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

		float impulse;
		if (k != 0.0f)
		{
			impulse = - C / k;
		}
		else
		{
			impulse = 0.0f;
		}

		b2Vec2 P = impulse * ay;
		float LA = impulse * sAy;
		float LB = impulse * sBy;

		cA -= m_invMassA * P;
		aA -= m_invIA * LA;
		cB += m_invMassB * P;
		aB += m_invIB * LB;



		data.positions[m_indexA].c = cA;
		data.positions[m_indexA].a = aA;


		data.positions[m_indexB].c = cB;
		data.positions[m_indexB].a = aB;

		return GlobalMembers.b2Abs(C) <= DefineConstants.b2_linearSlop;
	}

	protected float m_frequencyHz;
	protected float m_dampingRatio;

	// Solver shared
	protected b2Vec2 m_localAnchorA = new b2Vec2();
	protected b2Vec2 m_localAnchorB = new b2Vec2();
	protected b2Vec2 m_localXAxisA = new b2Vec2();
	protected b2Vec2 m_localYAxisA = new b2Vec2();

	protected float m_impulse;
	protected float m_motorImpulse;
	protected float m_springImpulse;

	protected float m_maxMotorTorque;
	protected float m_motorSpeed;
	protected bool m_enableMotor;

	// Solver temp
	protected int m_indexA;
	protected int m_indexB;
	protected b2Vec2 m_localCenterA = new b2Vec2();
	protected b2Vec2 m_localCenterB = new b2Vec2();
	protected float m_invMassA;
	protected float m_invMassB;
	protected float m_invIA;
	protected float m_invIB;

	protected b2Vec2 m_ax = new b2Vec2();
	protected b2Vec2 m_ay = new b2Vec2();
	protected float m_sAx;
	protected float m_sBx;
	protected float m_sAy;
	protected float m_sBy;

	protected float m_mass;
	protected float m_motorMass;
	protected float m_springMass;

	protected float m_bias;
	protected float m_gamma;
}
