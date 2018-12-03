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





























/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
public class b2PrismaticJointDef : b2JointDef
{
	public b2PrismaticJointDef()
	{
		type = b2JointType.e_prismaticJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		localAxisA.Set(1.0f, 0.0f);
		referenceAngle = 0.0f;
		enableLimit = false;
		lowerTranslation = 0.0f;
		upperTranslation = 0.0f;
		enableMotor = false;
		maxMotorForce = 0.0f;
		motorSpeed = 0.0f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and unit world axis.

	// Linear constraint (point-to-line)
	// d = p2 - p1 = x2 + r2 - x1 - r1
	// C = dot(perp, d)
	// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
	//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
	// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
	//
	// Angular constraint
	// C = a2 - a1 + a_initial
	// Cdot = w2 - w1
	// J = [0 0 -1 0 0 1]
	//
	// K = J * invM * JT
	//
	// J = [-a -s1 a s2]
	//     [0  -1  0  1]
	// a = perp
	// s1 = cross(d + r1, a) = cross(p2 - x1, a)
	// s2 = cross(r2, a) = cross(p2 - x2, a)


	// Motor/Limit linear constraint
	// C = dot(ax1, d)
	// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
	// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

	// Block Solver
	// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
	// when the mass has poor distribution (leading to large torques about the joint anchor points).
	//
	// The Jacobian has 3 rows:
	// J = [-uT -s1 uT s2] // linear
	//     [0   -1   0  1] // angular
	//     [-vT -a1 vT a2] // limit
	//
	// u = perp
	// v = axis
	// s1 = cross(d + r1, u), s2 = cross(r2, u)
	// a1 = cross(d + r1, v), a2 = cross(r2, v)

	// M * (v2 - v1) = JT * df
	// J * v2 = bias
	//
	// v2 = v1 + invM * JT * df
	// J * (v1 + invM * JT * df) = bias
	// K * df = bias - J * v1 = -Cdot
	// K = J * invM * JT
	// Cdot = J * v1 - bias
	//
	// Now solve for f2.
	// df = f2 - f1
	// K * (f2 - f1) = -Cdot
	// f2 = invK * (-Cdot) + f1
	//
	// Clamp accumulated limit impulse.
	// lower: f2(3) = max(f2(3), 0)
	// upper: f2(3) = min(f2(3), 0)
	//
	// Solve for correct f2(1:2)
	// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
	//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
	// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
	// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
	//
	// Now compute impulse to be applied:
	// df = f2 - f1

	public void Initialize(b2Body bA, b2Body bB, b2Vec2 anchor, b2Vec2 axis)
	{
		bodyA = bA;
		bodyB = bB;


		localAnchorA = bodyA.GetLocalPoint(anchor);


		localAnchorB = bodyB.GetLocalPoint(anchor);


		localAxisA = bodyA.GetLocalVector(axis);
		referenceAngle = bodyB.GetAngle() - bodyA.GetAngle();
	}

	/// The local anchor point relative to bodyA's origin.
	public b2Vec2 localAnchorA = new b2Vec2();

	/// The local anchor point relative to bodyB's origin.
	public b2Vec2 localAnchorB = new b2Vec2();

	/// The local translation unit axis in bodyA.
	public b2Vec2 localAxisA = new b2Vec2();

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	public float referenceAngle;

	/// Enable/disable the joint limit.
	public bool enableLimit;

	/// The lower translation limit, usually in meters.
	public float lowerTranslation;

	/// The upper translation limit, usually in meters.
	public float upperTranslation;

	/// Enable/disable the joint motor.
	public bool enableMotor;

	/// The maximum motor torque, usually in N-m.
	public float maxMotorForce;

	/// The desired motor speed in radians per second.
	public float motorSpeed;
}

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
public class b2PrismaticJoint : b2Joint
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
		return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
	}


	public override float GetReactionTorque(float inv_dt)
	{
		return inv_dt * m_impulse.y;
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

	/// Get the reference angle.


	public float GetReferenceAngle()
	{
		return m_referenceAngle;
	}

	/// Get the current joint translation, usually in meters.


	public float GetJointTranslation()
	{
		b2Vec2 pA = m_bodyA.GetWorldPoint(m_localAnchorA);
		b2Vec2 pB = m_bodyB.GetWorldPoint(m_localAnchorB);
		b2Vec2 d = pB - pA;
		b2Vec2 axis = m_bodyA.GetWorldVector(m_localXAxisA);

		float translation = Utils.b2Dot(d, axis);
		return translation;
	}

	/// Get the current joint translation speed, usually in meters per second.


	public float GetJointSpeed()
	{
		b2Body bA = m_bodyA;
		b2Body bB = m_bodyB;

		b2Vec2 rA = Utils.b2Mul(bA.m_xf.q, m_localAnchorA - bA.m_sweep.localCenter);
		b2Vec2 rB = Utils.b2Mul(bB.m_xf.q, m_localAnchorB - bB.m_sweep.localCenter);
		b2Vec2 p1 = bA.m_sweep.c + rA;
		b2Vec2 p2 = bB.m_sweep.c + rB;
		b2Vec2 d = p2 - p1;
		b2Vec2 axis = Utils.b2Mul(bA.m_xf.q, m_localXAxisA);



		b2Vec2 vA = new b2Vec2(bA.m_linearVelocity);


		b2Vec2 vB = new b2Vec2(bB.m_linearVelocity);
		float wA = bA.m_angularVelocity;
		float wB = bB.m_angularVelocity;

		float speed = Utils.b2Dot(d, Utils.b2Cross(wA, axis)) + Utils.b2Dot(axis, vB + Utils.b2Cross(wB, rB) - vA - Utils.b2Cross(wA, rA));
		return speed;
	}

	/// Is the joint limit enabled?


	public bool IsLimitEnabled()
	{
		return m_enableLimit;
	}

	/// Enable/disable the joint limit.
	public void EnableLimit(bool flag)
	{
		if (flag != m_enableLimit)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_enableLimit = flag;
			m_impulse.z = 0.0f;
		}
	}

	/// Get the lower joint limit, usually in meters.


	public float GetLowerLimit()
	{
		return m_lowerTranslation;
	}

	/// Get the upper joint limit, usually in meters.


	public float GetUpperLimit()
	{
		return m_upperTranslation;
	}

	/// Set the joint limits, usually in meters.
	public void SetLimits(float lower, float upper)
	{
		Debug.Assert(lower <= upper);
		if (lower != m_lowerTranslation || upper != m_upperTranslation)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_lowerTranslation = lower;
			m_upperTranslation = upper;
			m_impulse.z = 0.0f;
		}
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

	/// Set the motor speed, usually in meters per second.
	public void SetMotorSpeed(float speed)
	{
		if (speed != m_motorSpeed)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_motorSpeed = speed;
		}
	}

	/// Get the motor speed, usually in meters per second.


	public float GetMotorSpeed()
	{
		return m_motorSpeed;
	}

	/// Set the maximum motor force, usually in N.
	public void SetMaxMotorForce(float force)
	{
		if (force != m_maxMotorForce)
		{
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_maxMotorForce = force;
		}
	}


	public float GetMaxMotorForce()
	{
		return m_maxMotorForce;
	}

	/// Get the current motor force given the inverse time step, usually in N.


	public float GetMotorForce(float inv_dt)
	{
		return inv_dt * m_motorImpulse;
	}

	/// Dump to GlobalMembers.b2Log
	public override void Dump()
	{
		int indexA = m_bodyA.m_islandIndex;
		int indexB = m_bodyB.m_islandIndex;

		Utils.b2Log("  b2PrismaticJointDef jd;\n");
		Utils.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
		Utils.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
		Utils.b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		Utils.b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
		Utils.b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
		Utils.b2Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", m_localXAxisA.x, m_localXAxisA.y);
		Utils.b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
		Utils.b2Log("  jd.enableLimit = bool(%d);\n", m_enableLimit);
		Utils.b2Log("  jd.lowerTranslation = %.15lef;\n", m_lowerTranslation);
		Utils.b2Log("  jd.upperTranslation = %.15lef;\n", m_upperTranslation);
		Utils.b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
		Utils.b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
		Utils.b2Log("  jd.maxMotorForce = %.15lef;\n", m_maxMotorForce);
		Utils.b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
	}


    internal b2PrismaticJoint(b2PrismaticJointDef def) : base(def)
	{


		m_localAnchorA = def.localAnchorA;


		m_localAnchorB = def.localAnchorB;


		m_localXAxisA = def.localAxisA;
		m_localXAxisA.Normalize();
		m_localYAxisA = Utils.b2Cross(1.0f, m_localXAxisA);
		m_referenceAngle = def.referenceAngle;

		m_impulse.SetZero();
		m_motorMass = 0.0f;
		m_motorImpulse = 0.0f;

		m_lowerTranslation = def.lowerTranslation;
		m_upperTranslation = def.upperTranslation;
		m_maxMotorForce = def.maxMotorForce;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
		m_limitState = b2LimitState.e_inactiveLimit;

		m_axis.SetZero();
		m_perp.SetZero();
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

		// Compute the effective masses.
		b2Vec2 rA = Utils.b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = Utils.b2Mul(qB, m_localAnchorB - m_localCenterB);
		b2Vec2 d = (cB - cA) + rB - rA;

		float mA = m_invMassA;
		float mB = m_invMassB;
		float iA = m_invIA;
		float iB = m_invIB;

		{
		// Compute motor Jacobian and effective mass.


			m_axis = Utils.b2Mul(qA, m_localXAxisA);
			m_a1 = Utils.b2Cross(d + rA, m_axis);
			m_a2 = Utils.b2Cross(rB, m_axis);

			m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
			if (m_motorMass > 0.0f)
			{
				m_motorMass = 1.0f / m_motorMass;
			}
		}

		{
		// Prismatic constraint.


			m_perp = Utils.b2Mul(qA, m_localYAxisA);

			m_s1 = Utils.b2Cross(d + rA, m_perp);
			m_s2 = Utils.b2Cross(rB, m_perp);

			float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
			float k12 = iA * m_s1 + iB * m_s2;
			float k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
			float k22 = iA + iB;
			if (k22 == 0.0f)
			{
				// For bodies with fixed rotation.
				k22 = 1.0f;
			}
			float k23 = iA * m_a1 + iB * m_a2;
			float k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

			m_K.ex.Set(k11, k12, k13);
			m_K.ey.Set(k12, k22, k23);
			m_K.ez.Set(k13, k23, k33);
		}

		// Compute motor and limit terms.
		if (m_enableLimit)
		{
			float jointTranslation = Utils.b2Dot(m_axis, d);
			if (Utils.b2Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.b2_linearSlop)
			{
				m_limitState = b2LimitState.e_equalLimits;
			}
			else if (jointTranslation <= m_lowerTranslation)
			{
				if (m_limitState != b2LimitState.e_atLowerLimit)
				{
					m_limitState = b2LimitState.e_atLowerLimit;
					m_impulse.z = 0.0f;
				}
			}
			else if (jointTranslation >= m_upperTranslation)
			{
				if (m_limitState != b2LimitState.e_atUpperLimit)
				{
					m_limitState = b2LimitState.e_atUpperLimit;
					m_impulse.z = 0.0f;
				}
			}
			else
			{
				m_limitState = b2LimitState.e_inactiveLimit;
				m_impulse.z = 0.0f;
			}
		}
		else
		{
			m_limitState = b2LimitState.e_inactiveLimit;
			m_impulse.z = 0.0f;
		}

		if (m_enableMotor == false)
		{
			m_motorImpulse = 0.0f;
		}

		if (data.step.warmStarting)
		{
			// Account for variable time step.
			m_impulse *= data.step.dtRatio;
			m_motorImpulse *= data.step.dtRatio;

			b2Vec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
			float LA = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
			float LB = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;

			vA -= mA * P;
			wA -= iA * LA;

			vB += mB * P;
			wB += iB * LB;
		}
		else
		{
			m_impulse.SetZero();
			m_motorImpulse = 0.0f;
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

		// Solve linear motor constraint.
		if (m_enableMotor && m_limitState != b2LimitState.e_equalLimits)
		{
			float Cdot = Utils.b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
			float impulse = m_motorMass * (m_motorSpeed - Cdot);
			float oldImpulse = m_motorImpulse;
			float maxImpulse = data.step.dt * m_maxMotorForce;
			m_motorImpulse = Utils.b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_motorImpulse - oldImpulse;

			b2Vec2 P = impulse * m_axis;
			float LA = impulse * m_a1;
			float LB = impulse * m_a2;

			vA -= mA * P;
			wA -= iA * LA;

			vB += mB * P;
			wB += iB * LB;
		}

		b2Vec2 Cdot1 = new b2Vec2();
		Cdot1.x = Utils.b2Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
		Cdot1.y = wB - wA;

		if (m_enableLimit && m_limitState != b2LimitState.e_inactiveLimit)
		{
			// Solve prismatic and limit constraint in block form.
			float Cdot2;
			Cdot2 = Utils.b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
			b2Vec3 Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);



			b2Vec3 f1 = m_impulse;
			b2Vec3 df = m_K.Solve33(-Cdot);
			m_impulse += df;

			if (m_limitState == b2LimitState.e_atLowerLimit)
			{
				m_impulse.z = Utils.b2Max(m_impulse.z, 0.0f);
			}
			else if (m_limitState == b2LimitState.e_atUpperLimit)
			{
				m_impulse.z = Utils.b2Min(m_impulse.z, 0.0f);
			}

			// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
			b2Vec2 b = -Cdot1 - (m_impulse.z - f1.z) * new b2Vec2(m_K.ez.x, m_K.ez.y);
			b2Vec2 f2r = m_K.Solve22(b) + new b2Vec2(f1.x, f1.y);
			m_impulse.x = f2r.x;
			m_impulse.y = f2r.y;



			df = m_impulse - f1;

			b2Vec2 P = df.x * m_perp + df.z * m_axis;
			float LA = df.x * m_s1 + df.y + df.z * m_a1;
			float LB = df.x * m_s2 + df.y + df.z * m_a2;

			vA -= mA * P;
			wA -= iA * LA;

			vB += mB * P;
			wB += iB * LB;
		}
		else
		{
			// Limit is inactive, just solve the prismatic constraint in block form.
			b2Vec2 df = m_K.Solve22(-Cdot1);
			m_impulse.x += df.x;
			m_impulse.y += df.y;

			b2Vec2 P = df.x * m_perp;
			float LA = df.x * m_s1 + df.y;
			float LB = df.x * m_s2 + df.y;

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

    // A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
    // the position solver is not there to resolve forces.It is only there to cope with integration error.
    //
    // Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
    //
    // We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
    // solver indicates the limit is inactive.
    internal override bool SolvePositionConstraints(b2SolverData data)
	{
		b2Vec2 cA = data.positions[m_indexA].c;
		float aA = data.positions[m_indexA].a;
		b2Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;

		b2Rot qA = new b2Rot(aA);
		b2Rot qB = new b2Rot(aB);

		float mA = m_invMassA;
		float mB = m_invMassB;
		float iA = m_invIA;
		float iB = m_invIB;

		// Compute fresh Jacobians
		b2Vec2 rA = Utils.b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = Utils.b2Mul(qB, m_localAnchorB - m_localCenterB);
		b2Vec2 d = cB + rB - cA - rA;

		b2Vec2 axis = Utils.b2Mul(qA, m_localXAxisA);
		float a1 = Utils.b2Cross(d + rA, axis);
		float a2 = Utils.b2Cross(rB, axis);
		b2Vec2 perp = Utils.b2Mul(qA, m_localYAxisA);

		float s1 = Utils.b2Cross(d + rA, perp);
		float s2 = Utils.b2Cross(rB, perp);

		b2Vec3 impulse = new b2Vec3();
		b2Vec2 C1 = new b2Vec2();
		C1.x = Utils.b2Dot(perp, d);
		C1.y = aB - aA - m_referenceAngle;

		float linearError = Utils.b2Abs(C1.x);
		float angularError = Utils.b2Abs(C1.y);

		bool active = false;
		float C2 = 0.0f;
		if (m_enableLimit)
		{
			float translation = Utils.b2Dot(axis, d);
			if (Utils.b2Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.b2_linearSlop)
			{
				// Prevent large angular corrections
				C2 = Utils.b2Clamp(translation, -Settings.b2_maxLinearCorrection, Settings.b2_maxLinearCorrection);
				linearError = Utils.b2Max(linearError, Utils.b2Abs(translation));
				active = true;
			}
			else if (translation <= m_lowerTranslation)
			{
				// Prevent large linear corrections and allow some slop.
				C2 = Utils.b2Clamp(translation - m_lowerTranslation + Settings.b2_linearSlop, -Settings.b2_maxLinearCorrection, 0.0f);
				linearError = Utils.b2Max(linearError, m_lowerTranslation - translation);
				active = true;
			}
			else if (translation >= m_upperTranslation)
			{
				// Prevent large linear corrections and allow some slop.
				C2 = Utils.b2Clamp(translation - m_upperTranslation - Settings.b2_linearSlop, 0.0f, Settings.b2_maxLinearCorrection);
				linearError = Utils.b2Max(linearError, translation - m_upperTranslation);
				active = true;
			}
		}

		if (active)
		{
			float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
			float k12 = iA * s1 + iB * s2;
			float k13 = iA * s1 * a1 + iB * s2 * a2;
			float k22 = iA + iB;
			if (k22 == 0.0f)
			{
				// For fixed rotation
				k22 = 1.0f;
			}
			float k23 = iA * a1 + iB * a2;
			float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

			b2Mat33 K = new b2Mat33();
			K.ex.Set(k11, k12, k13);
			K.ey.Set(k12, k22, k23);
			K.ez.Set(k13, k23, k33);

			b2Vec3 C = new b2Vec3();
			C.x = C1.x;
			C.y = C1.y;
			C.z = C2;



			impulse = K.Solve33(-C);
		}
		else
		{
			float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
			float k12 = iA * s1 + iB * s2;
			float k22 = iA + iB;
			if (k22 == 0.0f)
			{
				k22 = 1.0f;
			}

			b2Mat22 K = new b2Mat22();
			K.ex.Set(k11, k12);
			K.ey.Set(k12, k22);

			b2Vec2 impulse1 = K.Solve(-C1);
			impulse.x = impulse1.x;
			impulse.y = impulse1.y;
			impulse.z = 0.0f;
		}

		b2Vec2 P = impulse.x * perp + impulse.z * axis;
		float LA = impulse.x * s1 + impulse.y + impulse.z * a1;
		float LB = impulse.x * s2 + impulse.y + impulse.z * a2;

		cA -= mA * P;
		aA -= iA * LA;
		cB += mB * P;
		aB += iB * LB;



		data.positions[m_indexA].c = cA;
		data.positions[m_indexA].a = aA;


		data.positions[m_indexB].c = cB;
		data.positions[m_indexB].a = aB;

		return linearError <= Settings.b2_linearSlop && angularError <= Settings.b2_angularSlop;
	}

    // Solver shared
    internal b2Vec2 m_localAnchorA = new b2Vec2();
    internal b2Vec2 m_localAnchorB = new b2Vec2();
    internal b2Vec2 m_localXAxisA = new b2Vec2();
    internal b2Vec2 m_localYAxisA = new b2Vec2();
    internal float m_referenceAngle;
	protected b2Vec3 m_impulse = new b2Vec3();
	protected float m_motorImpulse;
	protected float m_lowerTranslation;
	protected float m_upperTranslation;
	protected float m_maxMotorForce;
	protected float m_motorSpeed;
	protected bool m_enableLimit;
	protected bool m_enableMotor;
	protected b2LimitState m_limitState;

	// Solver temp
	protected int m_indexA;
	protected int m_indexB;
	protected b2Vec2 m_localCenterA = new b2Vec2();
	protected b2Vec2 m_localCenterB = new b2Vec2();
	protected float m_invMassA;
	protected float m_invMassB;
	protected float m_invIA;
	protected float m_invIB;
	protected b2Vec2 m_axis = new b2Vec2();
	protected b2Vec2 m_perp = new b2Vec2();
	protected float m_s1;
	protected float m_s2;
	protected float m_a1;
	protected float m_a2;
	protected b2Mat33 m_K = new b2Mat33();
	protected float m_motorMass;
}
