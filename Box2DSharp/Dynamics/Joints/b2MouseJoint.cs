﻿using System;
using System.Diagnostics;

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





























/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
public class b2MouseJointDef : b2JointDef
{
	public b2MouseJointDef()
	{
		type = b2JointType.e_mouseJoint;
		target.Set(0.0f, 0.0f);
		maxForce = 0.0f;
		frequencyHz = 5.0f;
		dampingRatio = 0.7f;
	}

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	public b2Vec2 target = new b2Vec2();

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	public float maxForce;

	/// The response speed.
	public float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	public float dampingRatio;
}

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
public class b2MouseJoint : b2Joint
{

	/// Implements b2Joint.


	public override b2Vec2 GetAnchorA()
	{
		return m_targetA;
	}

	/// Implements b2Joint.


	public override b2Vec2 GetAnchorB()
	{
		return m_bodyB.GetWorldPoint(m_localAnchorB);
	}

	/// Implements b2Joint.


	public override b2Vec2 GetReactionForce(float inv_dt)
	{
		return inv_dt * m_impulse;
	}

	/// Implements b2Joint.


	public override float GetReactionTorque(float inv_dt)
	{
		return inv_dt * 0.0f;
	}

	/// Use this to update the target point.
	public void SetTarget(b2Vec2 target)
	{
		if (target != m_targetA)
		{
			m_bodyB.SetAwake(true);


			m_targetA = target;
		}
	}


	public b2Vec2 GetTarget()
	{
		return m_targetA;
	}

	/// Set/get the maximum force in Newtons.
	public void SetMaxForce(float force)
	{
		m_maxForce = force;
	}


	public float GetMaxForce()
	{
		return m_maxForce;
	}

	/// Set/get the frequency in Hertz.
	public void SetFrequency(float hz)
	{
		m_frequencyHz = hz;
	}


	public float GetFrequency()
	{
		return m_frequencyHz;
	}

	/// Set/get the damping ratio (dimensionless).
	public void SetDampingRatio(float ratio)
	{
		m_dampingRatio = ratio;
	}


	public float GetDampingRatio()
	{
		return m_dampingRatio;
	}

	/// The mouse joint does not support dumping.
	public override void Dump()
	{
		GlobalMembers.b2Log("Mouse joint dumping is not supported.\n");
	}

	/// Implement b2Joint::ShiftOrigin
	public override void ShiftOrigin(b2Vec2 newOrigin)
	{
		m_targetA -= newOrigin;
	}


    //	friend class b2Joint;


    // p = attached point, m = mouse point
    // C = p - m
    // Cdot = v
    //      = v + cross(w, r)
    // J = [I r_skew]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    internal b2MouseJoint(b2MouseJointDef def) : base(def)
	{
		Debug.Assert(def.target.IsValid());
		Debug.Assert(GlobalMembers.b2IsValid(def.maxForce) && def.maxForce >= 0.0f);
		Debug.Assert(GlobalMembers.b2IsValid(def.frequencyHz) && def.frequencyHz >= 0.0f);
		Debug.Assert(GlobalMembers.b2IsValid(def.dampingRatio) && def.dampingRatio >= 0.0f);



		m_targetA = def.target;


		m_localAnchorB = GlobalMembers.b2MulT(m_bodyB.GetTransform(), m_targetA);

		m_maxForce = def.maxForce;
		m_impulse.SetZero();

		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;

		m_beta = 0.0f;
		m_gamma = 0.0f;
	}

    internal override void InitVelocityConstraints(b2SolverData data)
	{
		m_indexB = m_bodyB.m_islandIndex;


		m_localCenterB = m_bodyB.m_sweep.localCenter;
		m_invMassB = m_bodyB.m_invMass;
		m_invIB = m_bodyB.m_invI;

		b2Vec2 cB = data.positions[m_indexB].c;
		float aB = data.positions[m_indexB].a;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		b2Rot qB = new b2Rot(aB);

		float mass = m_bodyB.GetMass();

		// Frequency
		float omega = 2.0f * DefineConstants.b2_pi * m_frequencyHz;

		// Damping coefficient
		float d = 2.0f * mass * m_dampingRatio * omega;

		// Spring stiffness
		float k = mass * (omega * omega);

		// magic formulas
		// gamma has units of inverse mass.
		// beta has units of inverse time.
		float h = data.step.dt;
		Debug.Assert(d + h * k > float.Epsilon);
		m_gamma = h * (d + h * k);
		if (m_gamma != 0.0f)
		{
			m_gamma = 1.0f / m_gamma;
		}
		m_beta = h * k * m_gamma;

		// Compute the effective mass matrix.


		m_rB = GlobalMembers.b2Mul(qB, m_localAnchorB - m_localCenterB);

		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		b2Mat22 K = new b2Mat22();
		K.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
		K.ex.y = -m_invIB * m_rB.x * m_rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;



		m_mass = K.GetInverse();



		m_C = cB + m_rB - m_targetA;
		m_C *= m_beta;

		// Cheat with some damping
		wB *= 0.98f;

		if (data.step.warmStarting)
		{
			m_impulse *= data.step.dtRatio;
			vB += m_invMassB * m_impulse;
			wB += m_invIB * GlobalMembers.b2Cross(m_rB, m_impulse);
		}
		else
		{
			m_impulse.SetZero();
		}



		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
    internal override void SolveVelocityConstraints(b2SolverData data)
	{
		b2Vec2 vB = data.velocities[m_indexB].v;
		float wB = data.velocities[m_indexB].w;

		// Cdot = v + cross(w, r)
		b2Vec2 Cdot = vB + GlobalMembers.b2Cross(wB, m_rB);
		b2Vec2 impulse = GlobalMembers.b2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));



		b2Vec2 oldImpulse = new b2Vec2(m_impulse);
		m_impulse += impulse;
		float maxImpulse = data.step.dt * m_maxForce;
		if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
		{
			m_impulse *= maxImpulse / m_impulse.Length();
		}
		impulse = m_impulse - oldImpulse;

		vB += m_invMassB * impulse;
		wB += m_invIB * GlobalMembers.b2Cross(m_rB, impulse);



		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
	}
    internal override bool SolvePositionConstraints(b2SolverData data)
	{
		
		return true;
	}

	protected b2Vec2 m_localAnchorB = new b2Vec2();
	protected b2Vec2 m_targetA = new b2Vec2();
	protected float m_frequencyHz;
	protected float m_dampingRatio;
	protected float m_beta;

	// Solver shared
	protected b2Vec2 m_impulse = new b2Vec2();
	protected float m_maxForce;
	protected float m_gamma;

	// Solver temp
	protected int m_indexA;
	protected int m_indexB;
	protected b2Vec2 m_rB = new b2Vec2();
	protected b2Vec2 m_localCenterB = new b2Vec2();
	protected float m_invMassB;
	protected float m_invIB;
	protected b2Mat22 m_mass = new b2Mat22();
	protected b2Vec2 m_C = new b2Vec2();
}
