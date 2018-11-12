﻿//#define B2_DEBUG_SOLVER

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

public class b2VelocityConstraintPoint
{
	public b2Vec2 rA = new b2Vec2();
	public b2Vec2 rB = new b2Vec2();
	public float normalImpulse;
	public float tangentImpulse;
	public float normalMass;
	public float tangentMass;
	public float velocityBias;
}

public class b2ContactVelocityConstraint
{
	public b2VelocityConstraintPoint[] points = Arrays.InitializeWithDefaultInstances<b2VelocityConstraintPoint>(DefineConstants.b2_maxManifoldPoints);
	public b2Vec2 normal = new b2Vec2();
	public b2Mat22 normalMass = new b2Mat22();
	public b2Mat22 K = new b2Mat22();
	public int indexA;
	public int indexB;
	public float invMassA;
	public float invMassB;
	public float invIA;
	public float invIB;
	public float friction;
	public float restitution;
	public float tangentSpeed;
	public int pointCount;
	public int contactIndex;
}

public class b2ContactSolverDef
{
	public b2TimeStep step = new b2TimeStep();
	public b2Contact[] contacts;
	public int count;
	public b2Position[] positions;
	public b2Velocity[] velocities;
}

public class b2ContactSolver : System.IDisposable
{
	public b2ContactSolver(b2ContactSolverDef def)
	{
		m_step = def.step;
		m_count = def.count;
		m_positionConstraints = Arrays.InitializeWithDefaultInstances<b2ContactPositionConstraint>(m_count);
		m_velocityConstraints = Arrays.InitializeWithDefaultInstances<b2ContactVelocityConstraint>(m_count);
		m_positions = def.positions;
		m_velocities = def.velocities;
		m_contacts = def.contacts;

		// Initialize position independent portions of the constraints.
		for (int i = 0; i < m_count; ++i)
		{
			b2Contact contact = m_contacts[i];

			b2Fixture fixtureA = contact.m_fixtureA;
			b2Fixture fixtureB = contact.m_fixtureB;
			b2Shape shapeA = fixtureA.GetShape();
			b2Shape shapeB = fixtureB.GetShape();
			float radiusA = shapeA.m_radius;
			float radiusB = shapeB.m_radius;
			b2Body bodyA = fixtureA.GetBody();
			b2Body bodyB = fixtureB.GetBody();
			b2Manifold manifold = contact.GetManifold();

			int pointCount = manifold.pointCount;
			Debug.Assert(pointCount > 0);

			b2ContactVelocityConstraint vc = m_velocityConstraints[i];
			vc.friction = contact.m_friction;
			vc.restitution = contact.m_restitution;
			vc.tangentSpeed = contact.m_tangentSpeed;
			vc.indexA = bodyA.m_islandIndex;
			vc.indexB = bodyB.m_islandIndex;
			vc.invMassA = bodyA.m_invMass;
			vc.invMassB = bodyB.m_invMass;
			vc.invIA = bodyA.m_invI;
			vc.invIB = bodyB.m_invI;
			vc.contactIndex = i;
			vc.pointCount = pointCount;
			vc.K.SetZero();
			vc.normalMass.SetZero();

			b2ContactPositionConstraint pc = m_positionConstraints[i];
			pc.indexA = bodyA.m_islandIndex;
			pc.indexB = bodyB.m_islandIndex;
			pc.invMassA = bodyA.m_invMass;
			pc.invMassB = bodyB.m_invMass;
			pc.localCenterA = bodyA.m_sweep.localCenter;
			pc.localCenterB = bodyB.m_sweep.localCenter;
			pc.invIA = bodyA.m_invI;
			pc.invIB = bodyB.m_invI;
			pc.localNormal = manifold.localNormal;
			pc.localPoint = manifold.localPoint;
			pc.pointCount = pointCount;
			pc.radiusA = radiusA;
			pc.radiusB = radiusB;
			pc.type = manifold.type;

			for (int j = 0; j < pointCount; ++j)
			{
				b2ManifoldPoint cp = manifold.points[j];
				b2VelocityConstraintPoint vcp = vc.points[j];

				if (m_step.warmStarting)
				{
					vcp.normalImpulse = m_step.dtRatio * cp.normalImpulse;
					vcp.tangentImpulse = m_step.dtRatio * cp.tangentImpulse;
				}
				else
				{
					vcp.normalImpulse = 0.0f;
					vcp.tangentImpulse = 0.0f;
				}

				vcp.rA.SetZero();
				vcp.rB.SetZero();
				vcp.normalMass = 0.0f;
				vcp.tangentMass = 0.0f;
				vcp.velocityBias = 0.0f;

				pc.localPoints[j] = cp.localPoint;
			}
		}
	}
	public void Dispose()
	{
		m_velocityConstraints = null;
		m_positionConstraints = null;
	}


	// Initialize position dependent portions of the velocity constraints.
	public void InitializeVelocityConstraints()
	{
		for (int i = 0; i < m_count; ++i)
		{
			b2ContactVelocityConstraint vc = m_velocityConstraints[i];
			b2ContactPositionConstraint pc = m_positionConstraints[i];

			float radiusA = pc.radiusA;
			float radiusB = pc.radiusB;
			b2Manifold manifold = m_contacts[vc.contactIndex].GetManifold();

			int indexA = vc.indexA;
			int indexB = vc.indexB;

			float mA = vc.invMassA;
			float mB = vc.invMassB;
			float iA = vc.invIA;
			float iB = vc.invIB;
			b2Vec2 localCenterA = new b2Vec2(pc.localCenterA);
			b2Vec2 localCenterB = new b2Vec2(pc.localCenterB);

			b2Vec2 cA = m_positions[indexA].c;
			float aA = m_positions[indexA].a;
			b2Vec2 vA = m_velocities[indexA].v;
			float wA = m_velocities[indexA].w;

			b2Vec2 cB = m_positions[indexB].c;
			float aB = m_positions[indexB].a;
			b2Vec2 vB = m_velocities[indexB].v;
			float wB = m_velocities[indexB].w;

			Debug.Assert(manifold.pointCount > 0);

			b2Transform xfA = new b2Transform();
			b2Transform xfB = new b2Transform();
			xfA.q.Set(aA);
			xfB.q.Set(aB);
			xfA.p = cA - GlobalMembers.b2Mul(xfA.q, localCenterA);
			xfB.p = cB - GlobalMembers.b2Mul(xfB.q, localCenterB);

			b2WorldManifold worldManifold = new b2WorldManifold();
			worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

			vc.normal = worldManifold.normal;

			int pointCount = vc.pointCount;
			for (int j = 0; j < pointCount; ++j)
			{
				b2VelocityConstraintPoint vcp = vc.points[j];

				vcp.rA = worldManifold.points[j] - cA;
				vcp.rB = worldManifold.points[j] - cB;

				float rnA = GlobalMembers.b2Cross(vcp.rA, vc.normal);
				float rnB = GlobalMembers.b2Cross(vcp.rB, vc.normal);

				float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				vcp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

				b2Vec2 tangent = GlobalMembers.b2Cross(vc.normal, 1.0f);

				float rtA = GlobalMembers.b2Cross(vcp.rA, tangent);
				float rtB = GlobalMembers.b2Cross(vcp.rB, tangent);

				float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

				vcp.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

				// Setup a velocity bias for restitution.
				vcp.velocityBias = 0.0f;
				float vRel = GlobalMembers.b2Dot(vc.normal, vB + GlobalMembers.b2Cross(wB, vcp.rB) - vA - GlobalMembers.b2Cross(wA, vcp.rA));
				if (vRel < -DefineConstants.b2_velocityThreshold)
				{
					vcp.velocityBias = -vc.restitution * vRel;
				}
			}

			// If we have two points, then prepare the block solver.
			if (vc.pointCount == 2 && GlobalMembers.g_blockSolve)
			{
				b2VelocityConstraintPoint vcp1 = vc.points[0];
				b2VelocityConstraintPoint vcp2 = vc.points[1];

				float rn1A = GlobalMembers.b2Cross(vcp1.rA, vc.normal);
				float rn1B = GlobalMembers.b2Cross(vcp1.rB, vc.normal);
				float rn2A = GlobalMembers.b2Cross(vcp2.rA, vc.normal);
				float rn2B = GlobalMembers.b2Cross(vcp2.rB, vc.normal);

				float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
				float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
				float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				const float k_maxConditionNumber = 1000.0f;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
				{
					// K is safe to invert.
					vc.K.ex.Set(k11, k12);
					vc.K.ey.Set(k12, k22);
					vc.normalMass = vc.K.GetInverse();
				}
				else
				{
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					vc.pointCount = 1;
				}
			}
		}
	}

	public void WarmStart()
	{
		// Warm start.
		for (int i = 0; i < m_count; ++i)
		{
			b2ContactVelocityConstraint vc = m_velocityConstraints[i];

			int indexA = vc.indexA;
			int indexB = vc.indexB;
			float mA = vc.invMassA;
			float iA = vc.invIA;
			float mB = vc.invMassB;
			float iB = vc.invIB;
			int pointCount = vc.pointCount;

			b2Vec2 vA = m_velocities[indexA].v;
			float wA = m_velocities[indexA].w;
			b2Vec2 vB = m_velocities[indexB].v;
			float wB = m_velocities[indexB].w;



			b2Vec2 normal = new b2Vec2(vc.normal);
			b2Vec2 tangent = GlobalMembers.b2Cross(normal, 1.0f);

			for (int j = 0; j < pointCount; ++j)
			{
				b2VelocityConstraintPoint vcp = vc.points[j];
				b2Vec2 P = vcp.normalImpulse * normal + vcp.tangentImpulse * tangent;
				wA -= iA * GlobalMembers.b2Cross(vcp.rA, P);
				vA -= mA * P;
				wB += iB * GlobalMembers.b2Cross(vcp.rB, P);
				vB += mB * P;
			}



			m_velocities[indexA].v = vA;
			m_velocities[indexA].w = wA;


			m_velocities[indexB].v = vB;
			m_velocities[indexB].w = wB;
		}
	}
	public void SolveVelocityConstraints()
	{
		for (int i = 0; i < m_count; ++i)
		{
			b2ContactVelocityConstraint vc = m_velocityConstraints[i];

			int indexA = vc.indexA;
			int indexB = vc.indexB;
			float mA = vc.invMassA;
			float iA = vc.invIA;
			float mB = vc.invMassB;
			float iB = vc.invIB;
			int pointCount = vc.pointCount;

			b2Vec2 vA = m_velocities[indexA].v;
			float wA = m_velocities[indexA].w;
			b2Vec2 vB = m_velocities[indexB].v;
			float wB = m_velocities[indexB].w;



			b2Vec2 normal = new b2Vec2(vc.normal);
			b2Vec2 tangent = GlobalMembers.b2Cross(normal, 1.0f);
			float friction = vc.friction;

			Debug.Assert(pointCount == 1 || pointCount == 2);

			// Solve tangent constraints first because non-penetration is more important
			// than friction.
			for (int j = 0; j < pointCount; ++j)
			{
				b2VelocityConstraintPoint vcp = vc.points[j];

				// Relative velocity at contact
				b2Vec2 dv = vB + GlobalMembers.b2Cross(wB, vcp.rB) - vA - GlobalMembers.b2Cross(wA, vcp.rA);

				// Compute tangent force
				float vt = GlobalMembers.b2Dot(dv, tangent) - vc.tangentSpeed;
				float lambda = vcp.tangentMass * (-vt);

				// b2Clamp the accumulated force
				float maxFriction = friction * vcp.normalImpulse;
				float newImpulse = GlobalMembers.b2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - vcp.tangentImpulse;
				vcp.tangentImpulse = newImpulse;

				// Apply contact impulse
				b2Vec2 P = lambda * tangent;

				vA -= mA * P;
				wA -= iA * GlobalMembers.b2Cross(vcp.rA, P);

				vB += mB * P;
				wB += iB * GlobalMembers.b2Cross(vcp.rB, P);
			}

			// Solve normal constraints
			if (pointCount == 1 || GlobalMembers.g_blockSolve == false)
			{
				for (int j = 0; j < pointCount; ++j)
				{
					b2VelocityConstraintPoint vcp = vc.points[j];

					// Relative velocity at contact
					b2Vec2 dv = vB + GlobalMembers.b2Cross(wB, vcp.rB) - vA - GlobalMembers.b2Cross(wA, vcp.rA);

					// Compute normal impulse
					float vn = GlobalMembers.b2Dot(dv, normal);
					float lambda = -vcp.normalMass * (vn - vcp.velocityBias);

					// b2Clamp the accumulated impulse
					float newImpulse = GlobalMembers.b2Max(vcp.normalImpulse + lambda, 0.0f);
					lambda = newImpulse - vcp.normalImpulse;
					vcp.normalImpulse = newImpulse;

					// Apply contact impulse
					b2Vec2 P = lambda * normal;
					vA -= mA * P;
					wA -= iA * GlobalMembers.b2Cross(vcp.rA, P);

					vB += mB * P;
					wB += iB * GlobalMembers.b2Cross(vcp.rB, P);
				}
			}
			else
			{
				// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
				// Build the mini LCP for this contact patch
				//
				// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
				//
				// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
				// b = vn0 - velocityBias
				//
				// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
				// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
				// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
				// solution that satisfies the problem is chosen.
				//
				// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
				// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
				//
				// Substitute:
				//
				// x = a + d
				//
				// a := old total impulse
				// x := new total impulse
				// d := incremental impulse
				//
				// For the current iteration we extend the formula for the incremental impulse
				// to compute the new total impulse:
				//
				// vn = A * d + b
				//    = A * (x - a) + b
				//    = A * x + b - A * a
				//    = A * x + b'
				// b' = b - A * a;

				b2VelocityConstraintPoint cp1 = vc.points[0];
				b2VelocityConstraintPoint cp2 = vc.points[1];

				b2Vec2 a = new b2Vec2(cp1.normalImpulse, cp2.normalImpulse);
				Debug.Assert(a.x >= 0.0f && a.y >= 0.0f);

				// Relative velocity at contact
				b2Vec2 dv1 = vB + GlobalMembers.b2Cross(wB, cp1.rB) - vA - GlobalMembers.b2Cross(wA, cp1.rA);
				b2Vec2 dv2 = vB + GlobalMembers.b2Cross(wB, cp2.rB) - vA - GlobalMembers.b2Cross(wA, cp2.rA);

				// Compute normal velocity
				float vn1 = GlobalMembers.b2Dot(dv1, normal);
				float vn2 = GlobalMembers.b2Dot(dv2, normal);

				b2Vec2 b = new b2Vec2();
				b.x = vn1 - cp1.velocityBias;
				b.y = vn2 - cp2.velocityBias;

				// Compute b'
				b -= GlobalMembers.b2Mul(vc.K, a);

				//const float k_errorTol = 1e-3f;

				for (;;)
				{
					//
					// Case 1: vn = 0
					//
					// 0 = A * x + b'
					//
					// Solve for x:
					//
					// x = - inv(A) * b'
					//
					b2Vec2 x = - GlobalMembers.b2Mul(vc.normalMass, b);

					if (x.x >= 0.0f && x.y >= 0.0f)
					{
						// Get the incremental impulse
						b2Vec2 d = x - a;

						// Apply incremental impulse
						b2Vec2 P1 = d.x * normal;
						b2Vec2 P2 = d.y * normal;
						vA -= mA * (P1 + P2);
						wA -= iA * (GlobalMembers.b2Cross(cp1.rA, P1) + GlobalMembers.b2Cross(cp2.rA, P2));

						vB += mB * (P1 + P2);
						wB += iB * (GlobalMembers.b2Cross(cp1.rB, P1) + GlobalMembers.b2Cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

	
#if B2_DEBUG_SOLVER
						// Postconditions
						dv1 = vB + GlobalMembers.b2Cross(wB, cp1.rB) - vA - GlobalMembers.b2Cross(wA, cp1.rA);
						dv2 = vB + GlobalMembers.b2Cross(wB, cp2.rB) - vA - GlobalMembers.b2Cross(wA, cp2.rA);

						// Compute normal velocity
						vn1 = GlobalMembers.b2Dot(dv1, normal);
						vn2 = GlobalMembers.b2Dot(dv2, normal);

						Debug.Assert(GlobalMembers.b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
						Debug.Assert(GlobalMembers.b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
						break;
					}

					//
					// Case 2: vn1 = 0 and x2 = 0
					//
					//   0 = a11 * x1 + a12 * 0 + b1'
					// vn2 = a21 * x1 + a22 * 0 + b2'
					//
					x.x = - cp1.normalMass * b.x;
					x.y = 0.0f;
					vn1 = 0.0f;
					vn2 = vc.K.ex.y * x.x + b.y;
					if (x.x >= 0.0f && vn2 >= 0.0f)
					{
						// Get the incremental impulse
						b2Vec2 d = x - a;

						// Apply incremental impulse
						b2Vec2 P1 = d.x * normal;
						b2Vec2 P2 = d.y * normal;
						vA -= mA * (P1 + P2);
						wA -= iA * (GlobalMembers.b2Cross(cp1.rA, P1) + GlobalMembers.b2Cross(cp2.rA, P2));

						vB += mB * (P1 + P2);
						wB += iB * (GlobalMembers.b2Cross(cp1.rB, P1) + GlobalMembers.b2Cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER
						// Postconditions
						dv1 = vB + GlobalMembers.b2Cross(wB, cp1.rB) - vA - GlobalMembers.b2Cross(wA, cp1.rA);

						// Compute normal velocity
						vn1 = GlobalMembers.b2Dot(dv1, normal);

						Debug.Assert(GlobalMembers.b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
#endif
						break;
					}


					//
					// Case 3: vn2 = 0 and x1 = 0
					//
					// vn1 = a11 * 0 + a12 * x2 + b1'
					//   0 = a21 * 0 + a22 * x2 + b2'
					//
					x.x = 0.0f;
					x.y = - cp2.normalMass * b.y;
					vn1 = vc.K.ey.x * x.y + b.x;
					vn2 = 0.0f;

					if (x.y >= 0.0f && vn1 >= 0.0f)
					{
						// Resubstitute for the incremental impulse
						b2Vec2 d = x - a;

						// Apply incremental impulse
						b2Vec2 P1 = d.x * normal;
						b2Vec2 P2 = d.y * normal;
						vA -= mA * (P1 + P2);
						wA -= iA * (GlobalMembers.b2Cross(cp1.rA, P1) + GlobalMembers.b2Cross(cp2.rA, P2));

						vB += mB * (P1 + P2);
						wB += iB * (GlobalMembers.b2Cross(cp1.rB, P1) + GlobalMembers.b2Cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER
						// Postconditions
						dv2 = vB + GlobalMembers.b2Cross(wB, cp2.rB) - vA - GlobalMembers.b2Cross(wA, cp2.rA);

						// Compute normal velocity
						vn2 = GlobalMembers.b2Dot(dv2, normal);

						Debug.Assert(GlobalMembers.b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
						break;
					}

					//
					// Case 4: x1 = 0 and x2 = 0
					//
					// vn1 = b1
					// vn2 = b2;
					x.x = 0.0f;
					x.y = 0.0f;
					vn1 = b.x;
					vn2 = b.y;

					if (vn1 >= 0.0f && vn2 >= 0.0f)
					{
						// Resubstitute for the incremental impulse
						b2Vec2 d = x - a;

						// Apply incremental impulse
						b2Vec2 P1 = d.x * normal;
						b2Vec2 P2 = d.y * normal;
						vA -= mA * (P1 + P2);
						wA -= iA * (GlobalMembers.b2Cross(cp1.rA, P1) + GlobalMembers.b2Cross(cp2.rA, P2));

						vB += mB * (P1 + P2);
						wB += iB * (GlobalMembers.b2Cross(cp1.rB, P1) + GlobalMembers.b2Cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						break;
					}

					// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
					break;
				}
			}

			m_velocities[indexA].v = vA;
			m_velocities[indexA].w = wA;
			m_velocities[indexB].v = vB;
			m_velocities[indexB].w = wB;
		}
	}
	public void StoreImpulses()
	{
		for (int i = 0; i < m_count; ++i)
		{
			b2ContactVelocityConstraint vc = m_velocityConstraints[i];
			b2Manifold manifold = m_contacts[vc.contactIndex].GetManifold();

			for (int j = 0; j < vc.pointCount; ++j)
			{
				manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
				manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
			}
		}
	}


	// Sequential solver.
	public bool SolvePositionConstraints()
	{
		float minSeparation = 0.0f;

		for (int i = 0; i < m_count; ++i)
		{
			b2ContactPositionConstraint pc = m_positionConstraints[i];

			int indexA = pc.indexA;
			int indexB = pc.indexB;
			b2Vec2 localCenterA = new b2Vec2(pc.localCenterA);
			float mA = pc.invMassA;
			float iA = pc.invIA;
			b2Vec2 localCenterB = new b2Vec2(pc.localCenterB);
			float mB = pc.invMassB;
			float iB = pc.invIB;
			int pointCount = pc.pointCount;

			b2Vec2 cA = m_positions[indexA].c;
			float aA = m_positions[indexA].a;

			b2Vec2 cB = m_positions[indexB].c;
			float aB = m_positions[indexB].a;

			// Solve normal constraints
			for (int j = 0; j < pointCount; ++j)
			{
				b2Transform xfA = new b2Transform();
				b2Transform xfB = new b2Transform();
				xfA.q.Set(aA);
				xfB.q.Set(aB);
				xfA.p = cA - GlobalMembers.b2Mul(xfA.q, localCenterA);
				xfB.p = cB - GlobalMembers.b2Mul(xfB.q, localCenterB);

				b2PositionSolverManifold psm = new b2PositionSolverManifold();
				psm.Initialize(pc, xfA, xfB, j);
				b2Vec2 normal = new b2Vec2(psm.normal);

				b2Vec2 point = new b2Vec2(psm.point);
				float separation = psm.separation;

				b2Vec2 rA = point - cA;
				b2Vec2 rB = point - cB;

				// Track max constraint error.
				minSeparation = GlobalMembers.b2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				float C = GlobalMembers.b2Clamp(DefineConstants.b2_baumgarte * (separation + DefineConstants.b2_linearSlop), -DefineConstants.b2_maxLinearCorrection, 0.0f);

				// Compute the effective mass.
				float rnA = GlobalMembers.b2Cross(rA, normal);
				float rnB = GlobalMembers.b2Cross(rB, normal);
				float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				float impulse = K > 0.0f ? - C / K : 0.0f;

				b2Vec2 P = impulse * normal;

				cA -= mA * P;
				aA -= iA * GlobalMembers.b2Cross(rA, P);

				cB += mB * P;
				aB += iB * GlobalMembers.b2Cross(rB, P);
			}

			m_positions[indexA].c = cA;
			m_positions[indexA].a = aA;

			m_positions[indexB].c = cB;
			m_positions[indexB].a = aB;
		}

		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -3.0f * DefineConstants.b2_linearSlop;
	}

	// Sequential position solver for position constraints.
	public bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB)
	{
		float minSeparation = 0.0f;

		for (int i = 0; i < m_count; ++i)
		{
			b2ContactPositionConstraint pc = m_positionConstraints[i];

			int indexA = pc.indexA;
			int indexB = pc.indexB;
			b2Vec2 localCenterA = new b2Vec2(pc.localCenterA);
			b2Vec2 localCenterB = new b2Vec2(pc.localCenterB);
			int pointCount = pc.pointCount;

			float mA = 0.0f;
			float iA = 0.0f;
			if (indexA == toiIndexA || indexA == toiIndexB)
			{
				mA = pc.invMassA;
				iA = pc.invIA;
			}

			float mB = 0.0f;
			float iB = 0.0F;
			if (indexB == toiIndexA || indexB == toiIndexB)
			{
				mB = pc.invMassB;
				iB = pc.invIB;
			}

			b2Vec2 cA = m_positions[indexA].c;
			float aA = m_positions[indexA].a;

			b2Vec2 cB = m_positions[indexB].c;
			float aB = m_positions[indexB].a;

			// Solve normal constraints
			for (int j = 0; j < pointCount; ++j)
			{
				b2Transform xfA = new b2Transform();
				b2Transform xfB = new b2Transform();
				xfA.q.Set(aA);
				xfB.q.Set(aB);
				xfA.p = cA - GlobalMembers.b2Mul(xfA.q, localCenterA);
				xfB.p = cB - GlobalMembers.b2Mul(xfB.q, localCenterB);

				b2PositionSolverManifold psm = new b2PositionSolverManifold();
				psm.Initialize(pc, xfA, xfB, j);
				b2Vec2 normal = new b2Vec2(psm.normal);

				b2Vec2 point = new b2Vec2(psm.point);
				float separation = psm.separation;

				b2Vec2 rA = point - cA;
				b2Vec2 rB = point - cB;

				// Track max constraint error.
				minSeparation = GlobalMembers.b2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				float C = GlobalMembers.b2Clamp(DefineConstants.b2_toiBaugarte * (separation + DefineConstants.b2_linearSlop), -DefineConstants.b2_maxLinearCorrection, 0.0f);

				// Compute the effective mass.
				float rnA = GlobalMembers.b2Cross(rA, normal);
				float rnB = GlobalMembers.b2Cross(rB, normal);
				float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				float impulse = K > 0.0f ? - C / K : 0.0f;

				b2Vec2 P = impulse * normal;

				cA -= mA * P;
				aA -= iA * GlobalMembers.b2Cross(rA, P);

				cB += mB * P;
				aB += iB * GlobalMembers.b2Cross(rB, P);
			}

			m_positions[indexA].c = cA;
			m_positions[indexA].a = aA;

			m_positions[indexB].c = cB;
			m_positions[indexB].a = aB;
		}

		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -1.5f * DefineConstants.b2_linearSlop;
	}

	public b2TimeStep m_step = new b2TimeStep();
	public b2Position[] m_positions;
	public b2Velocity[] m_velocities;
	public b2ContactPositionConstraint[] m_positionConstraints;
	public b2ContactVelocityConstraint[] m_velocityConstraints;
	public b2Contact[] m_contacts;
	public int m_count;
}

public class b2ContactPositionConstraint
{
	public b2Vec2[] localPoints = Arrays.InitializeWithDefaultInstances<b2Vec2>(DefineConstants.b2_maxManifoldPoints);
	public b2Vec2 localNormal = new b2Vec2();
	public b2Vec2 localPoint = new b2Vec2();
	public int indexA;
	public int indexB;
	public float invMassA;
	public float invMassB;
	public b2Vec2 localCenterA = new b2Vec2();
	public b2Vec2 localCenterB = new b2Vec2();
	public float invIA;
	public float invIB;
	public b2Manifold.Type type;
	public float radiusA;
	public float radiusB;
	public int pointCount;
}

public class b2PositionSolverManifold
{
	public void Initialize(b2ContactPositionConstraint pc, b2Transform xfA, b2Transform xfB, int index)
	{
		Debug.Assert(pc.pointCount > 0);

		switch (pc.type)
		{
		case b2Manifold.Type.e_circles:
		{
				b2Vec2 pointA = GlobalMembers.b2Mul(xfA, pc.localPoint);
				b2Vec2 pointB = GlobalMembers.b2Mul(xfB, pc.localPoints[0]);
				normal = pointB - pointA;
				normal.Normalize();
				point = 0.5f * (pointA + pointB);
				separation = GlobalMembers.b2Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
		}
			break;

		case b2Manifold.Type.e_faceA:
		{
				normal = GlobalMembers.b2Mul(xfA.q, pc.localNormal);
				b2Vec2 planePoint = GlobalMembers.b2Mul(xfA, pc.localPoint);

				b2Vec2 clipPoint = GlobalMembers.b2Mul(xfB, pc.localPoints[index]);
				separation = GlobalMembers.b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
				point = clipPoint;
		}
			break;

		case b2Manifold.Type.e_faceB:
		{
				normal = GlobalMembers.b2Mul(xfB.q, pc.localNormal);
				b2Vec2 planePoint = GlobalMembers.b2Mul(xfB, pc.localPoint);

				b2Vec2 clipPoint = GlobalMembers.b2Mul(xfA, pc.localPoints[index]);
				separation = GlobalMembers.b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
				point = clipPoint;

				// Ensure normal points from A to B
				normal = -normal;
		}
			break;
		}
	}

	public b2Vec2 normal = new b2Vec2();
	public b2Vec2 point = new b2Vec2();
	public float separation;
}
