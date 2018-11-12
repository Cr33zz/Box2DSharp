using System;
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

public enum b2JointType
{
	e_unknownJoint,
	e_revoluteJoint,
	e_prismaticJoint,
	e_distanceJoint,
	e_pulleyJoint,
	e_mouseJoint,
	e_gearJoint,
	e_wheelJoint,
	e_weldJoint,
	e_frictionJoint,
	e_ropeJoint,
	e_motorJoint
}

public enum b2LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
}

public class b2Jacobian
{
	public b2Vec2 linear = new b2Vec2();
	public float angularA;
	public float angularB;
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
public class b2JointEdge
{
	public b2Body other; ///< provides quick access to the other body attached.
	public b2Joint joint; ///< the joint
	public b2JointEdge prev; ///< the previous joint edge in the body's joint list
	public b2JointEdge next; ///< the next joint edge in the body's joint list
}

/// Joint definitions are used to construct joints.
public class b2JointDef
{
	public b2JointDef()
	{
		type = b2JointType.e_unknownJoint;
		userData = null;
		bodyA = null;
		bodyB = null;
		collideConnected = false;
	}

	/// The joint type is set automatically for concrete joint types.
	public b2JointType type;

	/// Use this to attach application specific data to your joints.
	public object userData;

	/// The first attached body.
	public b2Body bodyA;

	/// The second attached body.
	public b2Body bodyB;

	/// Set this flag to true if the attached bodies should collide.
	public bool collideConnected;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
public abstract class b2Joint : System.IDisposable
{

	/// Get the type of the concrete joint.

	public b2JointType GetType()
	{
		return m_type;
	}

	/// Get the first body attached to this joint.
	public b2Body GetBodyA()
	{
		return m_bodyA;
	}

	/// Get the second body attached to this joint.
	public b2Body GetBodyB()
	{
		return m_bodyB;
	}

	/// Get the anchor point on bodyA in world coordinates.
	public abstract b2Vec2 GetAnchorA();

	/// Get the anchor point on bodyB in world coordinates.
	public abstract b2Vec2 GetAnchorB();

	/// Get the reaction force on bodyB at the joint anchor in Newtons.
	public abstract b2Vec2 GetReactionForce(float inv_dt);

	/// Get the reaction torque on bodyB in N*m.
	public abstract float GetReactionTorque(float inv_dt);

	/// Get the next joint the world joint list.
	public b2Joint GetNext()
	{
		return m_next;
	}

	/// Get the user data pointer.
	public object GetUserData()
	{
		return m_userData;
	}

	/// Set the user data pointer.
	public void SetUserData(object data)
	{
		m_userData = data;
	}

	/// Short-cut function to determine if either body is inactive.
	public bool IsActive()
	{
		return m_bodyA.IsActive() && m_bodyB.IsActive();
	}

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	public bool GetCollideConnected()
	{
		return m_collideConnected;
	}

	/// Dump this joint to the log file.
	public virtual void Dump()
	{
		Console.Write("// Dump is not supported for this joint type.\n");
	}

	/// Shift the origin for any points stored in world coordinates.
	public virtual void ShiftOrigin(b2Vec2 newOrigin)
	{
	}

    internal static b2Joint Create(b2JointDef def)
	{
		b2Joint joint = null;

		switch (def.type)
		{
		case b2JointType.e_distanceJoint:
		{
				joint = new b2DistanceJoint((b2DistanceJointDef)def);
		}
			break;

		case b2JointType.e_mouseJoint:
		{
				joint = new b2MouseJoint((b2MouseJointDef)def);
		}
			break;

		case b2JointType.e_prismaticJoint:
		{
				joint = new b2PrismaticJoint((b2PrismaticJointDef)def);
		}
			break;

		case b2JointType.e_revoluteJoint:
		{
				joint = new b2RevoluteJoint((b2RevoluteJointDef)def);
		}
			break;

		case b2JointType.e_pulleyJoint:
		{
				joint = new b2PulleyJoint((b2PulleyJointDef)def);
		}
			break;

		case b2JointType.e_gearJoint:
		{
				joint = new b2GearJoint((b2GearJointDef)def);
		}
			break;

		case b2JointType.e_wheelJoint:
		{
				joint = new b2WheelJoint((b2WheelJointDef)def);
		}
			break;

		case b2JointType.e_weldJoint:
		{
				joint = new b2WeldJoint((b2WeldJointDef)def);
		}
			break;

		case b2JointType.e_frictionJoint:
		{
				joint = new b2FrictionJoint((b2FrictionJointDef)def);
		}
			break;

		case b2JointType.e_ropeJoint:
		{
				joint = new b2RopeJoint((b2RopeJointDef)def);
		}
			break;

		case b2JointType.e_motorJoint:
		{
				joint = new b2MotorJoint((b2MotorJointDef)def);
		}
			break;

		default:
			Debug.Assert(false);
			break;
		}

		return joint;
	}
    internal static void Destroy(b2Joint joint)
	{		
	}

	protected b2Joint(b2JointDef def)
	{
		Debug.Assert(def.bodyA != def.bodyB);

		m_type = def.type;
		m_prev = null;
		m_next = null;
		m_bodyA = def.bodyA;
		m_bodyB = def.bodyB;
		m_index = 0;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		m_userData = def.userData;

		m_edgeA.joint = null;
		m_edgeA.other = null;
		m_edgeA.prev = null;
		m_edgeA.next = null;

		m_edgeB.joint = null;
		m_edgeB.other = null;
		m_edgeB.prev = null;
		m_edgeB.next = null;
	}
	public virtual void Dispose()
	{
	}

	protected abstract void InitVelocityConstraints(b2SolverData data);
	protected abstract void SolveVelocityConstraints(b2SolverData data);

	// This returns true if the position errors are within tolerance.
	protected abstract bool SolvePositionConstraints(b2SolverData data);

    internal b2JointType m_type;
    internal b2Joint m_prev;
    internal b2Joint m_next;
    internal b2JointEdge m_edgeA = new b2JointEdge();
    internal b2JointEdge m_edgeB = new b2JointEdge();
    internal b2Body m_bodyA;
    internal b2Body m_bodyB;

    internal int m_index;

    internal bool m_islandFlag;
    internal bool m_collideConnected;

	protected object m_userData;
}


