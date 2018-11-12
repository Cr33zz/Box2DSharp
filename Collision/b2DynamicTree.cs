#define b2DEBUG

using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;

/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
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

/// A node in the dynamic tree. The client does not interact with this directly.
public class b2TreeNode
{
	public bool IsLeaf()
	{
		return child1 == DefineConstants.b2_nullNode;
	}

	/// Enlarged AABB
	public b2AABB aabb = new b2AABB();

	public object userData;

    public int parentOrNext;

    public int child1;
	public int child2;

	// leaf = 0, free node = -1
	public int height;
}

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
public class b2DynamicTree : System.IDisposable
{
	/// Constructing the tree initializes the node pool.
	public b2DynamicTree()
	{
		m_root = DefineConstants.b2_nullNode;

		m_nodeCapacity = 16;
		m_nodeCount = 0;
		m_nodes = new b2TreeNode[m_nodeCapacity];

		// Build a linked list for the free list.
		for (int i = 0; i < m_nodeCapacity - 1; ++i)
		{
            m_nodes[i] = new b2TreeNode();
			m_nodes[i].parentOrNext = i + 1;
			m_nodes[i].height = -1;
		}

	    m_nodes[m_nodeCapacity - 1] = new b2TreeNode();
        m_nodes[m_nodeCapacity - 1].parentOrNext = DefineConstants.b2_nullNode;
		m_nodes[m_nodeCapacity - 1].height = -1;
		m_freeList = 0;

		m_path = 0;

		m_insertionCount = 0;
	}

	/// Destroy the tree, freeing the node pool.
	public void Dispose()
	{
		// This frees the entire tree in one shot.
		m_nodes = null;
	}

	/// Create a proxy. Provide a tight fitting AABB and a userData pointer.

	// Create a proxy in the tree as a leaf node. We return the index
	// of the node instead of a pointer so that we can grow
	// the node pool.
	public int CreateProxy(b2AABB aabb, object userData)
	{
		int proxyId = AllocateNode();

		// Fatten the aabb.
		b2Vec2 r = new b2Vec2(DefineConstants.b2_aabbExtension, DefineConstants.b2_aabbExtension);
		m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
		m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
		m_nodes[proxyId].userData = userData;
		m_nodes[proxyId].height = 0;

		InsertLeaf(proxyId);

		return proxyId;
	}

	/// Destroy a proxy. This asserts if the id is invalid.
	public void DestroyProxy(int proxyId)
	{
		Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
		Debug.Assert(m_nodes[proxyId].IsLeaf());

		RemoveLeaf(proxyId);
		FreeNode(proxyId);
	}

	/// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
	/// then the proxy is removed from the tree and re-inserted. Otherwise
	/// the function returns immediately.
	/// @return true if the proxy was re-inserted.
	public bool MoveProxy(int proxyId, b2AABB aabb, b2Vec2 displacement)
	{
		Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);

		Debug.Assert(m_nodes[proxyId].IsLeaf());

		if (m_nodes[proxyId].aabb.Contains(aabb))
		{
			return false;
		}

		RemoveLeaf(proxyId);

		// Extend AABB.
		b2AABB b = aabb;
		b2Vec2 r = new b2Vec2(DefineConstants.b2_aabbExtension, DefineConstants.b2_aabbExtension);
		b.lowerBound = b.lowerBound - r;
		b.upperBound = b.upperBound + r;

		// Predict AABB displacement.
		b2Vec2 d = DefineConstants.b2_aabbMultiplier * displacement;

		if (d.x < 0.0f)
		{
			b.lowerBound.x += d.x;
		}
		else
		{
			b.upperBound.x += d.x;
		}

		if (d.y < 0.0f)
		{
			b.lowerBound.y += d.y;
		}
		else
		{
			b.upperBound.y += d.y;
		}

		m_nodes[proxyId].aabb = b;

		InsertLeaf(proxyId);
		return true;
	}

	/// Get proxy user data.
	/// @return the proxy user data or 0 if the id is invalid.
	public object GetUserData(int proxyId)
	{
		Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
		return m_nodes[proxyId].userData;
	}

	/// Get the fat AABB for a proxy.
	public b2AABB GetFatAABB(int proxyId)
	{
		Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
		return m_nodes[proxyId].aabb;
	}

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	public void Query<T>(T callback, b2AABB aabb)
	{
	    Stack<int> stack = new Stack<int>(256);
        stack.Push(m_root);

		while (stack.Count > 0)
		{
			int nodeId = stack.Pop();
			if (nodeId == DefineConstants.b2_nullNode)
			{
				continue;
			}

			b2TreeNode node = m_nodes[nodeId];

			if (GlobalMembers.b2TestOverlap(node.aabb, aabb))
			{
				if (node.IsLeaf())
				{
					bool proceed = callback.QueryCallback(nodeId);
					if (proceed == false)
					{
						return;
					}
				}
				else
				{
					stack.Push(node.child1);
					stack.Push(node.child2);
				}
			}
		}
	}

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	public void RayCast<T>(T callback, b2RayCastInput input)
	{
		b2Vec2 p1 = new b2Vec2(input.p1);
		b2Vec2 p2 = new b2Vec2(input.p2);
		b2Vec2 r = p2 - p1;
		Debug.Assert(r.LengthSquared() > 0.0f);
		r.Normalize();

		// v is perpendicular to the segment.
		b2Vec2 v = GlobalMembers.b2Cross(1.0f, r);
		b2Vec2 abs_v = GlobalMembers.b2Abs(v);

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)

		float maxFraction = input.maxFraction;

		// Build a bounding box for the segment.
		b2AABB segmentAABB = new b2AABB();
		{
			b2Vec2 t = p1 + maxFraction * (p2 - p1);
			segmentAABB.lowerBound = GlobalMembers.b2Min(p1, t);
			segmentAABB.upperBound = GlobalMembers.b2Max(p1, t);
		}

		Stack<int> stack = new Stack<int>(256);
		stack.Push(m_root);

		while (stack.Count > 0)
		{
			int nodeId = stack.Pop();
			if (nodeId == DefineConstants.b2_nullNode)
			{
				continue;
			}

			b2TreeNode node = m_nodes[nodeId];

			if (GlobalMembers.b2TestOverlap(node.aabb, segmentAABB) == false)
			{
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			b2Vec2 c = node.aabb.GetCenter();
			b2Vec2 h = node.aabb.GetExtents();
			float separation = GlobalMembers.b2Abs(GlobalMembers.b2Dot(v, p1 - c)) - GlobalMembers.b2Dot(abs_v, h);
			if (separation > 0.0f)
			{
				continue;
			}

			if (node.IsLeaf())
			{
				b2RayCastInput subInput = new b2RayCastInput();
				subInput.p1 = input.p1;
				subInput.p2 = input.p2;
				subInput.maxFraction = maxFraction;

				float value = callback.RayCastCallback(subInput, nodeId);

				if (value == 0.0f)
				{
					// The client has terminated the ray cast.
					return;
				}

				if (value > 0.0f)
				{
					// Update segment bounding box.
					maxFraction = value;
					b2Vec2 t = p1 + maxFraction * (p2 - p1);
					segmentAABB.lowerBound = GlobalMembers.b2Min(p1, t);
					segmentAABB.upperBound = GlobalMembers.b2Max(p1, t);
				}
			}
			else
			{
				stack.Push(node.child1);
				stack.Push(node.child2);
			}
		}
	}

	/// Validate this tree. For testing.
	public void Validate()
	{
#if b2DEBUG
		ValidateStructure(m_root);
		ValidateMetrics(m_root);

		int freeCount = 0;
		int freeIndex = m_freeList;
		while (freeIndex != DefineConstants.b2_nullNode)
		{
			Debug.Assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
			freeIndex = m_nodes[freeIndex].parentOrNext;
			++freeCount;
		}

		Debug.Assert(GetHeight() == ComputeHeight());

		Debug.Assert(m_nodeCount + freeCount == m_nodeCapacity);
#endif
	}

	/// Compute the height of the binary tree in O(N) time. Should not be
	/// called often.
	public int GetHeight()
	{
		if (m_root == DefineConstants.b2_nullNode)
		{
			return 0;
		}

		return m_nodes[m_root].height;
	}

	/// Get the maximum balance of an node in the tree. The balance is the difference
	/// in height of the two children of a node.
	public int GetMaxBalance()
	{
		int maxBalance = 0;
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			b2TreeNode node = m_nodes[i];
			if (node.height <= 1)
			{
				continue;
			}

			Debug.Assert(node.IsLeaf() == false);

			int child1 = node.child1;
			int child2 = node.child2;
			int balance = GlobalMembers.b2Abs(m_nodes[child2].height - m_nodes[child1].height);
			maxBalance = GlobalMembers.b2Max(maxBalance, balance);
		}

		return maxBalance;
	}

	/// Get the ratio of the sum of the node areas to the root area.

	//
	public float GetAreaRatio()
	{
		if (m_root == DefineConstants.b2_nullNode)
		{
			return 0.0f;
		}

		b2TreeNode root = m_nodes[m_root];
		float rootArea = root.aabb.GetPerimeter();

		float totalArea = 0.0f;
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			b2TreeNode node = m_nodes[i];
			if (node.height < 0)
			{
				// Free node in pool
				continue;
			}

			totalArea += node.aabb.GetPerimeter();
		}

		return totalArea / rootArea;
	}

	/// Build an optimal tree. Very expensive. For testing.
	public void RebuildBottomUp()
	{
		int[] nodes = new int[m_nodeCount];
		int count = 0;

		// Build array of leaves. Free the rest.
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			if (m_nodes[i].height < 0)
			{
				// free node in pool
				continue;
			}

			if (m_nodes[i].IsLeaf())
			{
				m_nodes[i].parentOrNext = DefineConstants.b2_nullNode;
				nodes[count] = i;
				++count;
			}
			else
			{
				FreeNode(i);
			}
		}

		while (count > 1)
		{
			float minCost = float.MaxValue;
			int iMin = -1;
			int jMin = -1;
			for (int i = 0; i < count; ++i)
			{
				b2AABB aabbi = m_nodes[nodes[i]].aabb;

				for (int j = i + 1; j < count; ++j)
				{
					b2AABB aabbj = m_nodes[nodes[j]].aabb;
					b2AABB b = new b2AABB();
					b.Combine(aabbi, aabbj);
					float cost = b.GetPerimeter();
					if (cost < minCost)
					{
						iMin = i;
						jMin = j;
						minCost = cost;
					}
				}
			}

			int index1 = nodes[iMin];
			int index2 = nodes[jMin];
			b2TreeNode child1 = m_nodes[index1];
			b2TreeNode child2 = m_nodes[index2];

			int parentIndex = AllocateNode();
			b2TreeNode parentOrNext = m_nodes[parentIndex];
			parentOrNext.child1 = index1;
			parentOrNext.child2 = index2;
			parentOrNext.height = 1 + GlobalMembers.b2Max(child1.height, child2.height);
			parentOrNext.aabb.Combine(child1.aabb, child2.aabb);
			parentOrNext.parentOrNext = DefineConstants.b2_nullNode;

			child1.parentOrNext = parentIndex;
			child2.parentOrNext = parentIndex;

			nodes[jMin] = nodes[count - 1];
			nodes[iMin] = parentIndex;
			--count;
		}

		m_root = nodes[0];
		nodes = null;

		Validate();
	}

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	public void ShiftOrigin(b2Vec2 newOrigin)
	{
		// Build array of leaves. Free the rest.
		for (int i = 0; i < m_nodeCapacity; ++i)
		{
			m_nodes[i].aabb.lowerBound -= newOrigin;
			m_nodes[i].aabb.upperBound -= newOrigin;
		}
	}



	// Allocate a node from the pool. Grow the pool if necessary.
	private int AllocateNode()
	{
		// Expand the node pool as needed.
		if (m_freeList == DefineConstants.b2_nullNode)
		{
			Debug.Assert(m_nodeCount == m_nodeCapacity);

			// The free list is empty. Rebuild a bigger pool.
			b2TreeNode[] oldNodes = m_nodes;
			m_nodeCapacity *= 2;
			m_nodes = new b2TreeNode[m_nodeCapacity];
			Array.Copy(oldNodes, m_nodes, m_nodeCount);
			
			// Build a linked list for the free list. The parentOrNext
			// pointer becomes the "parentOrNext" pointer.
			for (int i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
			{
                m_nodes[i] = new b2TreeNode();
				m_nodes[i].parentOrNext = i + 1;
				m_nodes[i].height = -1;
			}
            m_nodes[m_nodeCapacity - 1] = new b2TreeNode();
			m_nodes[m_nodeCapacity - 1].parentOrNext = DefineConstants.b2_nullNode;
			m_nodes[m_nodeCapacity - 1].height = -1;
			m_freeList = m_nodeCount;
		}

		// Peel a node off the free list.
		int nodeId = m_freeList;
		m_freeList = m_nodes[nodeId].parentOrNext;
		m_nodes[nodeId].parentOrNext = DefineConstants.b2_nullNode;
		m_nodes[nodeId].child1 = DefineConstants.b2_nullNode;
		m_nodes[nodeId].child2 = DefineConstants.b2_nullNode;
		m_nodes[nodeId].height = 0;
		m_nodes[nodeId].userData = null;
		++m_nodeCount;
		return nodeId;
	}

	// Return a node to the pool.
	private void FreeNode(int nodeId)
	{
		Debug.Assert(0 <= nodeId && nodeId < m_nodeCapacity);
		Debug.Assert(0 < m_nodeCount);
		m_nodes[nodeId].parentOrNext = m_freeList;
		m_nodes[nodeId].height = -1;
		m_freeList = nodeId;
		--m_nodeCount;
	}

	private void InsertLeaf(int leaf)
	{
		++m_insertionCount;

		if (m_root == DefineConstants.b2_nullNode)
		{
			m_root = leaf;
			m_nodes[m_root].parentOrNext = DefineConstants.b2_nullNode;
			return;
		}

		// Find the best sibling for this node
		b2AABB leafAABB = m_nodes[leaf].aabb;
		int index = m_root;
		while (m_nodes[index].IsLeaf() == false)
		{
			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;

			float area = m_nodes[index].aabb.GetPerimeter();

			b2AABB combinedAABB = new b2AABB();
			combinedAABB.Combine(m_nodes[index].aabb, leafAABB);
			float combinedArea = combinedAABB.GetPerimeter();

			// Cost of creating a new parentOrNext for this node and the new leaf
			float cost = 2.0f * combinedArea;

			// Minimum cost of pushing the leaf further down the tree
			float inheritanceCost = 2.0f * (combinedArea - area);

			// Cost of descending into child1
			float cost1;
			if (m_nodes[child1].IsLeaf())
			{
				b2AABB aabb = new b2AABB();
				aabb.Combine(leafAABB, m_nodes[child1].aabb);
				cost1 = aabb.GetPerimeter() + inheritanceCost;
			}
			else
			{
				b2AABB aabb = new b2AABB();
				aabb.Combine(leafAABB, m_nodes[child1].aabb);
				float oldArea = m_nodes[child1].aabb.GetPerimeter();
				float newArea = aabb.GetPerimeter();
				cost1 = (newArea - oldArea) + inheritanceCost;
			}

			// Cost of descending into child2
			float cost2;
			if (m_nodes[child2].IsLeaf())
			{
				b2AABB aabb = new b2AABB();
				aabb.Combine(leafAABB, m_nodes[child2].aabb);
				cost2 = aabb.GetPerimeter() + inheritanceCost;
			}
			else
			{
				b2AABB aabb = new b2AABB();
				aabb.Combine(leafAABB, m_nodes[child2].aabb);
				float oldArea = m_nodes[child2].aabb.GetPerimeter();
				float newArea = aabb.GetPerimeter();
				cost2 = newArea - oldArea + inheritanceCost;
			}

			// Descend according to the minimum cost.
			if (cost < cost1 && cost < cost2)
			{
				break;
			}

			// Descend
			if (cost1 < cost2)
			{
				index = child1;
			}
			else
			{
				index = child2;
			}
		}

		int sibling = index;

		// Create a new parentOrNext.
		int oldParent = m_nodes[sibling].parentOrNext;
		int newParent = AllocateNode();
		m_nodes[newParent].parentOrNext = oldParent;
		m_nodes[newParent].userData = null;
		m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
		m_nodes[newParent].height = m_nodes[sibling].height + 1;

		if (oldParent != DefineConstants.b2_nullNode)
		{
			// The sibling was not the root.
			if (m_nodes[oldParent].child1 == sibling)
			{
				m_nodes[oldParent].child1 = newParent;
			}
			else
			{
				m_nodes[oldParent].child2 = newParent;
			}

			m_nodes[newParent].child1 = sibling;
			m_nodes[newParent].child2 = leaf;
			m_nodes[sibling].parentOrNext = newParent;
			m_nodes[leaf].parentOrNext = newParent;
		}
		else
		{
			// The sibling was the root.
			m_nodes[newParent].child1 = sibling;
			m_nodes[newParent].child2 = leaf;
			m_nodes[sibling].parentOrNext = newParent;
			m_nodes[leaf].parentOrNext = newParent;
			m_root = newParent;
		}

		// Walk back up the tree fixing heights and AABBs
		index = m_nodes[leaf].parentOrNext;
		while (index != DefineConstants.b2_nullNode)
		{
			index = Balance(index);

			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;

			Debug.Assert(child1 != DefineConstants.b2_nullNode);
			Debug.Assert(child2 != DefineConstants.b2_nullNode);

			m_nodes[index].height = 1 + GlobalMembers.b2Max(m_nodes[child1].height, m_nodes[child2].height);
			m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

			index = m_nodes[index].parentOrNext;
		}

		//Validate();
	}
	private void RemoveLeaf(int leaf)
	{
		if (leaf == m_root)
		{
			m_root = DefineConstants.b2_nullNode;
			return;
		}

		int parentOrNext = m_nodes[leaf].parentOrNext;
		int grandParent = m_nodes[parentOrNext].parentOrNext;
		int sibling;
		if (m_nodes[parentOrNext].child1 == leaf)
		{
			sibling = m_nodes[parentOrNext].child2;
		}
		else
		{
			sibling = m_nodes[parentOrNext].child1;
		}

		if (grandParent != DefineConstants.b2_nullNode)
		{
			// Destroy parentOrNext and connect sibling to grandParent.
			if (m_nodes[grandParent].child1 == parentOrNext)
			{
				m_nodes[grandParent].child1 = sibling;
			}
			else
			{
				m_nodes[grandParent].child2 = sibling;
			}
			m_nodes[sibling].parentOrNext = grandParent;
			FreeNode(parentOrNext);

			// Adjust ancestor bounds.
			int index = grandParent;
			while (index != DefineConstants.b2_nullNode)
			{
				index = Balance(index);

				int child1 = m_nodes[index].child1;
				int child2 = m_nodes[index].child2;

				m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
				m_nodes[index].height = 1 + GlobalMembers.b2Max(m_nodes[child1].height, m_nodes[child2].height);

				index = m_nodes[index].parentOrNext;
			}
		}
		else
		{
			m_root = sibling;
			m_nodes[sibling].parentOrNext = DefineConstants.b2_nullNode;
			FreeNode(parentOrNext);
		}

		//Validate();
	}


	// Perform a left or right rotation if node A is imbalanced.
	// Returns the new root index.
	private int Balance(int iA)
	{
		Debug.Assert(iA != DefineConstants.b2_nullNode);

		b2TreeNode A = m_nodes[iA];
		if (A.IsLeaf() || A.height < 2)
		{
			return iA;
		}

		int iB = A.child1;
		int iC = A.child2;
		Debug.Assert(0 <= iB && iB < m_nodeCapacity);
		Debug.Assert(0 <= iC && iC < m_nodeCapacity);

		b2TreeNode B = m_nodes[iB];
		b2TreeNode C = m_nodes[iC];

		int balance = C.height - B.height;

		// Rotate C up
		if (balance > 1)
		{
			int iF = C.child1;
			int iG = C.child2;
			b2TreeNode F = m_nodes[iF];
			b2TreeNode G = m_nodes[iG];
			Debug.Assert(0 <= iF && iF < m_nodeCapacity);
			Debug.Assert(0 <= iG && iG < m_nodeCapacity);

			// Swap A and C
			C.child1 = iA;
			C.parentOrNext = A.parentOrNext;
			A.parentOrNext = iC;

			// A's old parentOrNext should point to C
			if (C.parentOrNext != DefineConstants.b2_nullNode)
			{
				if (m_nodes[C.parentOrNext].child1 == iA)
				{
					m_nodes[C.parentOrNext].child1 = iC;
				}
				else
				{
					Debug.Assert(m_nodes[C.parentOrNext].child2 == iA);
					m_nodes[C.parentOrNext].child2 = iC;
				}
			}
			else
			{
				m_root = iC;
			}

			// Rotate
			if (F.height > G.height)
			{
				C.child2 = iF;
				A.child2 = iG;
				G.parentOrNext = iA;
				A.aabb.Combine(B.aabb, G.aabb);
				C.aabb.Combine(A.aabb, F.aabb);

				A.height = 1 + GlobalMembers.b2Max(B.height, G.height);
				C.height = 1 + GlobalMembers.b2Max(A.height, F.height);
			}
			else
			{
				C.child2 = iG;
				A.child2 = iF;
				F.parentOrNext = iA;
				A.aabb.Combine(B.aabb, F.aabb);
				C.aabb.Combine(A.aabb, G.aabb);

				A.height = 1 + GlobalMembers.b2Max(B.height, F.height);
				C.height = 1 + GlobalMembers.b2Max(A.height, G.height);
			}

			return iC;
		}

		// Rotate B up
		if (balance < -1)
		{
			int iD = B.child1;
			int iE = B.child2;
			b2TreeNode D = m_nodes[iD];
			b2TreeNode E = m_nodes[iE];
			Debug.Assert(0 <= iD && iD < m_nodeCapacity);
			Debug.Assert(0 <= iE && iE < m_nodeCapacity);

			// Swap A and B
			B.child1 = iA;
			B.parentOrNext = A.parentOrNext;
			A.parentOrNext = iB;

			// A's old parentOrNext should point to B
			if (B.parentOrNext != DefineConstants.b2_nullNode)
			{
				if (m_nodes[B.parentOrNext].child1 == iA)
				{
					m_nodes[B.parentOrNext].child1 = iB;
				}
				else
				{
					Debug.Assert(m_nodes[B.parentOrNext].child2 == iA);
					m_nodes[B.parentOrNext].child2 = iB;
				}
			}
			else
			{
				m_root = iB;
			}

			// Rotate
			if (D.height > E.height)
			{
				B.child2 = iD;
				A.child1 = iE;
				E.parentOrNext = iA;
				A.aabb.Combine(C.aabb, E.aabb);
				B.aabb.Combine(A.aabb, D.aabb);

				A.height = 1 + GlobalMembers.b2Max(C.height, E.height);
				B.height = 1 + GlobalMembers.b2Max(A.height, D.height);
			}
			else
			{
				B.child2 = iE;
				A.child1 = iD;
				D.parentOrNext = iA;
				A.aabb.Combine(C.aabb, D.aabb);
				B.aabb.Combine(A.aabb, E.aabb);

				A.height = 1 + GlobalMembers.b2Max(C.height, D.height);
				B.height = 1 + GlobalMembers.b2Max(A.height, E.height);
			}

			return iB;
		}

		return iA;
	}

	private int ComputeHeight()
	{
		int height = ComputeHeight(m_root);
		return height;
	}

	// Compute the height of a sub-tree.
	private int ComputeHeight(int nodeId)
	{
		Debug.Assert(0 <= nodeId && nodeId < m_nodeCapacity);
		b2TreeNode node = m_nodes[nodeId];

		if (node.IsLeaf())
		{
			return 0;
		}

		int height1 = ComputeHeight(node.child1);
		int height2 = ComputeHeight(node.child2);
		return 1 + GlobalMembers.b2Max(height1, height2);
	}

	private void ValidateStructure(int index)
	{
		if (index == DefineConstants.b2_nullNode)
		{
			return;
		}

		if (index == m_root)
		{
			Debug.Assert(m_nodes[index].parentOrNext == DefineConstants.b2_nullNode);
		}

		b2TreeNode node = m_nodes[index];

		int child1 = node.child1;
		int child2 = node.child2;

		if (node.IsLeaf())
		{
			Debug.Assert(child1 == DefineConstants.b2_nullNode);
			Debug.Assert(child2 == DefineConstants.b2_nullNode);
			Debug.Assert(node.height == 0);
			return;
		}

		Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
		Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

		Debug.Assert(m_nodes[child1].parentOrNext == index);
		Debug.Assert(m_nodes[child2].parentOrNext == index);

		ValidateStructure(child1);
		ValidateStructure(child2);
	}

    private void ValidateMetrics(int index)
	{
		if (index == DefineConstants.b2_nullNode)
		{
			return;
		}

		b2TreeNode node = m_nodes[index];

		int child1 = node.child1;
		int child2 = node.child2;

		if (node.IsLeaf())
		{
			Debug.Assert(child1 == DefineConstants.b2_nullNode);
			Debug.Assert(child2 == DefineConstants.b2_nullNode);
			Debug.Assert(node.height == 0);
			return;
		}

		Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
		Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

		int height1 = m_nodes[child1].height;
		int height2 = m_nodes[child2].height;
		int height;
		height = 1 + GlobalMembers.b2Max(height1, height2);
		Debug.Assert(node.height == height);

		b2AABB aabb = new b2AABB();
		aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

		Debug.Assert(aabb.lowerBound == node.aabb.lowerBound);
		Debug.Assert(aabb.upperBound == node.aabb.upperBound);

		ValidateMetrics(child1);
		ValidateMetrics(child2);
	}

	private int m_root;

	private b2TreeNode[] m_nodes;
	private int m_nodeCount;
	private int m_nodeCapacity;

	private int m_freeList;

	/// This is used to incrementally traverse the tree for re-balancing.
	private uint m_path;

	private int m_insertionCount;
}
