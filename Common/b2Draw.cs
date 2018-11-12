/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

/// Color for debug drawing. Each value has the range [0,1].
public class b2Color
{
	public b2Color()
	{
	}
	public b2Color(float rIn, float gIn, float bIn, float aIn = 1.0f)
	{
		r = rIn;
		g = gIn;
		b = bIn;
		a = aIn;
	}

	public void Set(float rIn, float gIn, float bIn, float aIn = 1.0f)
	{
		r = rIn;
		g = gIn;
		b = bIn;
		a = aIn;
	}

	public float r;
	public float g;
	public float b;
	public float a;
}

/// Implement and register this class with a b2World to provide debug drawing of physics
/// entities in your game.
public abstract class b2Draw : System.IDisposable
{
	public b2Draw()
	{
		m_drawFlags = 0;
	}

	public virtual void Dispose()
	{
	}

	public enum DrawFlags
	{
		e_shapeBit = 0x0001, ///< draw shapes
		e_jointBit = 0x0002, ///< draw joint connections
		e_aabbBit = 0x0004, ///< draw axis aligned bounding boxes
		e_pairBit = 0x0008, ///< draw broad-phase pairs
		e_centerOfMassBit = 0x0010 ///< draw center of mass frame
	}

	/// Set the drawing flags.
	public void SetFlags(DrawFlags flags)
	{
		m_drawFlags = flags;
	}

	/// Get the drawing flags.
	public DrawFlags GetFlags()
	{
		return m_drawFlags;
	}

	/// Append flags to the current flags.
	public void AppendFlags(DrawFlags flags)
	{
		m_drawFlags |= flags;
	}

	/// Clear flags from the current flags.
	public void ClearFlags(DrawFlags flags)
	{
		m_drawFlags &= ~flags;
	}

	/// Draw a closed polygon provided in CCW order.
	public abstract void DrawPolygon(b2Vec2[] vertices, int vertexCount, b2Color color);

	/// Draw a solid closed polygon provided in CCW order.
	public abstract void DrawSolidPolygon(b2Vec2[] vertices, int vertexCount, b2Color color);

	/// Draw a circle.
	public abstract void DrawCircle(b2Vec2 center, float radius, b2Color color);

	/// Draw a solid circle.
	public abstract void DrawSolidCircle(b2Vec2 center, float radius, b2Vec2 axis, b2Color color);

	/// Draw a line segment.
	public abstract void DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color);

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	public abstract void DrawTransform(b2Transform xf);

	/// Draw a point.
	public abstract void DrawPoint(b2Vec2 p, float size, b2Color color);

	protected DrawFlags m_drawFlags;
}
