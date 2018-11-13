/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#if !NDEBUG
#define b2DEBUG
#endif

internal static class Settings
{
    public const float b2_pi = 3.14159265359f;
    public const int b2_maxManifoldPoints = 2;
    public const float b2_polygonRadius = 2.0f * b2_linearSlop;
    public const int b2_maxPolygonVertices = 8;
    public const float b2_aabbExtension = 0.1f;
    public const float b2_aabbMultiplier = 2.0f;
    public const float b2_linearSlop = 0.005f;
    public const int b2_maxSubSteps = 8;
    public const int b2_maxTOIContacts = 32;
    public const float b2_velocityThreshold = 1.0f;
    public const float b2_maxLinearCorrection = 0.2f;
    public const float b2_maxTranslation = 2.0f;
    public const float b2_maxTranslationSquared = b2_maxTranslation * b2_maxTranslation;
    public const float b2_baumgarte = 0.2f;
    public const float b2_toiBaugarte = 0.75f;
    public const float b2_timeToSleep = 0.5f;
    public const float b2_linearSleepTolerance = 0.01f;
    public const int b2_nullNode = -1;
    public const float b2_maxRotation = 0.5f * b2_pi;
    public const float b2_maxRotationSquared = b2_maxRotation * b2_maxRotation;
    public const float b2_maxAngularCorrection = 8.0f / 180.0f * b2_pi;

    public const int B2_DEBUG_SOLVER = 0;
}

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
public class b2Version
{
    public b2Version(int major, int minor, int revision)
    {
        this.major = major;
        this.minor = minor;
        this.revision = revision;
    }

    public int major; ///< significant changes
	public int minor; ///< incremental changes
	public int revision; ///< bug fixes
}