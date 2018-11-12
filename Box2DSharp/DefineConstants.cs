internal static class DefineConstants
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