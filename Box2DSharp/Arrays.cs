internal static class Arrays
{
    public static T[] InitializeWithDefaultInstances<T>(int length) where T : new()
    {
        T[] array = new T[length];
        for (int i = 0; i < length; i++)
        {
            array[i] = new T();
        }
        return array;
    }

    public static void DeleteArray<T>(T[] array) where T : System.IDisposable
    {
        foreach (T element in array)
        {
            if (element != null)
                element.Dispose();
        }
    }
}