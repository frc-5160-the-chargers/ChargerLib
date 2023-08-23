package frc.chargers.utils


/**
 * Convenience object for more concisely creating arrays until
 * [KT-43871](https://youtrack.jetbrains.com/issue/KT-43871) is resolved.
 * Works by implementing the get operator, allowing usage of square brackets.
 * If possible, prefer using EJML equations for initializing matrices.
 *
 * Typical array creation:
 * ```
 * val array1D = doubleArrayOf(1.0, 2.0, 3.0)
 * val array2D =
 *     arrayOf(
 *         doubleArrayOf(1.0, 2.0, 3.0),
 *         doubleArrayOf(4.0, 5.0, 6.0),
 *         doubleArrayOf(7.0, 8.0, 9.0),
 *     )
 * ```
 *
 * Usage of this class:
 * ```
 * val array1D = a[1.0, 2.0, 3.0]
 * val array2D = a[
 *                  a[1.0, 2.0, 3.0],
 *                  a[4.0, 5.0, 6.0],
 *                  a[7.0, 8.0, 9.0]
 *               ]
 * ```
 */
public object a {
    public operator fun get(vararg elements: Double): DoubleArray = doubleArrayOf(*elements)
    public operator fun get(vararg elements: Float): FloatArray = floatArrayOf(*elements)
    public inline operator fun <reified T> get(vararg elements: T): Array<T> = arrayOf(*elements)
}