package org.firstinspires.ftc.teamcode.utils;

public final class MathUtil {
    private MathUtil() {
        throw new AssertionError("utility class");
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value Value to clip.
     * @param deadband Range around zero.
     * @return The value after the deadband is applied.
     */
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Returns modulus of input.
     *
     * @param input Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Wraps an angle to the range -pi to pi radians.
     *
     * @param angleRadians Angle to wrap in radians.
     * @return The wrapped angle.
     */
    public static double angleModulus(double angleRadians) {
        return inputModulus(angleRadians, -Math.PI, Math.PI);
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue The value to end at.
     * @param t How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    @SuppressWarnings("ParameterName")
    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * MathUtil.clamp(t, 0, 1);
    }
}
