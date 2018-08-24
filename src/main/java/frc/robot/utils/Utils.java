package frc.robot.utils;

public class Utils {
    public static double lerp(double val, double minIn, double maxIn, double minOut, double maxOut) {
        final double percentComplete = (val - minIn) / (maxIn - minIn);
        return percentComplete * (maxOut - minOut) + minOut;
    }

    public static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        }
        return value;
    }

    public static boolean almostEquals(double a, double b) {
        return almostEquals(a, b, 1e-6);
    }

    public static boolean almostEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }
}
