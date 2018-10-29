package frc.robot.utils;

import java.util.ArrayList;

public class Utils {
    public static double lerp(double val, double minIn, double maxIn, double minOut, double maxOut) {
        final double percentComplete = (val - minIn) / (maxIn - minIn);
        return percentComplete * (maxOut - minOut) + minOut;
    }

    public static boolean almostEquals(double a, double b) {
        return almostEquals(a, b, 1e-6);
    }

    public static boolean almostEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static int binarySearch(ArrayList<Double> data, double target) {
        int lowIndex = 0;
        int highIndex = data.size() - 1;

        return _binarySearch(data, target, lowIndex, highIndex);
    }

    private static int _binarySearch(ArrayList<Double> data, double target, int lowIndex, int highIndex) {
        if (highIndex - lowIndex == 1) {
            return lowIndex;
        }

        int mid = (lowIndex + highIndex) / 2;

        if (data.get(mid) < target) {
            return _binarySearch(data, target, mid, highIndex);
        } else {
            return _binarySearch(data, target, lowIndex, mid);
        }
    }
}
