package frc.robot;

public class Utils {

    public static double lerp(double val, double minIn, double maxIn, double minOut,
            double maxOut) {
        final double percentComplete = (val - minIn) / (maxIn - minIn);
        return percentComplete * (maxOut - minOut) + minOut;
    }
}
