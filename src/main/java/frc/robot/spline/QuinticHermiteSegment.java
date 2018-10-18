package frc.robot.spline;

import java.util.ArrayList;

import frc.robot.utils.Point;
import frc.robot.utils.Vector;

/**
 * A single segment of a quintic Hermite spline.
 */
class QuinticHermiteSegment {
    ArrayList<Double> xCoefs;
    ArrayList<Double> yCoefs;

    Point start, end;
    Vector startDerivative, endDerivative;
    Vector startSecondDerivative, endSecondDerivative;

    /**
     * Constructs a quintic Hermite spline segment.
     * 
     * @param start                 The point to start the point at
     * @param end                   The point to end the segment at
     * @param startDerivative       The first derivative at the start of the segment
     * @param endDerivative         The second derivative at the start of the
     *                              segment
     * @param startSecondDerivative The second derivative at the start of the
     *                              segment
     * @param endSecondDerivative   The second derivative at the end of the segment
     */
    protected QuinticHermiteSegment(Point start, Point end, Vector startDerivative, Vector endDerivative,
            Vector startSecondDerivative, Vector endSecondDerivative) {

        this.start = start;
        this.end = end;
        this.startDerivative = startDerivative;
        this.endDerivative = endDerivative;
        this.startSecondDerivative = startSecondDerivative;
        this.endSecondDerivative = endSecondDerivative;

        xCoefs = new ArrayList<>();
        yCoefs = new ArrayList<>();

        calculateCoefficients();
    }

    /**
     * Calculates coefficients for spline polynomials.
     */
    private void calculateCoefficients() {
        xCoefs.add(start.getX());
        xCoefs.add(startDerivative.getX());
        xCoefs.add(0.5 * startSecondDerivative.getX());
        xCoefs.add(-10 * start.getX() + 10 * end.getX() - 6 * startDerivative.getX() - 4 * endDerivative.getX()
                - 1.5 * startSecondDerivative.getX() + 0.5 * endSecondDerivative.getX());
        xCoefs.add(15 * start.getX() - 15 * end.getX() + 8 * startDerivative.getX() + 7 * endDerivative.getX()
                + 1.5 * startSecondDerivative.getX() - endSecondDerivative.getX());
        xCoefs.add(-6 * start.getX() + 6 * end.getX() - 3 * startDerivative.getX() - 3 * endDerivative.getX()
                - 0.5 * startSecondDerivative.getX() + 0.5 * endSecondDerivative.getX());

        yCoefs.add(start.getY());
        yCoefs.add(startDerivative.getY());
        yCoefs.add(0.5 * startSecondDerivative.getY());
        yCoefs.add(-10 * start.getY() + 10 * end.getY() - 6 * startDerivative.getY() - 4 * endDerivative.getY()
                - 1.5 * startSecondDerivative.getY() + 0.5 * endSecondDerivative.getY());
        yCoefs.add(15 * start.getY() - 15 * end.getY() + 8 * startDerivative.getY() + 7 * endDerivative.getY()
                + 1.5 * startSecondDerivative.getY() - endSecondDerivative.getY());
        yCoefs.add(-6 * start.getY() + 6 * end.getY() - 3 * startDerivative.getY() - 3 * endDerivative.getY()
                - 0.5 * startSecondDerivative.getY() + 0.5 * endSecondDerivative.getY());
    }

    /**
     * Calculates the position of a robot's wheel at the specified local parameter
     * variable value as it drives along this segment.
     * 
     * @param s      The local paramter value to find the wheel position at
     * @param wheelX The x offset of the wheel from the center of the robot
     * @param wheelY The y offset of the wheel from the center of the robot
     * @return The point representing where the robot's wheel would be if it
     *         followed the spline perfectly
     */
    protected Point getWheel(double s, double wheelX, double wheelY) {
        Point robotCenter = position(s);
        Vector derivative = derivative(s);

        // Wheel offsets are for when robot is turned to PI/2 so the rotation needs to
        // take that into account.
        double rotation = derivative.getAngle() - 0.5 * Math.PI;

        Vector wheelOffset = new Vector(wheelX, wheelY).rotate(rotation);
        return robotCenter.translate(wheelOffset);
    }

    /**
     * Calculates the arc
     * length of this spline segment using numerical integration.
     * 
     * @return Arc length of this segment
     */
    protected double arcLength() {
        double chordLength = end.subtract(start).getMagnitude();
        int iterations = (int) (100.0 * chordLength);

        double arcLength = 0.0;
        double stepSize = 1.0 / iterations;

        for (int i = 0; i <= iterations; i++) {
            double s = i / iterations;
            arcLength += stepSize * derivative(s).getMagnitude();
        }

        return arcLength;
    }

    /**
     * Calculates the position of this segment at the specified local parameter
     * variable value.
     * 
     * @param s The parameter value to find the position at
     * @return The point on the spline at the specified parameter value
     */
    protected Point position(double s) {
        double x = 0.0;
        double y = 0.0;
        for (int exponent = 0; exponent < xCoefs.size(); exponent++) {
            double power = Math.pow(s, exponent);
            x += xCoefs.get(exponent) * power;
            y += yCoefs.get(exponent) * power;
        }

        return new Point(x, y);
    }

    /**
     * Calculates the derivative of this segment at the specified local parameter
     * variable value.
     * 
     * @param s The parameter value to find the derivative at
     * @return The derivative as a vector
     */
    protected Vector derivative(double s) {
        double x = 0.0;
        double y = 0.0;
        for (int exponent = 1; exponent < xCoefs.size(); exponent++) {
            double power = exponent * Math.pow(s, exponent - 1);
            x += xCoefs.get(exponent) * power;
            y += yCoefs.get(exponent) * power;
        }

        return new Vector(x, y);
    }

    /**
     * Calculates the second derivative of this segment at the specified local
     * parameter variable value.
     * 
     * @param s The parameter value to find the second derivative at
     * @return The second derivative as a vector
     */
    protected Vector secondDerivative(double s) {
        double x = 0.0;
        double y = 0.0;
        for (int exponent = 2; exponent < xCoefs.size(); exponent++) {
            double power = exponent * (exponent - 1) * Math.pow(s, exponent - 2);
            x += xCoefs.get(exponent) * power;
            y += yCoefs.get(exponent) * power;
        }

        return new Vector(x, y);
    }

    /**
     * Calculates the unsigned curvature of this segment at the specified local
     * parameter variable value.
     * 
     * @param s The value of the local parameter variable to find the curvature at
     * @return The unsigned curvature
     */
    protected double curvature(double s) {
        Vector first = derivative(s);
        Vector second = secondDerivative(s);

        double dividend = Math.abs(first.getX() * second.getY() - first.getY() * second.getX());
        double divisor = Math.pow(first.getX() * first.getX() + first.getY() * first.getY(), 1.5);

        return dividend / divisor;
    }
}
