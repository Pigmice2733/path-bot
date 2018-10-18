package frc.robot.spline;

import frc.robot.utils.Point;
import frc.robot.utils.Vector;

/**
 * A single segment of a quintic Hermite spline.
 */
class QuinticHermiteSegment {
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
    QuinticHermiteSegment(Point start, Point end, Vector startDerivative, Vector endDerivative,
            Vector startSecondDerivative, Vector endSecondDerivative) {
        this.start = start;
        this.end = end;
        this.startDerivative = startDerivative;
        this.endDerivative = endDerivative;
        this.startSecondDerivative = startSecondDerivative;
        this.endSecondDerivative = endSecondDerivative;
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
    Point getWheel(double s, double wheelX, double wheelY) {
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
    double arcLength() {
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
    Point position(double s) {
        // Quintic Hermite spline derivative coefficients
        double pointStartCoef = 1 - 10 * Math.pow(s, 3) + 15 * Math.pow(s, 4) - 6 * Math.pow(s, 5);
        double pointEndCoef = 10 * Math.pow(s, 3) - 15 * Math.pow(s, 4) + 6 * Math.pow(s, 5);
        double derivativeStartCoef = s - 6 * Math.pow(s, 3) + 8 * Math.pow(s, 4) - 3 * Math.pow(s, 5);
        double derivativeEndCoef = -4 * Math.pow(s, 3) + 7 * Math.pow(s, 4) - 3 * Math.pow(s, 5);
        double secondDerivativeStartCoef = 0.5 * Math.pow(s, 2) - 1.5 * Math.pow(s, 3) + 1.5 * Math.pow(s, 4)
                - 0.5 * Math.pow(s, 5);
        double secondDerivativeEndCoef = 0.5 * Math.pow(s, 3) - Math.pow(s, 4) + 0.5 * Math.pow(s, 5);

        double x = pointStartCoef * start.getX() + pointEndCoef * end.getX()
                + derivativeStartCoef * startDerivative.getX() + derivativeEndCoef * endDerivative.getX()
                + secondDerivativeStartCoef * startSecondDerivative.getX()
                + secondDerivativeEndCoef * endSecondDerivative.getX();
        double y = pointStartCoef * start.getY() + pointEndCoef * end.getY()
                + derivativeStartCoef * startDerivative.getY() + derivativeEndCoef * endDerivative.getY()
                + secondDerivativeStartCoef * startSecondDerivative.getY()
                + secondDerivativeEndCoef * endSecondDerivative.getY();

        return new Point(x, y);
    }

    /**
     * Calculates the derivative of this segment at the specified local parameter
     * variable value.
     * 
     * @param s The parameter value to find the derivative at
     * @return The derivative as a vector
     */
    Vector derivative(double s) {
        // Quintic Hermite spline derivative coefficients
        double pointCoef = -30 * Math.pow(s, 2) + 60 * Math.pow(s, 3) - 30 * Math.pow(s, 4);
        double derivativeStartCoef = 1 - 18 * Math.pow(s, 2) + 32 * Math.pow(s, 3) - 15 * Math.pow(s, 4);
        double derivativeEndCoef = -12 * Math.pow(s, 2) + 28 * Math.pow(s, 3) - 15 * Math.pow(s, 4);
        double secondDerivativeStartCoef = s - 4.5 * Math.pow(s, 2) + 6 * Math.pow(s, 3) - 2.5 * Math.pow(s, 4);
        double secondDerivativeEndCoef = 1.5 * Math.pow(s, 2) - 4 * Math.pow(s, 3) + 2.5 * Math.pow(s, 4);

        double dx = pointCoef * start.getX() + (-pointCoef) * end.getX() + derivativeStartCoef * startDerivative.getX()
                + derivativeEndCoef * endDerivative.getX() + secondDerivativeStartCoef * startSecondDerivative.getX()
                + secondDerivativeEndCoef * endSecondDerivative.getX();
        double dy = pointCoef * start.getY() + (-pointCoef) * end.getY() + derivativeStartCoef * startDerivative.getY()
                + derivativeEndCoef * endDerivative.getY() + secondDerivativeStartCoef * startSecondDerivative.getY()
                + secondDerivativeEndCoef * endSecondDerivative.getY();

        return new Vector(dx, dy);
    }

    /**
     * Calculates the second derivative of this segment at the specified local
     * parameter variable value.
     * 
     * @param s The parameter value to find the second derivative at
     * @return The second derivative as a vector
     */
    Vector secondDerivative(double s) {
        // Quintic Hermite spline derivative coefficients
        double pointCoef = -60 * s + 180 * Math.pow(s, 2) - 120 * Math.pow(s, 3);
        double derivativeStartCoef = -36 * s + 96 * Math.pow(s, 2) - 60 * Math.pow(s, 3);
        double derivativeEndCoef = -24 * s + 84 * Math.pow(s, 2) - 60 * Math.pow(s, 3);
        double secondDerivativeStartCoef = 1 - 9 * s + 18 * Math.pow(s, 2) - 10 * Math.pow(s, 3);
        double secondDerivativeEndCoef = 3 * s + 12 * Math.pow(s, 2) + 10 * Math.pow(s, 3);

        double ddx = pointCoef * start.getX() + (-pointCoef) * end.getX() + derivativeStartCoef * startDerivative.getX()
                + derivativeEndCoef * endDerivative.getX() + secondDerivativeStartCoef * startSecondDerivative.getX()
                + secondDerivativeEndCoef * endSecondDerivative.getX();
        double ddy = pointCoef * start.getY() + (-pointCoef) * end.getY() + derivativeStartCoef * startDerivative.getY()
                + derivativeEndCoef * endDerivative.getY() + secondDerivativeStartCoef * startSecondDerivative.getY()
                + secondDerivativeEndCoef * endSecondDerivative.getY();

        return new Vector(ddx, ddy);
    }

    /**
     * Calculates the third derivative of this segment at the specified local
     * parameter variable value.
     * 
     * @param s The parameter value to find the third derivative at
     * @return The third derivative as a vector
     */
    Vector thirdDerivative(double s) {
        // Quintic Hermite spline derivative coefficients
        double pointCoef = -60 + 360 * s - 360 * Math.pow(s, 2);
        double derivativeStartCoef = -36 + 192 * s - 180 * Math.pow(s, 2);
        double derivativeEndCoef = -24 + 168 * s - 180 * Math.pow(s, 2);
        double secondDerivativeStartCoef = -9 + 36 * s - 30 * Math.pow(s, 2);
        double secondDerivativeEndCoef = 3 + 24 * s + 30 * Math.pow(s, 2);

        double dddx = pointCoef * start.getX() + (-pointCoef) * end.getX()
                + derivativeStartCoef * startDerivative.getX() + derivativeEndCoef * endDerivative.getX()
                + secondDerivativeStartCoef * startSecondDerivative.getX()
                + secondDerivativeEndCoef * endSecondDerivative.getX();
        double dddy = pointCoef * start.getY() + (-pointCoef) * end.getY()
                + derivativeStartCoef * startDerivative.getY() + derivativeEndCoef * endDerivative.getY()
                + secondDerivativeStartCoef * startSecondDerivative.getY()
                + secondDerivativeEndCoef * endSecondDerivative.getY();

        return new Vector(dddx, dddy);
    }

    /**
     * Calculates the unsigned curvature of this segment at the specified local
     * parameter variable value.
     * 
     * @param s The value of the local parameter variable to find the curvature at
     * @return The unsigned curvature
     */
    double curvature(double s) {
        Vector first = derivative(s);
        Vector second = secondDerivative(s);

        double dividend = Math.abs(first.getX() * second.getY() - first.getY() * second.getX());
        double divisor = Math.pow(first.getX() * first.getX() + first.getY() * first.getY(), 1.5);

        return dividend / divisor;
    }
}
