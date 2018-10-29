package frc.robot.spline;

import java.util.ArrayList;

import frc.robot.utils.Point;
import frc.robot.utils.Vector;

/**
 * A single segment of a quintic Hermite spline.
 */
class QuinticSplineSegment {
    Point start, end;
    Vector startDerivative, endDerivative;
    Vector startSecondDerivative, endSecondDerivative;

    private ArrayList<Double> xCoefs;
    private ArrayList<Double> yCoefs;

    private double arcLength;

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
    protected QuinticSplineSegment(Point start, Point end, Vector startDerivative, Vector endDerivative,
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

        arcLength = calculateArcLength();
    }

    /**
     * Calculates coefficients for x and y quintic polynomials.
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
     * @param s           The local paramter value to find the wheel position at
     * @param wheelWidth  The width offset of the wheel from the center of the robot
     * @param wheelLength The length offset of the wheel from the center of the
     *                    robot
     * @return The point representing where the robot's wheel would be if it
     *         followed the spline perfectly
     */
    protected Point getWheel(double s, double wheelWidth, double wheelLength) {
        Point robotCenter = getPosition(s);
        Vector derivative = getDerivative(s);

        double rotation = derivative.getAngle();

        Vector wheelOffset = new Vector(wheelLength, -wheelWidth).rotate(rotation);
        return robotCenter.translate(wheelOffset);
    }

    /**
     * Gets the arc length of this spline segment.
     * 
     * @return Arc length of this segment
     */
    protected double getArcLength() {
        return arcLength;
    }

    /**
     * Calculates the arc length of this spline segment using numerical integration.
     * 
     * @return Arc length of this segment
     */
    private double calculateArcLength() {
        double chordLength = end.subtract(start).getMagnitude();
        int iterations = (int) (200.0 * chordLength);

        double arcLength = 0.0;
        Point previousPoint = start;

        for (int i = 0; i <= iterations; i++) {
            double s = (float) i / (float) iterations;
            Point currentPoint = getPosition(s);
            double delta = currentPoint.subtract(previousPoint).getMagnitude();
            previousPoint = currentPoint;
            arcLength += delta;
        }

        return arcLength;
    }

    /**
     * Computes the heading and curvature at the boundaries a series of chunks of
     * uniform length along the spline segment.
     * 
     * @param chunkLength      The length each chunk should be
     * @param initialArcLength Arc length value to starting measuring the chunks out
     *                         from at the start of the spline
     * @return An object containing an array of the curvatures, and an array of the
     *         headings (in radians) at the end of each chunk, and the extra arc
     *         length that was not counted as part of a full chunk
     */
    protected QuinticSpline.SplineChunks getSplineChunks(double chunkLength, double initialArcLength) {
        ArrayList<Double> curvatureSegments = new ArrayList<>();
        ArrayList<Double> headingSegments = new ArrayList<>();

        double currentArcLength = initialArcLength;
        double arcLengthOfPreviousChunk = 0.0;
        double stepSize = chunkLength / (100.0 * arcLength);
        double s = 0.0;

        while (s <= 1.0) {
            currentArcLength += stepSize * getDerivative(s).getMagnitude();
            if ((currentArcLength - arcLengthOfPreviousChunk) > chunkLength) {
                arcLengthOfPreviousChunk = currentArcLength;
                curvatureSegments.add(getCurvature(s));
                headingSegments.add(getHeading(s));
            }
            s += stepSize;
        }

        return new QuinticSpline.SplineChunks(curvatureSegments, headingSegments,
                currentArcLength - arcLengthOfPreviousChunk);
    }

    /**
     * Calculates the position of this segment at the specified local parameter
     * variable value.
     * 
     * @param s The parameter value to find the position at
     * @return The point on the spline at the specified parameter value
     */
    protected Point getPosition(double s) {
        double x = 0.0;
        double y = 0.0;
        for (int exponent = xCoefs.size() - 1; exponent >= 0; exponent--) {
            x = x * s + xCoefs.get(exponent);
            y = y * s + yCoefs.get(exponent);
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
    protected Vector getDerivative(double s) {
        double x = 0.0;
        double y = 0.0;
        for (int exponent = xCoefs.size() - 1; exponent >= 1; exponent--) {
            x = x * s + exponent * xCoefs.get(exponent);
            y = y * s + exponent * yCoefs.get(exponent);
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
    protected Vector getSecondDerivative(double s) {
        double x = 0.0;
        double y = 0.0;
        for (int exponent = xCoefs.size() - 1; exponent >= 2; exponent--) {
            x = x * s + exponent * (exponent - 1) * xCoefs.get(exponent);
            y = y * s + exponent * (exponent - 1) * yCoefs.get(exponent);
        }

        return new Vector(x, y);
    }

    /**
     * Calculates the signed curvature of this segment at the specified local
     * parameter variable value.
     * 
     * @param s The value of the local parameter variable to find the curvature at
     * @return The signed curvature
     */
    protected double getCurvature(double s) {
        Vector first = getDerivative(s);
        Vector second = getSecondDerivative(s);

        double dividend = first.getX() * second.getY() - first.getY() * second.getX();
        double divisor = Math.pow(first.getX() * first.getX() + first.getY() * first.getY(), 1.5);

        return dividend / divisor;
    }

    /**
     * Calculates the heading, in radians, of the robot at the specified local
     * parameter variable value.
     * 
     * @param s The value of the local parameter variable to find the curvature at
     * @return The heading in radians
     */
    protected double getHeading(double s) {
        return getDerivative(s).getAngle();
    }
}
