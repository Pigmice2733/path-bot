package frc.robot.spline;

import java.util.ArrayList;

import frc.robot.utils.Point;
import frc.robot.utils.Vector;

/**
 * A full quintic Hermite spline. The spline is stored as a list of individual
 * segments, each of which is in Hermite basis form. This makes it easy to
 * manipulkate and modify the spline but slower to compute points and
 * derivatives along the spline.
 */
public class QuinticSpline {
    private ArrayList<Double> knots;
    private ArrayList<QuinticSplineSegment> segments;

    /**
     * Stores the curvature and heading (in radians) at the end of each of a series
     * of chunks along a spline segment, as well as how much arc length was left
     * over at the end of a segment.
     */
    public static class SplineChunks {
        private ArrayList<Double> curvatureChunks;
        private ArrayList<Double> headingChunks;
        private double remainingArcLength;

        /**
         * Creates a new SplineChunks given the heading and curvature chunks, and the
         * remaining arc length from the segment that was not included as one of the
         * chunks.
         * 
         * @param curvatureChunks    An array of signed curvature values at the end of
         *                           each chunk
         * @param headingChunks      An array of heading values in radians at the end of
         *                           each chunk
         * @param remainingArcLength The amount of remaining arc length that was not
         *                           counted as part of a chunk
         */
        protected SplineChunks(ArrayList<Double> curvatureChunks, ArrayList<Double> headingChunks,
                double remainingArcLength) {
            this.curvatureChunks = curvatureChunks;
            this.headingChunks = headingChunks;
            this.remainingArcLength = remainingArcLength;
        }

        /**
         * Gets the curvature chunks.
         * 
         * @return ArrayList of the curvatures
         */
        public ArrayList<Double> getCurvatureChunks() {
            return curvatureChunks;
        }

        /**
         * Gets the heading chunks.
         * 
         * @return ArrayList of the headings, in radians
         */
        public ArrayList<Double> getHeadingChunks() {
            return headingChunks;
        }
    }

    /**
     * Constructs a quintic Hermite spline from a set of knots, points, derivatives,
     * and second derivatives.
     *
     * @param knots             The knots to use for the spline
     * @param points            The point to interpolate at the knots
     * @param derivatives       The derivaties to interpoate at the knots
     * @param secondDerivatives The second derivatives to interpoate at the knots
     */
    public QuinticSpline(ArrayList<Double> knots, ArrayList<Point> points, ArrayList<Vector> derivatives,
            ArrayList<Vector> secondDerivatives) {
        this.knots = knots;
        this.segments = new ArrayList<>();

        for (int i = 0; i < points.size() - 1; i += 1) {
            double knotLength = knots.get(i + 1) - knots.get(i);
            Vector startDerivative = derivatives.get(i).scale(knotLength);
            Vector endDerivative = derivatives.get(i + 1).scale(knotLength);
            Vector startSecondDerivative = secondDerivatives.get(i).scale(knotLength);
            Vector endSecondDerivative = secondDerivatives.get(i + 1).scale(knotLength);

            QuinticSplineSegment segment = new QuinticSplineSegment(points.get(i), points.get(i + 1), startDerivative,
                    endDerivative, startSecondDerivative, endSecondDerivative);
            segments.add(segment);
        }
    }

    /**
     * Gets all the control points of the spline.
     * 
     * @return An ArrayList holding the control points
     */
    public ArrayList<Point> getControlPoints() {
        ArrayList<Point> controlPoints = new ArrayList<>();
        for (QuinticSplineSegment segment : segments) {
            controlPoints.add(segment.start);
        }
        controlPoints.add(segments.get(segments.size() - 1).end);
        return controlPoints;
    }

    /**
     * Calculates the position of a robot's wheel at the specified global parameter
     * value as it drives along this spline.
     *
     * @param t      The global paramter value to find the wheel position at
     * @param wheelX The x offset of the wheel from the center of the robot
     * @param wheelY The y offset of the wheel from the center of the robot
     * @return The point representing where the robot's wheel would be if it
     *         followed the spline perfectly
     */
    public Point getWheel(double t, double wheelX, double wheelY) {
        int i = getSegmentIndex(t);
        // Transform from global parameter to local
        double s = globalToLocal(i, t);
        return segments.get(i).getWheel(s, wheelX, wheelY);
    }

    /**
     * Gets the point on this spline at the specified global paramter value.
     *
     * @param t The value of the global parameter to get the point at
     * @return The point at the parameter value
     */
    public Point getPosition(double t) {
        int i = getSegmentIndex(t);
        // Transform from global parameter to local
        double s = globalToLocal(i, t);
        return segments.get(i).getPosition(s);
    }

    /**
     * Gets the derivative of this spline at the specified global paramter value.
     *
     * @param t The value of the global parameter to get the point at
     * @return The derivative at the parameter value
     */
    public Vector getDerivative(double t) {
        int i = getSegmentIndex(t);
        // Transform from global parameter to local
        double s = globalToLocal(i, t);
        double knotLength = knots.get(i + 1) - knots.get(i);
        // Get derivative with repsect to local parameter and scale so it is with
        // respect to the global parameter
        return segments.get(i).getDerivative(s).scale(1.0 / knotLength);
    }

    /**
     * Gets the second derivative of this spline at the specified global paramter
     * value.
     *
     * @param t The value of the global parameter to get the second derivative at
     * @return The second derivative at the parameter value
     */
    public Vector getSecondDerivative(double t) {
        int i = getSegmentIndex(t);
        // Transform from global parameter to local
        double s = globalToLocal(i, t);
        double knotLength = knots.get(i + 1) - knots.get(i);
        // Get second derivative with repsect to local parameter and scale so it is with
        // respect to the global parameter
        return segments.get(i).getSecondDerivative(s).scale(1.0 / knotLength);
    }

    /**
     * Gets the point on this spline at the specified global paramter value.
     *
     * @param t The value of the global parameter to get the point at
     * @return The point at the parameter value
     */
    public double getCurvature(double t) {
        int i = getSegmentIndex(t);
        // Transform from global parameter to local
        double s = globalToLocal(i, t);
        return segments.get(i).getCurvature(s);
    }

    /**
     * Gets the knot length of this spline. This is the maximum parameter value of
     * this spline.
     *
     * @return The last knot of this spline
     */
    public double getLength() {
        return knots.get(knots.size() - 1);
    }

    /**
     * Computes the heading and curvature at the boundaries a series of chunks of
     * uniform length
     * along the spline. Knowing the curvature of the spline at a series of points
     * along it allows the creation of motion profiles to efficiently follow to
     * spline.
     * 
     * @param chunkLength The length each chunk should be
     * @return An array of the curvatures at the end of each chunk
     */
    public SplineChunks computeSplineChunks(double chunkLength) {
        ArrayList<Double> curvatureChunks = new ArrayList<>();
        ArrayList<Double> headingChunks = new ArrayList<>();

        curvatureChunks.add(segments.get(0).getCurvature(0.0));
        headingChunks.add(segments.get(0).getCurvature(0.0));

        double initialArcLength = 0.0;
        for (QuinticSplineSegment segment : segments) {
            SplineChunks chunks = segment.getSplineChunks(chunkLength, initialArcLength);
            initialArcLength = chunks.remainingArcLength;
            curvatureChunks.addAll(chunks.curvatureChunks);
            headingChunks.addAll(chunks.headingChunks);
        }

        return new SplineChunks(curvatureChunks, headingChunks, 0.0);
    }

    /**
     * Gets the number of segment start/end points this spline has, including the
     * very start and end.
     *
     * @return
     */
    private int getNumberOfPoints() {
        return segments.size() + 1;
    }

    /**
     * Finds the index of the segment that the global parameter value <code>t</code>
     * falls within.
     *
     * @param t Global parameter variable value to find the segment of
     * @return Index of the segment <cde>t</code> falls within
     */
    private int getSegmentIndex(double t) {
        int endIndex = getNumberOfPoints();
        for (int i = 1; i < endIndex; i++) {
            if (t < knots.get(i)) {
                return i - 1;
            }
        }
        return endIndex - 2;
    }

    /**
     * Convert from global parameter variable <code>t</code> to local parameter
     * <code>s</code>.
     *
     * @param segmentIndex Index of the start point of the segment. A
     *                     <code>segmentIndex</code> of <code>0</code> refers to the
     *                     first segment, etc.
     * @param t            Global parameter variable value to convert
     * @return The corresponding value of the local parameter <code>s</code>
     */
    private double globalToLocal(int segmentIndex, double t) {
        double startKnot = knots.get(segmentIndex);
        double endKnot = knots.get(segmentIndex + 1);
        return (t - startKnot) / (endKnot - startKnot);
    }
}
