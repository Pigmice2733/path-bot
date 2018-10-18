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
public class QuinticHermite {
    ArrayList<Double> knots;
    ArrayList<QuinticHermiteSegment> segments;

    /**
     * Constructs a quintic Hermite spline from a set of knots, points, derivatives,
     * and second derivatives.
     * 
     * @param knots             The knots to use for the spline
     * @param points            The point to interpolate at the knots
     * @param derivatives       The derivaties to interpoate at the knots
     * @param secondDerivatives The second derivatives to interpoate at the knots
     */
    QuinticHermite(ArrayList<Double> knots, ArrayList<Point> points, ArrayList<Vector> derivatives,
            ArrayList<Vector> secondDerivatives) {
        this.knots = knots;
        this.segments = new ArrayList<>();

        for (int i = 0; i < points.size() - 1; i += 1) {
            double knotLength = knots.get(i + 1) - knots.get(i);
            Vector startDerivative = derivatives.get(i).scale(knotLength);
            Vector endDerivative = derivatives.get(i + 1).scale(knotLength);
            Vector startSecondDerivative = secondDerivatives.get(i).scale(knotLength);
            Vector endSecondDerivative = secondDerivatives.get(i + 1).scale(knotLength);

            QuinticHermiteSegment segment = new QuinticHermiteSegment(points.get(i), points.get(i + 1), startDerivative,
                    endDerivative, startSecondDerivative, endSecondDerivative);
            segments.add(segment);
        }
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
    public Point getPoint(double t) {
        int i = getSegmentIndex(t);
        // Transform from global parameter to local
        double s = globalToLocal(i, t);
        return segments.get(i).position(s);
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
        return segments.get(i).derivative(s).scale(1.0 / knotLength);
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
        return segments.get(i).secondDerivative(s).scale(1.0 / knotLength);
    }

    /**
     * Gets the third derivative of this spline at the specified global paramter
     * value.
     * 
     * @param t The value of the global parameter to get the third derivative at
     * @return The third derivative at the parameter value
     */
    public Vector getThirdDerivative(double t) {
        int i = getSegmentIndex(t);
        // Transform from global parameter to local
        double s = globalToLocal(i, t);
        double knotLength = knots.get(i + 1) - knots.get(i);
        // Get third derivative with repsect to local parameter and scale so it is with
        // respect to the global parameter
        return segments.get(i).thirdDerivative(s).scale(1.0 / knotLength);
    }

    /**
     * Gets the arc length of this spline. This is the same as the maximum
     * parameter value opf this spline.
     * 
     * @return The full arc length of this spline
     */
    public double getLength() {
        return knots.get(knots.size() - 1);
    }

    /**
     * Gets the number of segment start/end points this spline has, including the
     * very start and end.
     * 
     * @return
     */
    int getNumberOfPoints() {
        return segments.size() + 1;
    }

    /**
     * Reparameterizes this spline with the new knot values. Scales all derivatives
     * and second derivatives to match the new knots.
     * 
     * @param newKnots The new knot values to use. This must have the same number of
     *                 knots as the spline currently has, which is the same as the
     *                 number of points it has.
     */
    void reparameterize(ArrayList<Double> newKnots) {
        // Derivaitives/second derivatives of each segment are with respect to the local
        // parameter value, which is dependent on the spacing of the knots. When the
        // spline is reparameterized, the derivatives/second erivatives need to be
        // scaled to match the new knots.
        for (int i = 0; i < segments.size(); i++) {
            double knotRatio = (newKnots.get(i + 1) - newKnots.get(i)) / (knots.get(i + 1) - knots.get(i));
            QuinticHermiteSegment segment = segments.get(i);
            segment.startDerivative = segment.startDerivative.scale(knotRatio);
            segment.endDerivative = segment.endDerivative.scale(knotRatio);
            segment.startSecondDerivative = segment.startSecondDerivative.scale(knotRatio);
            segment.endSecondDerivative = segment.endSecondDerivative.scale(knotRatio);
        }
        knots = newKnots;
    }

    /**
     * Get the arc length of a spline segment.
     * 
     * @param segmentIndex Index of the start point of the segment. A
     *                     <code>segmentIndex</code> of <code>0</code> refers to the
     *                     first segment, etc.
     * @return Arc length of the segment
     */
    double segmentArcLength(int segmentIndex) {
        return segments.get(segmentIndex).arcLength();
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
