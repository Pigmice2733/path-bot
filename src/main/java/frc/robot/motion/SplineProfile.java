package frc.robot.motion;

import java.util.ArrayList;

import frc.robot.spline.QuinticSpline;
import frc.robot.spline.QuinticSpline.SplineChunks;
import frc.robot.utils.Bounds;
import frc.robot.utils.Utils;

public class SplineProfile {
    public ArrayList<Double> velocities;
    public ArrayList<Double> times;
    public ArrayList<Double> curvatures;
    public ArrayList<Double> headings;
    private double trackWidth;
    private double maxWheelVelocity;
    private double chunkLength;

    /**
     * Constructs a SplineProfile to efficiently guide a robot along a QuinticSpline
     * path.
     * 
     * @param spline           QuinticSpline path
     * @param chunkLength      Distance along path to make each chunk - smaller
     *                         means higher resolution
     * @param maxWheelVelocity Maximum velocity of the drivetrain in a straight line
     * @param maxAcceleration  Robot's maximum acceleration/deceleration
     * @param trackWidth       Track width of the robot's drivetrain
     */
    public SplineProfile(QuinticSpline spline, double chunkLength, double maxWheelVelocity, double maxAcceleration,
            double trackWidth) {
        SplineChunks chunks = spline.computeSplineChunks(chunkLength);
        curvatures = chunks.getCurvatureChunks();
        headings = chunks.getHeadingChunks();

        velocities = new ArrayList<>();
        times = new ArrayList<>();

        this.trackWidth = trackWidth;
        this.maxWheelVelocity = maxWheelVelocity;
        this.chunkLength = chunkLength;

        // Spline profile starts with velocity 0.0
        velocities.add(0.0);

        // Iterate over list forwards and calculate maximum velocities possible based on
        // path curvature, and the robot's acceleration limit.
        for (int i = 1; i < curvatures.size(); i++) {
            double initialVelocity = velocities.get(i - 1);
            double curvatureVelocityLimit = maxVelocityFromCurvature(curvatures.get(i));
            // The maximum velocity is the minimum of the maximum velocity possible based on
            // the path curvature, and the maximum velocity attainable if the robot
            // accelerated across the entire previous chunk.
            double velocityChangeLimit = Math
                    .sqrt(initialVelocity * initialVelocity + 2 * maxAcceleration * chunkLength);
            double maxVelocity = Math.min(curvatureVelocityLimit, velocityChangeLimit);
            velocities.add(maxVelocity);
        }

        // Spline profile end with velocity 0.0
        velocities.add(0.0);

        // Iterate over list backwards and calculate maximum velocities possible based
        // on the robot's acceleration limit, and the limits calculated during the
        // forward pass.
        for (int i = curvatures.size() - 1; i >= 0; i--) {
            // The maximum velocity is the minimum of the maximum velocity calcuated the
            // first time through, and the maximum velocity the robot can have and still
            // have enoguh time to decelerate to stay within the velocity limit of the next
            // chunk.
            double initialVelocity = velocities.get(i + 1);
            double maxVelocity = Math.min(velocities.get(i),
                    Math.sqrt(initialVelocity * initialVelocity + 2 * maxAcceleration * chunkLength));
            velocities.set(i, maxVelocity);
        }

        // Time starts at 0.0
        double time = 0.0;
        times.add(time);
        for (int i = 0; i < velocities.size() - 1; i++) {
            double averageVelocity = (velocities.get(i + 1) + velocities.get(i)) / 2.0;
            time += chunkLength / averageVelocity;
            times.add(time);
        }

        curvatures.add(0, curvatures.get(0));
        headings.add(0, headings.get(0));
    }

    /**
     * Gets a Setpoint for this profile at a specific time.
     * 
     * @param time Time to get the setpoint for, will be clamped within bounds of
     *             profile
     * @return The Setpoint describing the profile at the specified moment in time
     */
    public Setpoint getSetpointAtTime(double time) {
        // Time should be within the bounds of the profile. If it is past the end, the
        // setpoint from the end of the profile should be returned.
        time = new Bounds(0.0, times.get(times.size() - 1)).clamp(time);

        int index = Utils.binarySearch(times, time);

        Chunk chunk = Chunk.createVelocityDistance(chunkLength, velocities.get(index), velocities.get(index + 1),
                curvatures.get(index), curvatures.get(+1), 0.0, 0.0);
        Setpoint sp = new Setpoint(chunk, time - times.get(index), index * chunkLength, curvatures.get(index),
                headings.get(index));
        return sp;
    }

    /**
     * Gets the time length of the profile.
     * 
     * @return Time at the end of the profile
     */
    public double getLength() {
        return times.get(times.size() - 1);
    }

    // The maximum velocity the robot can go is limited by the maximum velocity the
    // outer wheel can go along the path.
    private double maxVelocityFromCurvature(double curvature) {
        if (curvature < 1e-4) {
            return maxWheelVelocity;
        }

        double radius = 1.0 / curvature;
        double outerTrackRadius = radius + 0.5 * trackWidth;

        return maxWheelVelocity * (radius / outerTrackRadius);
    }
}