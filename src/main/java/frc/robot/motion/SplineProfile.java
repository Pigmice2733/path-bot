package frc.robot.motion;

import java.util.ArrayList;

import frc.robot.spline.QuinticSpline;
import frc.robot.utils.Bounds;
import frc.robot.utils.Utils;

public class SplineProfile {
    private ArrayList<Double> velocities;
    private ArrayList<Double> times;
    private double trackWidth;
    private double maxWheelVelocity;
    private double chunkLength;

    public SplineProfile(QuinticSpline spline, double chunkLength, double maxWheelVelocity, double maxAcceleration,
            double trackWidth) {
        ArrayList<Double> curvatureChunks = spline.computeCurvatureChunks(chunkLength);

        velocities = new ArrayList<>();
        times = new ArrayList<>();

        this.trackWidth = trackWidth;
        this.maxWheelVelocity = maxWheelVelocity;
        this.chunkLength = chunkLength;

        // Spline profile starts with velocity 0.0
        velocities.add(0.0);

        // Iterate over list forwards and calculate maximum velocities possible based on
        // path curvature, and the robot's acceleration limit.
        for (int i = 1; i < curvatureChunks.size(); i++) {
            double maxVelocity = maxVelocityFromCurvature(curvatureChunks.get(i));
            // The maximum velocity is the minimum of the maximum velocity possible based on
            // the path curvature, and the maximum velocity attainable if the robot
            // accelerated across the entire previous chunk.
            maxVelocity = Math.min(maxVelocity, velocities.get(i - 1) + Math.sqrt(chunkLength * maxAcceleration));
            velocities.set(i, maxVelocity);
        }

        // Spline profile end with velocity 0.0
        velocities.add(0.0);

        // Iterate over list backwards and calculate maximum velocities possible based
        // on the robot's acceleration limit, and the limits calculated during the
        // forward pass.
        for (int i = curvatureChunks.size() - 2; i >= 0; i--) {
            // The maximum velocity is the minimum of the maximum velocity calcuated the
            // first time through, and the maximum velocity the robot can have and still
            // have enoguh time to decelerate to stay within the velocity limit of the next
            // chunk.
            double maxVelocity = Math.min(velocities.get(i),
                    velocities.get(i + 1) + Math.sqrt(chunkLength * maxAcceleration));
            velocities.set(i, maxVelocity);
        }

        // Time starts at 0.0
        times.add(0.0);
        for (int i = 0; i < velocities.size() - 1; i++) {
            double averageVelocity = (velocities.get(i + 1) + velocities.get(i)) / 2.0;
            times.add(chunkLength / averageVelocity);
        }
    }

    public Setpoint getSetpointAtTime(double time) {
        // Time should be within the bounds of the profile. If it is past the end, the
        // setpoint from the end of the profile should be returned.
        time = new Bounds(0.0, times.get(times.size() - 1)).clamp(time);

        int index = Utils.binarySearch(times, time);

        Chunk chunk = Chunk.createVelocityDistance(chunkLength, velocities.get(index), velocities.get(index + 1));
        return new Setpoint(chunk, time - times.get(index), index * chunkLength);
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