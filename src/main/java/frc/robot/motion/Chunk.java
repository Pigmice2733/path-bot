package frc.robot.motion;

import java.lang.Math;

public class Chunk {
    final private double distance, startVelocity, endVelocity, duration;

    private Chunk(double distance, double startVelocity, double endVelocity, double duration) {
        this.distance = distance;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.duration = duration;
    }

    public static Chunk createConstantVelocity(double velocity, double distance) {
        double duration = distance / velocity;
        return new Chunk(distance, velocity, velocity, duration);
    }

    public static Chunk createVelocityTransition(double startVelocity, double endVelocity, double maxAccel,
            double maxDecel) {
        final double averageVelocity = (startVelocity + endVelocity) / 2;
        final double deltaVelocity = (endVelocity - startVelocity);

        double duration, distance;

        if (Math.abs(endVelocity) > Math.abs(startVelocity)) {
            duration = Math.abs(deltaVelocity) / maxAccel;
        } else {
            duration = Math.abs(deltaVelocity) / maxDecel;
        }

        distance = averageVelocity * duration;

        return new Chunk(distance, startVelocity, endVelocity, duration);
    }

    public double getDuration() {
        return duration;
    }

    public double getTotalDistance() {
        return distance;
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public double getPosition(double time) {
        final double currentVelocity = getVelocity(time);
        final double averageVelocity = (startVelocity + currentVelocity) / 2;
        return averageVelocity * time;
    }

    public double getVelocity(double time) {
        final double slope = (endVelocity - startVelocity) / duration;
        return startVelocity + slope * time;
    }
}
