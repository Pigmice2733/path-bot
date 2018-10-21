package frc.robot.motion;

import java.lang.Math;

class Chunk {
    final private double distance, startVelocity, endVelocity, acceleration, duration;

    private Chunk(double distance, double startVelocity, double endVelocity, double duration) {
        this.distance = distance;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.duration = duration;
        this.acceleration = (endVelocity - startVelocity) / duration;
    }

    protected static Chunk createVelocityDistance(double distance, double startVelocity, double endVelocity) {
        double averageVelocity = (startVelocity + endVelocity) / 2.0;
        double duration = distance / averageVelocity;
        return new Chunk(distance, startVelocity, endVelocity, duration);
    }

    protected static Chunk createConstantVelocity(double velocity, double distance) {
        double duration = distance / velocity;
        return new Chunk(distance, velocity, velocity, duration);
    }

    protected static Chunk createVelocityTransition(double startVelocity, double endVelocity, double maxAccel,
            double maxDecel) {
        final double averageVelocity = (startVelocity + endVelocity) / 2.0;
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

    protected Setpoint getSetpoint(double time) {
        return new Setpoint(getPosition(time), getVelocity(time), acceleration);
    }

    protected double getDuration() {
        return duration;
    }

    protected double getTotalDistance() {
        return distance;
    }

    protected double getEndVelocity() {
        return endVelocity;
    }

    protected double getPosition(double time) {
        final double currentVelocity = getVelocity(time);
        final double averageVelocity = (startVelocity + currentVelocity) / 2.0;
        return averageVelocity * time;
    }

    protected double getVelocity(double time) {
        return startVelocity + acceleration * time;
    }

    protected double getAcceleration() {
        return acceleration;
    }
}
