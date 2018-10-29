package frc.robot.motion;

public class Setpoint {
    private final double position, velocity, acceleration;
    private final double curvature, heading;

    protected Setpoint(double position, double velocity, double acceleration, double curvature, double heading) {
        this.acceleration = acceleration;
        this.velocity = velocity;
        this.position = position;
        this.curvature = curvature;
        this.heading = heading;
    }

    protected Setpoint(Chunk chunk, double time, double previousDistance, double curvature, double heading) {
        acceleration = chunk.getAcceleration();
        velocity = chunk.getVelocity(time);
        position = chunk.getPosition(time) + previousDistance;
        this.curvature = curvature;
        this.heading = heading;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getPosition() {
        return position;
    }

    public double getCurvature() {
        return curvature;
    }

    public double getHeading() {
        return heading;
    }
}