package frc.robot.motion;

public class Setpoint {
    private final double position, velocity, acceleration;

    protected Setpoint(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    protected Setpoint(Chunk chunk, double time, double previousDistance) {
        acceleration = chunk.getAcceleration();
        velocity = chunk.getVelocity(time);
        position = chunk.getPosition(time) + previousDistance;
    }

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }
}