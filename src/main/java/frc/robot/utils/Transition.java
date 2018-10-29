package frc.robot.utils;

public class Transition {
    private double start;
    private double rate;
    private Bounds bounds;

    /**
     * Creates a transition.
     * 
     * @param start    Start value for the transition
     * @param end      End value for the transition
     * @param duration Duration of the transition
     */
    public Transition(double start, double end, double duration) {
        this.start = start;
        this.rate = (end - start) / duration;
        bounds = new Bounds(0.0, duration);
    }

    /**
     * Creates a 0.0 to 0.0 transition with duration 1.0.
     * 
     * @return A zero transition
     */
    public static Transition zero() {
        return new Transition(0.0, 0.0, 1.0);
    }

    /**
     * Gets the value at the specified time into the transition.
     * 
     * @param time How far into the transition to get the value at
     * @return The value of the transition
     */
    public double get(double time) {
        time = bounds.clamp(time);
        return start + rate * time;
    }

    /**
     * Integrates the value of the transition from the beginning to the specified
     * time.
     * 
     * @param time How far into the transition to integrate to
     * @return The integral of the transition
     */
    public double integrate(double time) {
        time = bounds.clamp(time);
        return 0.5 * (start + get(time)) * time;
    }

    /**
     * Gets the rate of the transition.
     * 
     * @return Transition rate
     */
    public double getRate() {
        return rate;
    }
}