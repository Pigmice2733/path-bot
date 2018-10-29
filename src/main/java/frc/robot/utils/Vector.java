package frc.robot.utils;

/**
 * 2D vector data type
 */
public class Vector implements XY {
    private final double x, y;

    /**
     * Constructs a vector from x and y components.
     * 
     * @param x The x component of the vector
     * @param y The y component of the vector
     */
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the zero vector, which has no length or direction, each component is
     * zero.
     * 
     * @return The zero vector
     */
    public static Vector zero() {
        return new Vector(0, 0);
    }

    /**
     * Gets the x component of the vector.
     * 
     * @return The x component
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the y component of the vector.
     * 
     * @return y component
     */
    public double getY() {
        return y;
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) {
            return true;
        }
        if ((o == null) || (o.getClass() != this.getClass())) {
            return false;
        }
        Vector other = (Vector) o;
        return Utils.almostEquals(x, other.x) && Utils.almostEquals(y, other.y);
    }

    @Override
    public int hashCode() {
        return (int) (3.0 + 11.0 * x + 193.0 * y);
    }

    /**
     * Scales the vector by a scalar factor.
     * 
     * @param scale The factor to scale each vector
     * @return The scaled vector
     */
    public Vector scale(double scale) {
        return new Vector(x * scale, y * scale);
    }

    /**
     * Adds another vector to this one and returns the result.
     * 
     * @param v The vector to add to this one
     * @return The sum of the vectors
     */
    public Vector add(Vector v) {
        return new Vector(x + v.x, y + v.y);
    }

    /**
     * Computes the magnitude of this vector.
     * 
     * @return The magnitude of this vector
     */
    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Calculates the angle of this vector in radians.
     * 
     * @return The angle of this vector
     */
    public double getAngle() {
        return Math.atan2(y, x);
    }

    /**
     * Rotates the vector by an angle.
     * 
     * @param angle The rotation angle in radians
     * @return The rotated vector
     */
    public Vector rotate(double angle) {
        double rotatedX = x * Math.cos(angle) - y * Math.sin(angle);
        double rotatedY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector(rotatedX, rotatedY);
    }
}
