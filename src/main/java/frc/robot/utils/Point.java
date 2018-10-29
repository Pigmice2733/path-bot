package frc.robot.utils;

/**
 * 2D point data type
 */
public class Point implements XY {
    private final double x, y;

    /**
     * Constructs a point from x and y coordinates.
     * 
     * @param x The x coordinate of the point
     * @param y The y coordinate of the point
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the origin (0, 0) point, each component
     * is zero.
     * 
     * @return The origin
     */
    public static Point origin() {
        return new Point(0, 0);
    }

    /**
     * Gets the x coordinate of the point.
     * 
     * @return The x coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the y coordinate of the point.
     * 
     * @return The y coordinate
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
        Point other = (Point) o;
        return Utils.almostEquals(x, other.x) && Utils.almostEquals(y, other.y);
    }

    @Override
    public int hashCode() {
        return (int) (7.0 + 3.0 * x + 227.0 * y);
    }

    /**
     * Translates this point by a vector.
     * 
     * @param translation The vector to add to this point
     * @return The translated vector
     */
    public Point translate(Vector translation) {
        return new Point(x + translation.getX(), y + translation.getY());
    }

    /**
     * Finds the vector offset between this point and another.
     * 
     * @param p The point to subtract from this one
     * @return The vector offset between the points
     */
    public Vector subtract(Point p) {
        return new Vector(x - p.x, y - p.y);
    }

    /**
     * Rotates this point
     * 
     * @param angle  The angle in radians to rotate by
     * @param center The center of rotation
     * @return The rotated point
     */
    public Point rotate(double angle, Point center) {
        Vector offset = subtract(center);
        Vector rotated = offset.rotate(angle);
        return center.translate(rotated);
    }
}
