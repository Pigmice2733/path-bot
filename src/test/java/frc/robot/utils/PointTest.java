package frc.robot.utils;

import org.junit.Assert;
import org.junit.Test;

public class PointTest {
    private static final double epsilon = 1e-6;

    @Test
    public void xyTest() {
        Point p = new Point(5.3, 6.0);
        Assert.assertEquals(5.3, p.getX(), epsilon);
        Assert.assertEquals(6.0, p.getY(), epsilon);

    }

    @Test
    public void transformTest() {
        Vector transform = new Vector(0.0, 0.0);
        Point p = new Point(4.0, -12.0);
        Point transformed = p.translate(transform);
        Assert.assertEquals(4.0, transformed.getX(), epsilon);
        Assert.assertEquals(-12.0, transformed.getY(), epsilon);

        transform = new Vector(-12.0, 3.0);
        transformed = p.translate(transform);
        Assert.assertEquals(-8.0, transformed.getX(), epsilon);
        Assert.assertEquals(-9.0, transformed.getY(), epsilon);
    }

    @Test
    public void subtractTest() {
        Point b = new Point(-32.0, 1.0);
        Point a = new Point(4.0, -12.0);
        Vector offset = b.subtract(a);

        Assert.assertEquals(-36.0, offset.getX(), epsilon);
        Assert.assertEquals(13.0, offset.getY(), epsilon);
    }

    @Test
    public void rotateTest() {
        Point orginal = new Point(-1.0, -1.0);
        Point center = new Point(0.0, 0.0);
        Point rotated = orginal.rotate(-0.5 * Math.PI, center);

        Assert.assertEquals(-1.0, rotated.getX(), epsilon);
        Assert.assertEquals(1.0, rotated.getY(), epsilon);

        orginal = new Point(-4.0, 4.0);
        center = new Point(-5.0, 5.0);
        rotated = orginal.rotate(1.5 * Math.PI, center);

        Assert.assertEquals(-6.0, rotated.getX(), epsilon);
        Assert.assertEquals(4.0, rotated.getY(), epsilon);
    }
}
