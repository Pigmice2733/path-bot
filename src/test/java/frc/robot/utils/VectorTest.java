package frc.robot.utils;

import org.junit.Assert;
import org.junit.Test;

public class VectorTest {
    private static final double epsilon = 1e-6;

    @Test
    public void xyTest() {
        Vector v = new Vector(-5.3, 4.0);
        Assert.assertEquals(-5.3, v.getX(), epsilon);
        Assert.assertEquals(4.0, v.getY(), epsilon);

    }

    @Test
    public void zeroTest() {
        Vector zero = Vector.zero();
        Assert.assertEquals(0.0, zero.getX(), epsilon);
        Assert.assertEquals(0.0, zero.getY(), epsilon);
    }

    @Test
    public void scaleTest() {
        Vector original = new Vector(-2.0, 5.0);
        Vector scaled = original.scale(-10.0);
        Assert.assertEquals(20.0, scaled.getX(), epsilon);
        Assert.assertEquals(-50.0, scaled.getY(), epsilon);

        scaled = scaled.scale(-0.1);
        Assert.assertEquals(-2.0, scaled.getX(), epsilon);
        Assert.assertEquals(5.0, scaled.getY(), epsilon);
    }

    @Test
    public void addTest() {
        Vector a = new Vector(-2.0, 5.0);
        Vector b = new Vector(-10.0, -12.0);
        Vector c = a.add(b);

        Assert.assertEquals(-12.0, c.getX(), epsilon);
        Assert.assertEquals(-7.0, c.getY(), epsilon);
    }

    @Test
    public void getMagnitudeTest() {
        Vector mixedSign = new Vector(-2.0, 5.0);
        Vector negative = new Vector(-3.0, -4.0);
        Vector zero = new Vector(0.0, 0.0);

        Assert.assertEquals(Math.sqrt(29.0), mixedSign.getMagnitude(), epsilon);
        Assert.assertEquals(5.0, negative.getMagnitude(), epsilon);
        Assert.assertEquals(0.0, zero.getMagnitude(), epsilon);
    }

    @Test
    public void rotateTest() {
        Vector original = new Vector(-1.0, 1.0);
        Vector rotated = original.rotate(-1.5 * Math.PI);

        Assert.assertEquals(-1.0, rotated.getX(), epsilon);
        Assert.assertEquals(-1.0, rotated.getY(), epsilon);

        rotated = Vector.zero().rotate(Math.PI);
        Assert.assertEquals(0.0, rotated.getX(), epsilon);
        Assert.assertEquals(0.0, rotated.getY(), epsilon);
    }
}
