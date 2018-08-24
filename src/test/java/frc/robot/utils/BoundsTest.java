package frc.robot.utils;

import org.junit.Assert;
import org.junit.Test;

public class BoundsTest {
    private static final double epsilon = 1e-6;

    @Test
    public void size() {
        Bounds bounds = new Bounds(0.0, 0.0);
        Assert.assertEquals(0.0, bounds.size(), epsilon);
        bounds = new Bounds(-2.0, 2.0);
        Assert.assertEquals(4.0, bounds.size(), epsilon);
        bounds = Bounds.noBounds();
        Assert.assertEquals(Double.POSITIVE_INFINITY, bounds.size(), epsilon);
    }

    @Test
    public void clamp() {
        Bounds bounds = new Bounds(0.0, 0.0);
        Assert.assertEquals(0.0, bounds.clamp(-6.3), epsilon);
        Assert.assertEquals(0.0, bounds.clamp(0.0), epsilon);
        bounds = new Bounds(-2.0, 2.0);
        Assert.assertEquals(1.0, bounds.clamp(1.0), epsilon);
        Assert.assertEquals(-2.0, bounds.clamp(-4), epsilon);
        bounds = Bounds.noBounds();
        Assert.assertEquals(-120.6, bounds.clamp(-120.6), epsilon);
        Assert.assertEquals(Double.MAX_VALUE, bounds.clamp(Double.MAX_VALUE), epsilon);
        Assert.assertEquals(Double.MIN_VALUE, bounds.clamp(Double.MIN_VALUE), epsilon);
    }
}