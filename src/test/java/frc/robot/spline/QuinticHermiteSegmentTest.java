package frc.robot.spline;

import org.junit.Test;

import frc.robot.utils.Point;
import frc.robot.utils.Vector;
import org.junit.Assert;

public class QuinticHermiteSegmentTest {
    @Test
    public void straightSegment() {
        Point start = new Point(0.0, 0.0);
        Point end = new Point(0.0, 5.0);
        Vector firstStart = new Vector(0.0, 1.0);
        Vector firstEnd = new Vector(0.0, 1.0);
        Vector secondStart = new Vector(0.0, 0.0);
        Vector secondEnd = new Vector(0.0, 0.0);

        QuinticHermiteSegment segment = new QuinticHermiteSegment(start, end, firstStart, firstEnd, secondStart,
                secondEnd);

        Assert.assertEquals(start, segment.position(0.0));
        Assert.assertEquals(end, segment.position(1.0));
        Assert.assertEquals(firstStart, segment.derivative(0.0));
        Assert.assertEquals(firstEnd, segment.derivative(1.0));
        Assert.assertEquals(secondStart, segment.secondDerivative(0.0));
        Assert.assertEquals(secondEnd, segment.secondDerivative(1.0));

        Assert.assertEquals(new Point(0.0, 3.330239999999998), segment.position(0.6));
        Assert.assertEquals(new Vector(0.0, 7.912000000000006), segment.derivative(0.6));
        Assert.assertEquals(new Vector(0.0, -11.520000000000003), segment.secondDerivative(0.6));
    }

    @Test
    public void curvedSegment() {
        Point start = new Point(0.0, 0.0);
        Point end = new Point(0.0, 5.0);
        Vector firstStart = new Vector(1.0, 0.0);
        Vector firstEnd = new Vector(-1.0, 0.0);
        Vector secondStart = new Vector(0.0, 0.0);
        Vector secondEnd = new Vector(0.0, 0.0);

        QuinticHermiteSegment segment = new QuinticHermiteSegment(start, end, firstStart, firstEnd, secondStart,
                secondEnd);

        Assert.assertEquals(start, segment.position(0.0));
        Assert.assertEquals(end, segment.position(1.0));
        Assert.assertEquals(firstStart, segment.derivative(0.0));
        Assert.assertEquals(firstEnd, segment.derivative(1.0));
        Assert.assertEquals(secondStart, segment.secondDerivative(0.0));
        Assert.assertEquals(secondEnd, segment.secondDerivative(1.0));

        Assert.assertEquals(new Point(0.2541, 0.8153999999999997), segment.position(0.3));
        Assert.assertEquals(new Vector(0.5680000000000002, 6.615), segment.derivative(0.3));
        Assert.assertEquals(new Vector(-2.5199999999999987, 25.2), segment.secondDerivative(0.3));
    }
}
