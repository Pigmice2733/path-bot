package frc.robot.spline;

import org.junit.Test;

import frc.robot.utils.Point;
import frc.robot.utils.Vector;
import org.junit.Assert;

public class QuinticSplineSegmentTest {
    private static final double epsilon = 1e-6;

    @Test
    public void straightSegment() {
        Point start = new Point(0.0, 0.0);
        Point end = new Point(0.0, 5.0);
        Vector firstStart = new Vector(0.0, 1.0);
        Vector firstEnd = new Vector(0.0, 1.0);
        Vector secondStart = new Vector(0.0, 0.0);
        Vector secondEnd = new Vector(0.0, 0.0);

        QuinticSplineSegment segment = new QuinticSplineSegment(start, end, firstStart, firstEnd, secondStart,
                secondEnd);

        Assert.assertEquals(5.0, segment.getArcLength(), 1e-4);

        Assert.assertEquals(start, segment.getPosition(0.0));
        Assert.assertEquals(new Point(0.0, 3.330239999999998), segment.getPosition(0.6));
        Assert.assertEquals(end, segment.getPosition(1.0));

        Assert.assertEquals(firstStart, segment.getDerivative(0.0));
        Assert.assertEquals(new Vector(0.0, 7.912000000000006), segment.getDerivative(0.6));
        Assert.assertEquals(firstEnd, segment.getDerivative(1.0));

        Assert.assertEquals(secondStart, segment.getSecondDerivative(0.0));
        Assert.assertEquals(new Vector(0.0, -11.520000000000003), segment.getSecondDerivative(0.6));
        Assert.assertEquals(secondEnd, segment.getSecondDerivative(1.0));

        Assert.assertEquals(0.0, segment.getCurvature(0.0), epsilon);
        Assert.assertEquals(0.0, segment.getCurvature(0.6), epsilon);
        Assert.assertEquals(0.0, segment.getCurvature(1.0), epsilon);
    }

    @Test
    public void curvedSegment() {
        Point start = new Point(0.0, 0.0);
        Point end = new Point(0.0, 5.0);
        Vector firstStart = new Vector(1.0, 0.0);
        Vector firstEnd = new Vector(-1.0, 0.0);
        Vector secondStart = new Vector(0.0, 0.0);
        Vector secondEnd = new Vector(0.0, 0.0);

        QuinticSplineSegment segment = new QuinticSplineSegment(start, end, firstStart, firstEnd, secondStart,
                secondEnd);

        Assert.assertEquals(5.1848, segment.getArcLength(), 1e-4);

        Assert.assertEquals(start, segment.getPosition(0.0));
        Assert.assertEquals(new Point(0.2541, 0.8153999), segment.getPosition(0.3));
        Assert.assertEquals(end, segment.getPosition(1.0));

        Assert.assertEquals(firstStart, segment.getDerivative(0.0));
        Assert.assertEquals(new Vector(0.5680000, 6.615), segment.getDerivative(0.3));
        Assert.assertEquals(firstEnd, segment.getDerivative(1.0));

        Assert.assertEquals(secondStart, segment.getSecondDerivative(0.0));
        Assert.assertEquals(new Vector(-2.5199999, 25.2), segment.getSecondDerivative(0.3));
        Assert.assertEquals(secondEnd, segment.getSecondDerivative(1.0));

        Assert.assertEquals(0.0, segment.getCurvature(0.0), epsilon);
        Assert.assertEquals(0.04510946, segment.getCurvature(0.6), epsilon);
        Assert.assertEquals(0.0, segment.getCurvature(1.0), epsilon);
    }
}
