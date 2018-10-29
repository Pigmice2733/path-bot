package frc.robot.spline;

import org.junit.Test;

import frc.robot.plots.XYPlot;
import frc.robot.plots.XYPlot.Data;
import frc.robot.utils.Point;
import frc.robot.utils.Vector;

import java.util.ArrayList;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;

public class QuinticSplineSegmentTest {
    private static final double epsilon = 1e-6;
    private static QuinticSplineSegment straight, curved;

    @BeforeClass
    public static void init() {
        Point start = new Point(0.0, 0.0);
        Point end = new Point(0.0, 5.0);
        Vector firstStart = new Vector(0.0, 1.0);
        Vector firstEnd = new Vector(0.0, 1.0);
        Vector secondStart = new Vector(0.0, 0.0);
        Vector secondEnd = new Vector(0.0, 0.0);

        straight = new QuinticSplineSegment(start, end, firstStart, firstEnd, secondStart, secondEnd);

        firstStart = new Vector(1.0, 0.0);
        firstEnd = new Vector(-1.0, 0.0);

        curved = new QuinticSplineSegment(start, end, firstStart, firstEnd, secondStart, secondEnd);
    }

    @Test
    public void arcLength() {
        Assert.assertEquals(5.0, straight.getArcLength(), 1e-4);
        Assert.assertEquals(5.1848, curved.getArcLength(), 1e-4);
    }

    @Test
    public void position() {
        Assert.assertEquals(new Point(0.0, 0.0), straight.getPosition(0.0));
        Assert.assertEquals(new Point(0.0, 3.330239999999998), straight.getPosition(0.6));
        Assert.assertEquals(new Point(0.0, 5.0), straight.getPosition(1.0));

        Assert.assertEquals(new Point(0.0, 0.0), curved.getPosition(0.0));
        Assert.assertEquals(new Point(0.2541, 0.8153999), curved.getPosition(0.3));
        Assert.assertEquals(new Point(0.0, 5.0), curved.getPosition(1.0));
    }

    @Test
    public void derivative() {
        Assert.assertEquals(new Vector(0.0, 1.0), straight.getDerivative(0.0));
        Assert.assertEquals(new Vector(0.0, 7.912000000000006), straight.getDerivative(0.6));
        Assert.assertEquals(new Vector(0.0, 1.0), straight.getDerivative(1.0));

        Assert.assertEquals(new Vector(1.0, 0.0), curved.getDerivative(0.0));
        Assert.assertEquals(new Vector(0.5680000, 6.615), curved.getDerivative(0.3));
        Assert.assertEquals(new Vector(-1.0, 0.0), curved.getDerivative(1.0));
    }

    @Test
    public void secondDerivative() {
        Assert.assertEquals(new Vector(0.0, 0.0), straight.getSecondDerivative(0.0));
        Assert.assertEquals(new Vector(0.0, -11.520000000000003), straight.getSecondDerivative(0.6));
        Assert.assertEquals(new Vector(0.0, 0.0), straight.getSecondDerivative(1.0));

        Assert.assertEquals(new Vector(0.0, 0.0), curved.getSecondDerivative(0.0));
        Assert.assertEquals(new Vector(-2.5199999, 25.2), curved.getSecondDerivative(0.3));
        Assert.assertEquals(new Vector(0.0, 0.0), curved.getSecondDerivative(1.0));
    }

    @Test
    public void wheel() {
        Assert.assertEquals(Math.sqrt(50.0),
                straight.getWheel(0.6, 5.0, -5.0).subtract(straight.getPosition(0.6)).getMagnitude(), epsilon);
        Assert.assertEquals(Math.sqrt(29.0),
                straight.getWheel(0.4, -2.0, 5.0).subtract(straight.getPosition(0.4)).getMagnitude(), epsilon);
        Assert.assertEquals(3.0, straight.getWheel(0.2, 3.0, 0.0).subtract(straight.getPosition(0.2)).getMagnitude(),
                epsilon);
        Assert.assertEquals(5.0, straight.getWheel(0.6, 5.0, -5.0).subtract(straight.getPosition(0.6)).getX(), epsilon);

        Assert.assertEquals(Math.sqrt(50.0),
                curved.getWheel(0.6, 5.0, -5.0).subtract(curved.getPosition(0.6)).getMagnitude(), epsilon);
        Assert.assertEquals(Math.sqrt(29.0),
                curved.getWheel(0.4, -2.0, 5.0).subtract(curved.getPosition(0.4)).getMagnitude(), epsilon);
        Assert.assertEquals(3.0, curved.getWheel(0.2, 3.0, 0.0).subtract(curved.getPosition(0.2)).getMagnitude(),
                epsilon);
    }

    @Test
    public void curvature() {
        Assert.assertEquals(0.0, straight.getCurvature(0.0), epsilon);
        Assert.assertEquals(0.0, straight.getCurvature(0.6), epsilon);
        Assert.assertEquals(0.0, straight.getCurvature(1.0), epsilon);

        Assert.assertEquals(0.0, curved.getCurvature(0.0), epsilon);
        Assert.assertEquals(0.04510946, curved.getCurvature(0.6), epsilon);
        Assert.assertEquals(0.0, curved.getCurvature(1.0), epsilon);
    }

    @Test
    public void heading() {
        Assert.assertEquals(0.5 * Math.PI, straight.getHeading(0.0), epsilon);
        Assert.assertEquals(0.5 * Math.PI, straight.getHeading(0.5), epsilon);
        Assert.assertEquals(0.5 * Math.PI, straight.getHeading(1.0), epsilon);

        Assert.assertEquals(0.0, curved.getHeading(0.0), epsilon);
        Assert.assertEquals(0.5 * Math.PI, curved.getHeading(0.5), epsilon);
        Assert.assertEquals(Math.PI, curved.getHeading(1.0), epsilon);
    }

    @AfterClass
    public static void plot() {
        if (XYPlot.shouldGraph("splines")) {
            XYPlot plot = new XYPlot("CurvedSegment", "Center", curved::getPosition, 1.0, 0.01);

            ArrayList<Point> controlPoints = new ArrayList<>();
            controlPoints.add(curved.start);
            controlPoints.add(curved.end);
            plot.addPoints("Control points", controlPoints);

            Data leftWheel = t -> curved.getWheel(t, -0.5, 0.0);
            Data rightWheel = t -> curved.getWheel(t, 0.5, 0.0);
            plot.addSeries("Left Wheel", leftWheel, 1.0, 0.01);
            plot.addSeries("Right Wheel", rightWheel, 1.0, 0.01);

            plot.save("splines");
        }
    }
}
