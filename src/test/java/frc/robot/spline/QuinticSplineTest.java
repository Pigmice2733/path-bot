package frc.robot.spline;

import java.util.ArrayList;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.plots.XYPlot;
import frc.robot.plots.XYPlot.Data;
import frc.robot.utils.Point;
import frc.robot.utils.Vector;

public class QuinticSplineTest {
    private static final double epsilon = 1e-6;
    private static QuinticSpline spline;

    @BeforeClass
    public static void init() {
        ArrayList<Double> knots = new ArrayList<>();
        knots.add(0.0);
        knots.add(2.0);
        knots.add(30.0);
        knots.add(50.0);

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(1.0, 0.0));
        points.add(new Point(0.0, 1.0));
        points.add(new Point(-1.0, 0.0));
        points.add(new Point(0.0, -1.0));

        ArrayList<Vector> derivatives = new ArrayList<>();
        derivatives.add(new Vector(0.0, 1.0));
        derivatives.add(new Vector(-1.0, 0.0));
        derivatives.add(new Vector(0.0, -1.0));
        derivatives.add(new Vector(1.0, 0.0));

        ArrayList<Vector> secondDerivatives = new ArrayList<>();
        secondDerivatives.add(new Vector(-1.0, 0.0));
        secondDerivatives.add(new Vector(0.0, -1.0));
        secondDerivatives.add(new Vector(1.0, 0.0));
        secondDerivatives.add(new Vector(0.0, 1.0));

        spline = new QuinticSpline(knots, points, derivatives, secondDerivatives);
    }

    @Test
    public void wheel() {
        Assert.assertEquals(new Point(2.0, 1.0), spline.getWheel(0.0, 1.0, 1.0));
        Assert.assertEquals(new Point(-1.0, 2.0), spline.getWheel(2.0, 1.0, 1.0));
        Assert.assertEquals(new Point(-2.0, -1.0), spline.getWheel(30.0, 1.0, 1.0));
        Assert.assertEquals(new Point(1.0, -2.0), spline.getWheel(50.0, 1.0, 1.0));
    }

    @Test
    public void position() {
        Assert.assertEquals(new Point(1.0, 0.0), spline.getPosition(0.0));
        Assert.assertEquals(new Point(0.0, 1.0), spline.getPosition(2.0));
        Assert.assertEquals(new Point(-1.0, 0.0), spline.getPosition(30.0));
        Assert.assertEquals(new Point(0.0, -1.0), spline.getPosition(50.0));
    }

    @Test
    public void derivative() {
        Assert.assertEquals(new Vector(0.0, 1.0), spline.getDerivative(0.0));
        Assert.assertEquals(new Vector(-1.0, 0.0), spline.getDerivative(2.0));
        Assert.assertEquals(new Vector(0.0, -1.0), spline.getDerivative(30.0));
        Assert.assertEquals(new Vector(1.0, 0.0), spline.getDerivative(50.0));
    }

    @Test
    public void secondDerivative() {
        Assert.assertEquals(new Vector(-1.0, 0.0), spline.getSecondDerivative(0.0));
        Assert.assertEquals(new Vector(0.0, -1.0), spline.getSecondDerivative(2.0));
        Assert.assertEquals(new Vector(1.0, 0.0), spline.getSecondDerivative(30.0));
        Assert.assertEquals(new Vector(0.0, 1.0), spline.getSecondDerivative(50.0));
    }

    @Test
    public void curvatureChunks() {
        ArrayList<Double> chunks = spline.computeSplineChunks(10.0).getCurvatureChunks();
        Assert.assertEquals(0.5, chunks.get(0), epsilon);
        Assert.assertEquals(-0.20015517144121364, chunks.get(1), epsilon);
        Assert.assertEquals(-0.023657782253813504, chunks.get(2), epsilon);
        Assert.assertEquals(-0.0023610720938180628, chunks.get(3), epsilon);
    }

    @Test
    public void headingChunks() {
        ArrayList<Double> chunks = spline.computeSplineChunks(10.0).getHeadingChunks();
        Assert.assertEquals(0.500000000, chunks.get(0), epsilon);
        Assert.assertEquals(0.847187428, chunks.get(1), epsilon);
        Assert.assertEquals(-1.556796834, chunks.get(2), epsilon);
        Assert.assertEquals(-0.018410761, chunks.get(3), epsilon);
    }

    @Test
    public void length() {
        Assert.assertEquals(50.0, spline.getLength(), epsilon);
    }

    @AfterClass
    public static void plot() {
        if (XYPlot.shouldGraph("splines")) {
            XYPlot plot = new XYPlot("QuinticSpline", "Center", spline::getPosition, spline.getLength(),
                    0.01 * spline.getLength());
            plot.addPoints("Control points", spline.getControlPoints());

            Data leftWheel = t -> spline.getWheel(t, -0.5, 0.0);
            Data rightWheel = t -> spline.getWheel(t, 0.5, 0.0);
            plot.addSeries("Left Wheel", leftWheel, spline.getLength(), 0.01 * spline.getLength());
            plot.addSeries("Right Wheel", rightWheel, spline.getLength(), 0.01 * spline.getLength());

            plot.save("splines");
        }
    }
}