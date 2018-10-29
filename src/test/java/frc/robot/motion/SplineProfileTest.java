package frc.robot.motion;

import java.util.ArrayList;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.plots.ProfilePlot;
import frc.robot.spline.QuinticSpline;
import frc.robot.utils.Point;
import frc.robot.utils.Vector;

public class SplineProfileTest {
    private static final double epsilon = 1e-3;
    private static SplineProfile profile;

    @BeforeClass
    public static void init() {
        ArrayList<Double> knots = new ArrayList<>();
        knots.add(0.0);
        knots.add(1.0);
        knots.add(2.0);

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0.0, 0.0));
        points.add(new Point(0.0, 5.0));
        points.add(new Point(0.0, 10.0));

        ArrayList<Vector> derivatives = new ArrayList<>();
        derivatives.add(new Vector(0.0, 5.0));
        derivatives.add(new Vector(0.0, 5.0));
        derivatives.add(new Vector(0.0, 5.0));

        ArrayList<Vector> secondDerivatives = new ArrayList<>();
        secondDerivatives.add(new Vector(0.0, 0.0));
        secondDerivatives.add(new Vector(0.0, 0.0));
        secondDerivatives.add(new Vector(0.0, 0.0));

        QuinticSpline spline = new QuinticSpline(knots, points, derivatives, secondDerivatives);
        profile = new SplineProfile(spline, 0.1, 3.0, 2.0, 0.7);
    }

    @Test
    public void straightPath() {
        Setpoint sp = profile.getSetpointAtTime(1.4);
        Assert.assertEquals(1.9600, sp.getPosition(), epsilon);
        Assert.assertEquals(2.8000, sp.getVelocity(), epsilon);
        Assert.assertEquals(2.0, sp.getAcceleration(), epsilon);
        Assert.assertEquals(0.0, sp.getCurvature(), epsilon);
        Assert.assertEquals(Math.PI / 2.0, sp.getHeading(), epsilon);

        sp = profile.getSetpointAtTime(2.4);
        Assert.assertEquals(4.9500, sp.getPosition(), epsilon);
        Assert.assertEquals(3.0000, sp.getVelocity(), epsilon);
        Assert.assertEquals(0.0, sp.getAcceleration(), epsilon);
        Assert.assertEquals(0.0, sp.getCurvature(), epsilon);
        Assert.assertEquals(Math.PI / 2.0, sp.getHeading(), epsilon);

        sp = profile.getSetpointAtTime(4.0);
        Assert.assertEquals(9.3052, sp.getPosition(), epsilon);
        Assert.assertEquals(1.6670, sp.getVelocity(), epsilon);
        Assert.assertEquals(-2.0, sp.getAcceleration(), epsilon);
        Assert.assertEquals(0.0, sp.getCurvature(), epsilon);
        Assert.assertEquals(Math.PI / 2.0, sp.getHeading(), epsilon);
    }

    @AfterClass
    public static void plot() {
        if (ProfilePlot.shouldGraph("profiles")) {
            ProfilePlot plot = new ProfilePlot("SplineProfile", profile::getSetpointAtTime, profile.getLength(),
                    0.01 * profile.getLength());
            plot.save("profiles");
        }
    }
}