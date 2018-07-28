package frc.robot.motion;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.Test;
import frc.robot.ProfilePlotter;;

public class StaticProfileTest {

    private static final double epsilon = 1e-6;

    private static Boolean shouldGraph() {
        String graphProp = System.getProperty("graph").toLowerCase();
        return (graphProp.equals("true") || graphProp.equals("t"));
    }

    public static class TrapezoidalTest {
        private static StaticProfile trapezoidalProfile = new StaticProfile(0.0, 0.0, 16.0, 4.0, 2.0, 1.0);

        @Test
        public void getVelocity() {
            // Acceleration
            Assert.assertEquals(0.0, trapezoidalProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(3.0, trapezoidalProfile.getVelocity(1.5), epsilon);

            // Max velocity
            Assert.assertEquals(4.0, trapezoidalProfile.getVelocity(2.0), epsilon);
            Assert.assertEquals(4.0, trapezoidalProfile.getVelocity(3.0), epsilon);

            // Deceleration
            Assert.assertEquals(1.0, trapezoidalProfile.getVelocity(6.0), epsilon);
            Assert.assertEquals(0.5, trapezoidalProfile.getVelocity(6.5), epsilon);
            Assert.assertEquals(0.0, trapezoidalProfile.getVelocity(7.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Acceleration
            Assert.assertEquals(0.0, trapezoidalProfile.getPosition(0.0), epsilon);
            Assert.assertEquals(2.25, trapezoidalProfile.getPosition(1.5), epsilon);

            // Max velocity
            Assert.assertEquals(4.0, trapezoidalProfile.getPosition(2.0), epsilon);
            Assert.assertEquals(8.0, trapezoidalProfile.getPosition(3.0), epsilon);

            // Deceleration
            Assert.assertEquals(15.5, trapezoidalProfile.getPosition(6.0), epsilon);
            Assert.assertEquals(15.875, trapezoidalProfile.getPosition(6.5), epsilon);
            Assert.assertEquals(16.0, trapezoidalProfile.getPosition(7.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph()) {
                ProfilePlotter.plotPosition(trapezoidalProfile, 0.1, "trapezoid");
                ProfilePlotter.plotVelocity(trapezoidalProfile, 0.1, "trapezoid");
            }
        }
    }

    public static class WrongDirectionTest {
        private static StaticProfile wrongDirectionProfile = new StaticProfile(-1, 0.5, 16.0, 4.0, 2.0, 1.0);

        @Test
        public void getVelocity() {
            // Acceleration
            Assert.assertEquals(0.0, wrongDirectionProfile.getVelocity(1.0), epsilon);
            Assert.assertEquals(3.0, wrongDirectionProfile.getVelocity(2.5), epsilon);

            // Max velocity
            Assert.assertEquals(4.0, wrongDirectionProfile.getVelocity(3.0), epsilon);
            Assert.assertEquals(4.0, wrongDirectionProfile.getVelocity(4.0), epsilon);

            // Deceleration
            Assert.assertEquals(1.0, wrongDirectionProfile.getVelocity(7.0), epsilon);
            Assert.assertEquals(0.5, wrongDirectionProfile.getVelocity(7.5), epsilon);
            Assert.assertEquals(0.0, wrongDirectionProfile.getVelocity(8.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Acceleration
            Assert.assertEquals(0.0, wrongDirectionProfile.getPosition(1.0), epsilon);
            Assert.assertEquals(2.25, wrongDirectionProfile.getPosition(2.5), epsilon);

            // Max velocity
            Assert.assertEquals(4.0, wrongDirectionProfile.getPosition(3.0), epsilon);
            Assert.assertEquals(8.0, wrongDirectionProfile.getPosition(4.0), epsilon);

            // Deceleration
            Assert.assertEquals(15.5, wrongDirectionProfile.getPosition(7.0), epsilon);
            Assert.assertEquals(15.875, wrongDirectionProfile.getPosition(7.5), epsilon);
            Assert.assertEquals(16.0, wrongDirectionProfile.getPosition(8.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph()) {
                ProfilePlotter.plotPosition(wrongDirectionProfile, 0.1, "wrongDir");
                ProfilePlotter.plotVelocity(wrongDirectionProfile, 0.1, "wrongDir");
            }
        }
    }

    public static class TriangularTest {
        private static StaticProfile triangularProfile = new StaticProfile(0.0, 0.0, 35.69, 9.0, 2.0, 2.15);

        @Test
        public void getVelocity() {
            // Acceleration
            // Assert.assertEquals(0.0, triangularProfile.getVelocity(0.0), epsilon);
            // Assert.assertEquals(8.0, triangularProfile.getVelocity(4.0), epsilon);

            // // Max velocity
            // Assert.assertEquals(8.6, triangularProfile.getVelocity(4.3), epsilon);

            // // Deceleration
            // Assert.assertEquals(4.3, triangularProfile.getVelocity(6.3), epsilon);
            // Assert.assertEquals(0.0, triangularProfile.getVelocity(8.3), epsilon);
        }

        @Test
        public void getPosition() {
            // // Acceleration
            // Assert.assertEquals(0.0, triangularProfile.getPosition(0.0), epsilon);

            // // Max velocity
            // Assert.assertEquals(18.49, triangularProfile.getPosition(4.3), epsilon);

            // // Deceleration
            // Assert.assertEquals(35.69, triangularProfile.getPosition(8.3), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph()) {
                ProfilePlotter.plotPosition(triangularProfile, 0.1, "triangle");
                ProfilePlotter.plotVelocity(triangularProfile, 0.1, "triangle");
            }
        }
    }
}
