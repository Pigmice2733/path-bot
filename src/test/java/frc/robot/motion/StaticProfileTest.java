package frc.robot.motion;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.Test;
import frc.robot.ProfilePlotter;;

public class StaticProfileTest {

    private static final double epsilon = 1e-6;

    private static Boolean shouldGraph(String graphName) {
        String graphProp = System.getProperty("graph").toLowerCase();
        return (graphProp.equals("true") || graphProp.equals("t") || graphProp.equals("all") || graphProp.equals("a")
                || graphProp.equals(graphName.toLowerCase()));
    }

    public static class TrapezoidalTest {
        // Pure trapezoid
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

        @Test
        public void getAcceleration() {
            // Acceleration
            Assert.assertEquals(2.0, trapezoidalProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(2.0, trapezoidalProfile.getAcceleration(1.5), epsilon);

            // Max velocity
            Assert.assertEquals(0.0, trapezoidalProfile.getAcceleration(2.0), epsilon);
            Assert.assertEquals(0.0, trapezoidalProfile.getAcceleration(2.99), epsilon);

            // Deceleration
            Assert.assertEquals(-1, trapezoidalProfile.getAcceleration(4.0), epsilon);
            Assert.assertEquals(-1, trapezoidalProfile.getAcceleration(6.5), epsilon);
            Assert.assertEquals(-1, trapezoidalProfile.getAcceleration(7.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("trapezoid")) {
                ProfilePlotter.plotPosition(trapezoidalProfile, 0.1, "trapezoid");
                ProfilePlotter.plotVelocity(trapezoidalProfile, 0.1, "trapezoid");
            }
        }
    }

    public static class WrongDirectionTest {
        // Trapezoid with leading wrong direction
        private static StaticProfile wrongDirectionProfile = new StaticProfile(-1, 0.5, 16.0, 4.0, 2.0, 1.0);

        @Test
        public void getVelocity() {
            // Direction correction
            Assert.assertEquals(-1.0, wrongDirectionProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(-0.5, wrongDirectionProfile.getVelocity(0.5), epsilon);

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
            // Direction correction
            Assert.assertEquals(0.5, wrongDirectionProfile.getPosition(0.0), epsilon);
            Assert.assertEquals(0.125, wrongDirectionProfile.getPosition(0.5), epsilon);

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

        @Test
        public void getAcceleration() {
            // Direction correction
            Assert.assertEquals(1.0, wrongDirectionProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(1.0, wrongDirectionProfile.getAcceleration(0.5), epsilon);

            // Acceleration
            Assert.assertEquals(2.0, wrongDirectionProfile.getAcceleration(1.0), epsilon);
            Assert.assertEquals(2.0, wrongDirectionProfile.getAcceleration(2.5), epsilon);

            // Max velocity
            Assert.assertEquals(0.0, wrongDirectionProfile.getAcceleration(3.0), epsilon);
            Assert.assertEquals(0.0, wrongDirectionProfile.getAcceleration(3.99), epsilon);

            // Deceleration
            Assert.assertEquals(-1, wrongDirectionProfile.getAcceleration(7.0), epsilon);
            Assert.assertEquals(-1, wrongDirectionProfile.getAcceleration(7.5), epsilon);
            Assert.assertEquals(-1, wrongDirectionProfile.getAcceleration(8.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("wrongDir")) {
                ProfilePlotter.plotPosition(wrongDirectionProfile, 0.1, "wrongDir");
                ProfilePlotter.plotVelocity(wrongDirectionProfile, 0.1, "wrongDir");
            }
        }
    }

    public static class TriangularTest {
        // Pure triangle
        private static StaticProfile triangularProfile = new StaticProfile(0.0, 0.0, 35.69, 9.0, 2.0, 2.15);

        @Test
        public void getVelocity() {
            // Acceleration
            Assert.assertEquals(0.0, triangularProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(8.0, triangularProfile.getVelocity(4.0), epsilon);

            // Max velocity
            Assert.assertEquals(8.6, triangularProfile.getVelocity(4.3), epsilon);

            // Deceleration
            Assert.assertEquals(4.3, triangularProfile.getVelocity(6.3), epsilon);
            Assert.assertEquals(0.0, triangularProfile.getVelocity(8.3), epsilon);
        }

        @Test
        public void getPosition() {
            // Acceleration
            Assert.assertEquals(0.0, triangularProfile.getPosition(0.0), epsilon);

            // Max velocity
            Assert.assertEquals(18.49, triangularProfile.getPosition(4.3), epsilon);

            // Deceleration
            Assert.assertEquals(35.69, triangularProfile.getPosition(8.3), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Acceleration
            Assert.assertEquals(2.0, triangularProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(2.0, triangularProfile.getAcceleration(4.2), epsilon);

            // Deceleration
            Assert.assertEquals(-2.15, triangularProfile.getAcceleration(4.4), epsilon);
            Assert.assertEquals(-2.15, triangularProfile.getAcceleration(8.3), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("triangle")) {
                ProfilePlotter.plotPosition(triangularProfile, 0.1, "triangle");
                ProfilePlotter.plotVelocity(triangularProfile, 0.1, "triangle");
            }
        }
    }

    public static class PartialTriangleTest {
        // Starts partway into triangle, leading corner chopped off
        private static StaticProfile quadrilateralProfile = new StaticProfile(1.0, 0.0, 5.5, 5.0, 1.0, 0.5);

        @Test
        public void getVelocity() {
            // Acceleration
            Assert.assertEquals(1.0, quadrilateralProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(1.5, quadrilateralProfile.getVelocity(0.5), epsilon);

            // Max velocity
            Assert.assertEquals(2.0, quadrilateralProfile.getVelocity(1.0), epsilon);

            // Deceleration
            Assert.assertEquals(1.0, quadrilateralProfile.getVelocity(3.0), epsilon);
            Assert.assertEquals(0.0, quadrilateralProfile.getVelocity(5.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Acceleration
            Assert.assertEquals(0.0, quadrilateralProfile.getPosition(0.0), epsilon);

            // Max velocity
            Assert.assertEquals(1.5, quadrilateralProfile.getPosition(1.0), epsilon);

            // Deceleration
            Assert.assertEquals(5.5, quadrilateralProfile.getPosition(5.0), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Acceleration
            Assert.assertEquals(1.0, quadrilateralProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(1.0, quadrilateralProfile.getAcceleration(0.9), epsilon);

            // Deceleration
            Assert.assertEquals(-0.5, quadrilateralProfile.getAcceleration(1.1), epsilon);
            Assert.assertEquals(-0.5, quadrilateralProfile.getAcceleration(5.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("partialTriangle")) {
                ProfilePlotter.plotPosition(quadrilateralProfile, 0.1, "partialTriangle");
                ProfilePlotter.plotVelocity(quadrilateralProfile, 0.1, "partialTriangle");
            }
        }
    }

    public static class ReverseTriangularTest {
        // Negative triangle profile, reverse direction
        private static StaticProfile reverseTriangleProfile = new StaticProfile(0.0, 0.0, -35.69, 9.0, 2.0, 2.15);

        @Test
        public void getVelocity() {
            // Acceleration
            Assert.assertEquals(0.0, reverseTriangleProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(-8.0, reverseTriangleProfile.getVelocity(4.0), epsilon);

            // Max velocity
            Assert.assertEquals(-8.6, reverseTriangleProfile.getVelocity(4.3), epsilon);

            // Deceleration
            Assert.assertEquals(-4.3, reverseTriangleProfile.getVelocity(6.3), epsilon);
            Assert.assertEquals(-0.0, reverseTriangleProfile.getVelocity(8.3), epsilon);
        }

        @Test
        public void getPosition() {
            // Acceleration
            Assert.assertEquals(0.0, reverseTriangleProfile.getPosition(0.0), epsilon);

            // Max velocity
            Assert.assertEquals(-18.49, reverseTriangleProfile.getPosition(4.3), epsilon);

            // Deceleration
            Assert.assertEquals(-35.69, reverseTriangleProfile.getPosition(8.3), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Acceleration
            Assert.assertEquals(-2.0, reverseTriangleProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(-2.0, reverseTriangleProfile.getAcceleration(4.2), epsilon);

            // Deceleration
            Assert.assertEquals(2.15, reverseTriangleProfile.getAcceleration(4.4), epsilon);
            Assert.assertEquals(2.15, reverseTriangleProfile.getAcceleration(5.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("reverseTriangle")) {
                ProfilePlotter.plotPosition(reverseTriangleProfile, 0.1, "reverseTriangle");
                ProfilePlotter.plotVelocity(reverseTriangleProfile, 0.1, "reverseTriangle");
            }
        }
    }

    public static class PartialTrapezoidTest {
        // Starts partway into trapezoid, leading corner chopped off
        private static StaticProfile pentagonProfile = new StaticProfile(2.5, 0.0, 11.25, 5.0, 2.5, 5.0);

        @Test
        public void getVelocity() {
            // Acceleration
            Assert.assertEquals(2.5, pentagonProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(3.75, pentagonProfile.getVelocity(0.5), epsilon);

            // Max velocity
            Assert.assertEquals(5.0, pentagonProfile.getVelocity(1.0), epsilon);
            Assert.assertEquals(5.0, pentagonProfile.getVelocity(2.0), epsilon);

            // Deceleration
            Assert.assertEquals(0.0, pentagonProfile.getVelocity(3.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Acceleration
            Assert.assertEquals(0.0, pentagonProfile.getPosition(0.0), epsilon);
            Assert.assertEquals(1.5625, pentagonProfile.getPosition(0.5), epsilon);

            // Max velocity
            Assert.assertEquals(3.75, pentagonProfile.getPosition(1.0), epsilon);
            Assert.assertEquals(8.75, pentagonProfile.getPosition(2.0), epsilon);

            // Deceleration
            Assert.assertEquals(10.625, pentagonProfile.getPosition(2.5), epsilon);
            Assert.assertEquals(11.25, pentagonProfile.getPosition(3.0), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Acceleration
            Assert.assertEquals(2.5, pentagonProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(2.5, pentagonProfile.getAcceleration(0.5), epsilon);

            // Max velocity
            Assert.assertEquals(0.0, pentagonProfile.getAcceleration(1.0), epsilon);
            Assert.assertEquals(0.0, pentagonProfile.getAcceleration(1.99), epsilon);

            // Deceleration
            Assert.assertEquals(-5.0, pentagonProfile.getAcceleration(2.5), epsilon);
            Assert.assertEquals(-5.0, pentagonProfile.getAcceleration(3.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("partialTrapezoid")) {
                ProfilePlotter.plotPosition(pentagonProfile, 0.1, "partialTrapezoid");
                ProfilePlotter.plotVelocity(pentagonProfile, 0.1, "partialTrapezoid");
            }
        }
    }

    public static class DecelerationTest {
        // Needs to decelerate directly from start of profile
        private static StaticProfile decelerationProfile = new StaticProfile(4.0, 4.0, 12.0, 6.0, 2.0, 1.0);

        @Test
        public void getVelocity() {

            // Deceleration
            Assert.assertEquals(4.0, decelerationProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(2.0, decelerationProfile.getVelocity(2.0), epsilon);
            Assert.assertEquals(0.0, decelerationProfile.getVelocity(4.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Deceleration
            Assert.assertEquals(4.0, decelerationProfile.getPosition(0.0), epsilon);
            Assert.assertEquals(10.0, decelerationProfile.getPosition(2.0), epsilon);
            Assert.assertEquals(12.0, decelerationProfile.getPosition(4.0), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Deceleration
            Assert.assertEquals(-1.0, decelerationProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(-1.0, decelerationProfile.getAcceleration(2.0), epsilon);
            Assert.assertEquals(-1.0, decelerationProfile.getAcceleration(4.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("deceleration")) {
                ProfilePlotter.plotPosition(decelerationProfile, 0.1, "deceleration");
                ProfilePlotter.plotVelocity(decelerationProfile, 0.1, "deceleration");
            }
        }
    }

    public static class MaxSpeedToDecelerationTest {
        // Starts at max speed, decelerates after max speed chunk
        private static StaticProfile maxSpeedToDecelerationProfile = new StaticProfile(8.0, 0.0, 40.0, 8.0, 2.0, 1.0);

        @Test
        public void getVelocity() {
            // Max velocity
            Assert.assertEquals(8.0, maxSpeedToDecelerationProfile.getVelocity(0.0), epsilon);
            Assert.assertEquals(8.0, maxSpeedToDecelerationProfile.getVelocity(1.0), epsilon);

            // Deceleration
            Assert.assertEquals(7.0, maxSpeedToDecelerationProfile.getVelocity(2.0), epsilon);
            Assert.assertEquals(2.5, maxSpeedToDecelerationProfile.getVelocity(6.5), epsilon);
            Assert.assertEquals(0.0, maxSpeedToDecelerationProfile.getVelocity(9.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Max velocity
            Assert.assertEquals(0.0, maxSpeedToDecelerationProfile.getPosition(0.0), epsilon);
            Assert.assertEquals(8.0, maxSpeedToDecelerationProfile.getPosition(1.0), epsilon);

            // Deceleration
            Assert.assertEquals(22.0, maxSpeedToDecelerationProfile.getPosition(3.0), epsilon);
            Assert.assertEquals(38.0, maxSpeedToDecelerationProfile.getPosition(7.0), epsilon);
            Assert.assertEquals(40.0, maxSpeedToDecelerationProfile.getPosition(9.0), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Max velocity
            Assert.assertEquals(0.0, maxSpeedToDecelerationProfile.getAcceleration(0.0), epsilon);
            Assert.assertEquals(0.0, maxSpeedToDecelerationProfile.getAcceleration(0.99), epsilon);

            // Deceleration
            Assert.assertEquals(-1.0, maxSpeedToDecelerationProfile.getAcceleration(3.0), epsilon);
            Assert.assertEquals(-1.0, maxSpeedToDecelerationProfile.getAcceleration(7.0), epsilon);
            Assert.assertEquals(-1.0, maxSpeedToDecelerationProfile.getAcceleration(9.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("maxSpeedDecel")) {
                ProfilePlotter.plotPosition(maxSpeedToDecelerationProfile, 0.1, "maxSpeedDecel");
                ProfilePlotter.plotVelocity(maxSpeedToDecelerationProfile, 0.1, "maxSpeedDecel");
            }
        }
    }

    public static class OvershootFromMaxSpeedTest {
        // Starts at max speed, overshoots while decelerating - needs to backtrack
        private static StaticProfile OvershootFromMaxSpeed = new StaticProfile(10.0, 0.0, 34.0, 10.0, 1.0, 1.0);

        @Test
        public void getVelocity() {
            // Deceleration
            Assert.assertEquals(10.0, OvershootFromMaxSpeed.getVelocity(0.0), epsilon);
            Assert.assertEquals(0.0, OvershootFromMaxSpeed.getVelocity(10.0), epsilon);

            // Backtrack triangle
            Assert.assertEquals(-4.0, OvershootFromMaxSpeed.getVelocity(14.0), epsilon);
            Assert.assertEquals(0.0, OvershootFromMaxSpeed.getVelocity(18.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Deceleration
            Assert.assertEquals(0.0, OvershootFromMaxSpeed.getPosition(0.0), epsilon);
            Assert.assertEquals(50.0, OvershootFromMaxSpeed.getPosition(10.0), epsilon);

            // Backtrack triangle
            Assert.assertEquals(42.0, OvershootFromMaxSpeed.getPosition(14.0), epsilon);
            Assert.assertEquals(34.0, OvershootFromMaxSpeed.getPosition(18.0), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Deceleration
            Assert.assertEquals(-1.0, OvershootFromMaxSpeed.getAcceleration(0.0), epsilon);
            Assert.assertEquals(-1.0, OvershootFromMaxSpeed.getAcceleration(9.99), epsilon);

            // Backtrack triangle
            Assert.assertEquals(-1.0, OvershootFromMaxSpeed.getAcceleration(13.5), epsilon);
            Assert.assertEquals(1.0, OvershootFromMaxSpeed.getAcceleration(18.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("overshootMax")) {
                ProfilePlotter.plotPosition(OvershootFromMaxSpeed, 0.1, "overshootMax");
                ProfilePlotter.plotVelocity(OvershootFromMaxSpeed, 0.1, "overshootMax");
            }
        }
    }

    public static class HighSpeedOvershootTest {
        // Starts at max speed, overshoots while decelerating - needs to backtrack
        private static StaticProfile HighSpeedOvershoot = new StaticProfile(50.0, 0.0, 1000.0, 10.0, 1.0, 1.0);

        @Test
        public void getVelocity() {
            // Deceleration
            Assert.assertEquals(50.0, HighSpeedOvershoot.getVelocity(0.0), epsilon);
            Assert.assertEquals(0.0, HighSpeedOvershoot.getVelocity(50.0), epsilon);

            // Backtrack trapezoid
            Assert.assertEquals(-10.0, HighSpeedOvershoot.getVelocity(60.0), epsilon);
            Assert.assertEquals(-10.0, HighSpeedOvershoot.getVelocity(75.0), epsilon);
            Assert.assertEquals(-5.0, HighSpeedOvershoot.getVelocity(80.0), epsilon);
            Assert.assertEquals(0.0, HighSpeedOvershoot.getVelocity(85.0), epsilon);
        }

        @Test
        public void getPosition() {
            // Deceleration
            Assert.assertEquals(0.0, HighSpeedOvershoot.getPosition(0.0), epsilon);
            Assert.assertEquals(1250.0, HighSpeedOvershoot.getPosition(50.0), epsilon);

            // Backtrack trapezoid
            Assert.assertEquals(1200.0, HighSpeedOvershoot.getPosition(60.0), epsilon);
            Assert.assertEquals(1050.0, HighSpeedOvershoot.getPosition(75.0), epsilon);
            Assert.assertEquals(1000.0, HighSpeedOvershoot.getPosition(85.0), epsilon);
        }

        @Test
        public void getAcceleration() {
            // Deceleration
            Assert.assertEquals(-1.0, HighSpeedOvershoot.getAcceleration(0.0), epsilon);
            Assert.assertEquals(-1.0, HighSpeedOvershoot.getAcceleration(49.0), epsilon);

            // Backtrack trapezoid
            Assert.assertEquals(0.0, HighSpeedOvershoot.getAcceleration(60.0), epsilon);
            Assert.assertEquals(1.0, HighSpeedOvershoot.getAcceleration(80.0), epsilon);
            Assert.assertEquals(1.0, HighSpeedOvershoot.getAcceleration(85.0), epsilon);
        }

        @AfterClass
        public static void plot() {
            if (shouldGraph("highOvershoot")) {
                ProfilePlotter.plotPosition(HighSpeedOvershoot, 0.1, "highOvershoot");
                ProfilePlotter.plotVelocity(HighSpeedOvershoot, 0.1, "highOvershoot");
            }
        }
    }
}
