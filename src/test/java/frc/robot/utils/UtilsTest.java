package frc.robot.utils;

import org.junit.Assert;
import org.junit.Test;

public class UtilsTest {
    public static class LerpTest {
        private static final double epsilon = 1e-6;

        @Test
        public void lerpEqualRanges() {
            Assert.assertEquals(2.0, Utils.lerp(2.0, 1.0, 3.0, 1.0, 3.0), epsilon);
        }

        @Test
        public void lerpOffsetEqualRanges() {
            Assert.assertEquals(7.0, Utils.lerp(6.0, 5.0, 7.0, 6.0, 8.0), epsilon);
        }

        @Test
        public void lerpUnequalPositiveRanges() {
            Assert.assertEquals(3.0, Utils.lerp(3.0, 2.0, 6.0, 1.0, 9.0), epsilon);
        }

        @Test
        public void lerpEqualNegativeRanges() {
            Assert.assertEquals(-4.0, Utils.lerp(-1.0, 0.0, -2.0, -3.0, -5.0), epsilon);
        }

        @Test
        public void lerpUnequalNegativeRanges() {
            Assert.assertEquals(-5.0, Utils.lerp(-1.0, 0.0, -2.0, -3.0, -7.0), epsilon);
        }

        @Test
        public void lerpMixedSignRanges() {
            Assert.assertEquals(Utils.lerp(-1.0, 0.0, -2.0, 5.0, 9.0), 7.0, epsilon);
        }
    }

    public static class AlmostEqualsTest {
        private static final double epsilon = 1e-6;

        @Test
        public void equal() {
            Assert.assertTrue(Utils.almostEquals(1.0000001, 1.000000, epsilon));
            Assert.assertTrue(Utils.almostEquals(1 / 3, 1 / 3 + 1e-7, epsilon));

            Assert.assertTrue(Utils.almostEquals(-1.0000001, -1.000000, epsilon));
            Assert.assertTrue(Utils.almostEquals(-0.0, -0.000000001, epsilon));

            Assert.assertTrue(Utils.almostEquals(-0.0, +0.0, epsilon));
        }

        @Test
        public void notEqual() {
            Assert.assertFalse(Utils.almostEquals(1.0000001, 2, epsilon));
            Assert.assertFalse(Utils.almostEquals(-0.0, 5.0, epsilon));
            Assert.assertFalse(Utils.almostEquals(-2.0, 2.0, epsilon));

        }
    }
}
