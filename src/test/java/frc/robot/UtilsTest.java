package frc.robot;

import org.junit.Assert;
import org.junit.Test;

public class UtilsTest {

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
