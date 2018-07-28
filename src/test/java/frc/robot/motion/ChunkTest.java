package frc.robot.motion;

import org.junit.Assert;
import org.junit.Test;

public class ChunkTest {
    private static final double epsilon = 1e-6;

    @Test
    public void DistanceTest() {
        Chunk chunk = Chunk.createVelocityTransition(-5.0, -11.0, 2, 3);
        Assert.assertEquals(-24.0, chunk.getTotalDistance(), epsilon);

        Assert.assertEquals(0, chunk.getPosition(0.0), epsilon);
        Assert.assertEquals(-9.75, chunk.getPosition(1.5), epsilon);
    }

    @Test
    public void VelocityTest() {
        Chunk chunk = Chunk.createVelocityTransition(-2.0, -14.0, 4, 3);

        Assert.assertEquals(-2, chunk.getVelocity(0.0), epsilon);
        Assert.assertEquals(-8, chunk.getVelocity(1.5), epsilon);
    }
}
