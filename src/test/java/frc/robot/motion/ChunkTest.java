package frc.robot.motion;

import org.junit.Assert;
import org.junit.Test;

public class ChunkTest {
    private static final double epsilon = 1e-6;

    @Test
    public void DistanceTest() {
        Chunk chunk = Chunk.createVelocityTransition(-5.0, -11.0, 2.0, 3.0);
        Assert.assertEquals(-24.0, chunk.getTotalDistance(), epsilon);

        Assert.assertEquals(0.0, chunk.getPosition(0.0), epsilon);
        Assert.assertEquals(-9.75, chunk.getPosition(1.5), epsilon);

        chunk = Chunk.createVelocityTransition(1.0, 16.0, 3.0, 5.0);
        Assert.assertEquals(42.5, chunk.getTotalDistance(), epsilon);

        Assert.assertEquals(0.0, chunk.getPosition(0.0), epsilon);
        Assert.assertEquals(28.0, chunk.getPosition(4.0), epsilon);

        chunk = Chunk.createConstantVelocity(10.0, 25.0);
        Assert.assertEquals(25.0, chunk.getTotalDistance(), epsilon);
        Assert.assertEquals(15.0, chunk.getPosition(1.5), epsilon);
    }

    @Test
    public void VelocityTest() {
        Chunk chunk = Chunk.createVelocityTransition(-2.0, -14.0, 4.0, 3.0);
        Assert.assertEquals(-2.0, chunk.getVelocity(0.0), epsilon);
        Assert.assertEquals(-8.0, chunk.getVelocity(1.5), epsilon);

        chunk = Chunk.createConstantVelocity(10.0, 25.0);
        Assert.assertEquals(10.0, chunk.getVelocity(1.23), epsilon);
    }

    @Test
    public void AccelerationTest() {
        Chunk chunk = Chunk.createVelocityTransition(-2.0, -14.0, 4.0, 3.0);
        Assert.assertEquals(-4.0, chunk.getAcceleration(), epsilon);
        chunk = Chunk.createVelocityTransition(0, 10.0, 2.0, 5.0);
        Assert.assertEquals(2.0, chunk.getAcceleration(), epsilon);
        chunk = Chunk.createConstantVelocity(10.0, 25.0);
        Assert.assertEquals(0.0, chunk.getAcceleration(), epsilon);
    }
}
