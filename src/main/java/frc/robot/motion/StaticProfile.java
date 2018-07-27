package frc.robot.motion;

import java.lang.Math;
import java.util.ArrayList;

public class StaticProfile {

    private static final double epsilon = 1e-6;
    private final ArrayList<Chunk> chunks;
    private double maxAccel, maxDecel, maxVelocity, startingPosition;
    public double profileDuration;

    private class Moment {
        private final Chunk chunk;
        private final double time;
        private final double previousDistance;

        public Moment(Chunk chunk, double time, double previousDistance) {
            this.chunk = chunk;
            this.time = time;
            this.previousDistance = previousDistance;
        }

        public double getVelocity() {
            return chunk.getVelocity(time);
        }

        public double getPosition() {
            return chunk.getPosition(time) + previousDistance;
        }
    }

    public StaticProfile(double currentVelocity, double currentPosition, double targetDistance,
            double maxVelocity, double maxAccel, double maxDecel) {
        final double targetDisplacement = targetDistance - currentPosition;
        startingPosition = currentPosition;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVelocity = maxVelocity;

        chunks = computeChunks(new ArrayList<Chunk>(), currentVelocity, targetDisplacement);

        profileDuration = 0.0;
        for (Chunk chunk : chunks) {
            profileDuration += chunk.getDuration();
        }
    }

    private ArrayList<Chunk> computeChunks(ArrayList<Chunk> chunks, double startVelocity,
            double remainingDistance) {
        Chunk chunk;
        // going in the wrong direction
        if (Math.signum(startVelocity) != Math.signum(remainingDistance) && startVelocity != 0) {
            // transition to 0
            chunk = Chunk.createVelocityTransition(startVelocity, 0, maxAccel, maxDecel);
        }
        // not going at max speed
        else if (startVelocity != maxVelocity) {
            // transition to max speed
            chunk = Chunk.createVelocityTransition(startVelocity, maxVelocity, maxAccel, maxDecel);
        }
        // going at max speed
        else {
            // create chunk that transitions to 0
            final Chunk decelChunk =
                    Chunk.createVelocityTransition(startVelocity, 0, maxAccel, maxDecel);
            if (decelChunk.getTotalDistance() < remainingDistance) {
                // doesn't go far enough
                chunk = Chunk.createConstantVelocity(maxVelocity,
                        remainingDistance - decelChunk.getTotalDistance());
            } else {
                chunk = decelChunk;
                // need to decide whether to create ending chunk here, or that it is a triangle
                // and it needs to rewrite the chunks
            }
        }
        // going in the wrong direction: create a chunk that transitions to 0
        // going faster than max speed: transition to max speed
        // going slower than max speed: transition to max speed
        // going at max speed:
        // create a chunk that goes to zero
        // if that chunk distance is total distance, use that chunk
        // otherwise create a new constant chunk that is the rest of the distance

        chunks.add(chunk);
        if (Math.abs(remainingDistance - chunk.getTotalDistance()) > epsilon) {
            // still has more distance to go
            return computeChunks(chunks, chunk.getEndVelocity(),
                    remainingDistance - chunk.getTotalDistance());
        }
        return chunks;
    }

    public double getVelocity(double time) {
        return getMoment(time).getVelocity();
    }

    public double getPosition(double time) {
        return getMoment(time).getPosition();
    }

    private Moment getMoment(double time) {
        double chunkStartTime = 0.0;
        double previousDistance = startingPosition;
        // find the chunk that this time is in and return it
        for (Chunk chunk : chunks) {
            double chunkEndTime = chunkStartTime + chunk.getDuration();
            if (time <= chunkEndTime) {
                return new Moment(chunk, time - chunkStartTime, previousDistance);
            }
            chunkStartTime = chunkEndTime;
            previousDistance += chunk.getTotalDistance();
        }
        // time is past all the chunks, return the end of the last chunk
        Chunk lastChunk = chunks.get(chunks.size() - 1);
        // remove last chunk from accumulations
        chunkStartTime = chunkStartTime - lastChunk.getDuration();
        previousDistance = previousDistance - lastChunk.getTotalDistance();
        return new Moment(lastChunk, time - chunkStartTime, previousDistance);
    }
}
