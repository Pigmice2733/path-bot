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

    public StaticProfile(double currentVelocity, double currentPosition, double targetDistance, double maxVelocity,
            double maxAccel, double maxDecel) {
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

    private ArrayList<Chunk> computeChunks(ArrayList<Chunk> chunks, double startVelocity, double remainingDistance) {
        Chunk chunk;
        // going in the wrong direction
        if (Math.signum(startVelocity) != Math.signum(remainingDistance) && startVelocity != 0 && chunks.size() == 0) {
            // transition to 0
            chunk = Chunk.createVelocityTransition(startVelocity, 0, maxAccel, maxDecel);
            System.out.println("case 1");
        }
        // not going at max speed
        else if (Math.abs(Math.abs(startVelocity) - maxVelocity) > epsilon) {
            // transition to max speed
            chunk = Chunk.createVelocityTransition(startVelocity, maxVelocity * Math.signum(remainingDistance),
                    maxAccel, maxDecel);
            System.out.println("case 2");
            // going at max speed
        } else {
            System.out.println("case 3");
            Chunk decelChunk = Chunk.createVelocityTransition(startVelocity, 0, maxAccel, maxDecel);
            // not going far enough, need constant middle chunk
            if (Math.abs(decelChunk.getTotalDistance()) < Math.abs(remainingDistance)) {
                System.out.println("not far enough, constant velocity");
                chunk = Chunk.createConstantVelocity(maxVelocity * Math.signum(remainingDistance),
                        remainingDistance - decelChunk.getTotalDistance());
            } else {
                // normal decel
                chunk = decelChunk;
                // goes too far, switch to triangle
                if (Math.abs(decelChunk.getTotalDistance()) > Math.abs(remainingDistance)) {
                    System.out.println("Too far, triangle");
                    remainingDistance += chunks.get(chunks.size() - 1).getTotalDistance();
                    startVelocity = chunks.get(chunks.size() - 1).getVelocity(0.0);
                    chunks.remove(chunks.size() - 1);

                    double beforeQuadrilateralDistance = 0.5 * (startVelocity * startVelocity) / maxAccel;
                    double fullTriangleDistance = Math.abs(remainingDistance + beforeQuadrilateralDistance);

                    double fullAccelerationTime = maxVelocity / maxAccel;
                    double decelerationTime = maxVelocity / maxDecel;
                    double timeRatio = fullAccelerationTime / (fullAccelerationTime + decelerationTime);

                    double accelerationDistance = timeRatio * fullTriangleDistance;
                    double accelerationTime = Math.sqrt((2 * accelerationDistance) / maxAccel);

                    double triangleMaxSpeed = accelerationTime * maxAccel * Math.signum(remainingDistance);

                    chunks.add(Chunk.createVelocityTransition(startVelocity, triangleMaxSpeed, maxAccel, maxDecel));
                    chunks.add(Chunk.createVelocityTransition(triangleMaxSpeed, 0, maxAccel, maxDecel));

                    return chunks;
                }
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
            return computeChunks(chunks, chunk.getEndVelocity(), remainingDistance - chunk.getTotalDistance());
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
