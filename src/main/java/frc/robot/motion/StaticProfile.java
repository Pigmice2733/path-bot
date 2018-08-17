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

    private ArrayList<Chunk> computeChunks(ArrayList<Chunk> chunks, double currentVelocity, double remainingDistance) {
        final Chunk chunk;

        double stoppingDistance = 0.5 * (Math.abs(currentVelocity) / maxDecel) * currentVelocity;
        double targetDirection = Math.signum(remainingDistance);
        double currentDirection = Math.signum(currentVelocity);

        /***
         * If going in the wrong direction and at start of profile: transition to
         * stopped
         *
         * Else if stopping distance > remaining distance and at start of profile:
         * transition to stopped
         *
         * Else if going faster than max speed: transition to max speed
         *
         * Else if going slower than max speed:
         * --- If stopping distance < remaining distance: transition to max speed
         * --- Else: transition to stopped
         *
         * Otherwise, must be going at max speed
         * --- If stopping distance < remaining distance: max velocity transition
         *
         * --- Else if stopping distance == remaining distance: transition to stopped
         *
         * --- Else, goes to far - triangular profile:
         * --- --- Rewrite profile into triangle
         * --- --- Target distance has been reached, return all chunks
         ***/

        // If going in the wrong direction and at start of profile
        if (currentDirection != targetDirection && currentVelocity != 0 && chunks.size() == 0) {
            // transition to stopped
            chunk = Chunk.createVelocityTransition(currentVelocity, 0, maxAccel, maxDecel);
        }
        // Else if stopping distance > remaining distance
        else if (Math.abs(stoppingDistance) > Math.abs(remainingDistance) && chunks.size() == 0) {
            // transition to stopped
            chunk = Chunk.createVelocityTransition(currentVelocity, 0, maxAccel, maxDecel);
        }
        // Else if going faster than max speed
        else if (Math.abs(currentVelocity) > maxVelocity) {
            // transition to max speed
            chunk = Chunk.createVelocityTransition(currentVelocity, maxVelocity * targetDirection, maxAccel, maxDecel);
        }
        // Else if going slower than max speed
        else if (Math.abs(currentVelocity) < maxVelocity) {
            // If stopping distance < remaining distance
            if (Math.abs(stoppingDistance) < Math.abs(remainingDistance)) {
                // transition to max speed
                chunk = Chunk.createVelocityTransition(currentVelocity, maxVelocity * targetDirection, maxAccel,
                        maxDecel);
            } else {
                // transiton to stopped
                chunk = Chunk.createVelocityTransition(currentVelocity, 0, maxAccel, maxDecel);
            }
        }
        // Otherwise, must be going at max speed
        else {
            // If stopping distance < remaining distance
            if (Math.abs(stoppingDistance) < Math.abs(remainingDistance)) {
                // max velocity transition - make up difference
                chunk = Chunk.createConstantVelocity(maxVelocity * targetDirection,
                        remainingDistance - stoppingDistance);
            }
            // Else if stopping distance == remaining distance
            else if (Math.abs(stoppingDistance - remainingDistance) < epsilon) {
                // transition to stopped
                chunk = Chunk.createVelocityTransition(maxVelocity * targetDirection, 0, maxAccel, maxDecel);
            }
            // Else, goes to far - triangular profile
            else {
                // Remove previous chunk
                remainingDistance += chunks.get(chunks.size() - 1).getTotalDistance();
                currentVelocity = chunks.get(chunks.size() - 1).getVelocity(0.0);
                stoppingDistance = 0.5 * (currentVelocity / maxDecel) * currentVelocity;
                targetDirection = Math.signum(remainingDistance);
                currentDirection = Math.signum(currentVelocity);
                chunks.remove(chunks.size() - 1);

                // Account for non-zero velocities going into triangular section of profile
                double precedingTriangleArea = 0.5 * (currentVelocity * currentVelocity) / maxAccel;
                double fullTriangleDistance = Math.abs(remainingDistance + precedingTriangleArea);

                // Calculate ratio of accel distance to full distance of triangular profile
                double fullAccelerationDistance = 0.5 * maxVelocity * (maxVelocity / maxAccel);
                double fullDecelerationDistance = 0.5 * maxVelocity * (maxVelocity / maxDecel);
                double ratio = fullAccelerationDistance / (fullAccelerationDistance + fullDecelerationDistance);

                double accelerationDistance = ratio * fullTriangleDistance;

                double triangleMaxSpeed = Math.sqrt(2 * accelerationDistance * maxAccel) * targetDirection;

                chunks.add(Chunk.createVelocityTransition(currentVelocity, triangleMaxSpeed, maxAccel, maxDecel));
                chunks.add(Chunk.createVelocityTransition(triangleMaxSpeed, 0, maxAccel, maxDecel));

                // Target distance has been reached, return all chunks
                return chunks;
            }
        }

        chunks.add(chunk);
        if (Math.abs(remainingDistance - chunk.getTotalDistance()) > epsilon) {
            // still have farther to go
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
