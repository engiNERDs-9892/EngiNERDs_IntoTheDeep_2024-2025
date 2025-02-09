package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

class DriveWrapper {
    class TrajectorySequenceBuilderWrapper {
        TrajectorySequenceBuilder innerBuilder;
        double offsetX;
        double offsetY;
        double rotate;
        double scale;

        private Pose2d transformPose2d(Pose2d pose) {
            Vector2d rotated = pose.vec().rotated(rotate).times(scale);
            return new Pose2d(rotated.getX() + offsetX, rotated.getY() + offsetY, pose.getHeading() + rotate);
        }

        private Vector2d transformVector2d(Vector2d vec) {
            Vector2d rotated = vec.rotated(rotate).times(scale);
            return new Vector2d(rotated.getX() + offsetX, rotated.getY() + offsetY);
        }

        private double transformDistance(double distance) {
            return distance * scale;
        }

        private double transformAngle(double angle) {
            return angle + rotate;
        }

        TrajectorySequenceBuilderWrapper(Pose2d startPose, DriveShim innerDriveShim, double offsetX, double offestY, double rotate, double scale) {
            this.offsetX = offsetX;
            this.offsetY = offestY;
            this.rotate = rotate;
            this.scale = scale;
            this.innerBuilder = innerDriveShim.trajectorySequenceBuilder(transformPose2d(startPose));
        }

        TrajectorySequenceBuilderWrapper setReversed(Boolean reversed) {
            innerBuilder.setReversed(reversed);
            return this;
        }

        TrajectorySequenceBuilderWrapper setTangent(double tangent) {
            innerBuilder.setTangent(transformAngle(tangent));
            return this;
        }

        TrajectorySequenceBuilderWrapper resetConstraints() {
            innerBuilder.resetConstraints();
            return this;
        }//constraints and markers
        TrajectorySequenceBuilderWrapper myMarker(){
            innerBuilder.addTemporalMarker(()->{});
            return this;
        }

        TrajectorySequenceBuilderWrapper waitSeconds(double seconds) {
            innerBuilder.waitSeconds(seconds);
            return this;
        }

        TrajectorySequenceBuilderWrapper splineTo(Vector2d endPosition, double endTangent) {
            innerBuilder.splineTo(transformVector2d(endPosition), transformAngle(endTangent));
            return this;
        }

        TrajectorySequenceBuilderWrapper splineToLinearHeading(Pose2d endPose, double endTangent) {
            innerBuilder.splineToLinearHeading(transformPose2d(endPose), transformAngle(endTangent));
            return this;
        }

        TrajectorySequenceBuilderWrapper splineToConstantHeading(Vector2d endPosition, double endTangent) {
            innerBuilder.splineToConstantHeading(transformVector2d(endPosition), transformAngle(endTangent));
            return this;
        }

        TrajectorySequenceBuilderWrapper splineToSplineHeading(Pose2d endPose, double endTangent) {
            innerBuilder.splineToSplineHeading(transformPose2d(endPose), transformAngle(endTangent));
            return this;
        }

        TrajectorySequenceBuilderWrapper lineTo(Vector2d endPosition) {
            innerBuilder.lineTo(transformVector2d(endPosition));
            return this;
        }

        TrajectorySequenceBuilderWrapper lineToLinearHeading(Pose2d endPose) {
            innerBuilder.lineToLinearHeading(transformPose2d(endPose));
            return this;
        }

        TrajectorySequenceBuilderWrapper lineToConstantHeading(Vector2d endPosition) {
            innerBuilder.lineToConstantHeading(transformVector2d(endPosition));
            return this;
        }

        TrajectorySequenceBuilderWrapper lineToSplineHeading(Pose2d endPose) {
            innerBuilder.lineToSplineHeading(transformPose2d(endPose));
            return this;
        }

        TrajectorySequenceBuilderWrapper strafeTo(Vector2d endPosition) {
            innerBuilder.strafeTo(transformVector2d(endPosition));
            return this;
        }

        TrajectorySequenceBuilderWrapper forward(double distance) {
            innerBuilder.forward(transformDistance(distance));
            return this;
        }

        TrajectorySequenceBuilderWrapper back(double distance) {
            innerBuilder.back(transformDistance(distance));
            return this;
        }

        TrajectorySequenceBuilderWrapper strafeLeft(double distance) {
            innerBuilder.strafeLeft(transformDistance(distance));
            return this;
        }

        TrajectorySequenceBuilderWrapper strafeRight(double distance) {
            innerBuilder.strafeRight(transformDistance(distance));
            return this;
        }

        TrajectorySequenceBuilderWrapper turn(double angle) {
            innerBuilder.turn(angle);
            return this;
        }

        TrajectorySequence build() {
            return innerBuilder.build();
        }

    }

    DriveShim innerDriveShim;
    double offsetX;
    double offsetY;
    double rotate;
    double scale;

    DriveWrapper(DriveShim drive, double offsetX, double offsetY, double rotate, double scale) {
        this.innerDriveShim = drive;
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.rotate = rotate;
        this.scale = scale;
    }

    TrajectorySequenceBuilderWrapper trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilderWrapper(startPose, innerDriveShim, offsetX, offsetY, rotate, scale);
    }

}
