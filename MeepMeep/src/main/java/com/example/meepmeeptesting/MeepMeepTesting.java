package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {
    private static class DriveWrapper {
        private class TrajectorySequenceBuilderWrapper {
            TrajectorySequenceBuilder innerBuilder;
            double offsetX;
            double offsetY;
            double rotate;
            double scale;

            private Pose2d transformPose2d(Pose2d pose){
                Vector2d rotated = pose.vec().rotated(rotate).times(scale);
                return new Pose2d(rotated.getX()+offsetX, rotated.getY()+offsetY, pose.getHeading()+rotate);
            }
            private Vector2d transformVector2d(Vector2d vec){
                Vector2d rotated = vec.rotated(rotate).times(scale);
                return new Vector2d(rotated.getX()+offsetX, rotated.getY()+offsetY);
            }
            private double transformDistance(double distance){
                return distance*scale;
            }
            private double transformAngle(double angle){
                return angle+rotate;
            }
            TrajectorySequenceBuilderWrapper(Pose2d startPose, DriveShim innerDriveShim, double offsetX, double offestY, double rotate, double scale){
                this.offsetX = offsetX;
                this.offsetY = offestY;
                this.rotate = rotate;
                this.scale = scale;
                this.innerBuilder = innerDriveShim.trajectorySequenceBuilder(transformPose2d(startPose));
            }
            TrajectorySequenceBuilderWrapper setReversed(Boolean reversed){
                innerBuilder.setReversed(reversed);
                return this;
            }
            TrajectorySequenceBuilderWrapper setTangent(double tangent){
                innerBuilder.setTangent(transformAngle(tangent));
                return this;
            }
            TrajectorySequenceBuilderWrapper resetConstraints() {
                innerBuilder.resetConstraints();
                return this;
            }//constraints and markers
            TrajectorySequenceBuilderWrapper waitSeconds(double seconds) {
                innerBuilder.waitSeconds(seconds);
                return this;
            }
            TrajectorySequenceBuilderWrapper splineTo(Vector2d endPosition, double endTangent){
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
            TrajectorySequenceBuilderWrapper forward(double distance){
                innerBuilder.forward(transformDistance(distance));
                return this;
            }
            TrajectorySequenceBuilderWrapper back(double distance){
                innerBuilder.back(transformDistance(distance));
                return this;
            }
            TrajectorySequenceBuilderWrapper strafeLeft(double distance){
                innerBuilder.strafeLeft(transformDistance(distance));
                return this;
            }
            TrajectorySequenceBuilderWrapper strafeRight(double distance){
                innerBuilder.strafeRight(transformDistance(distance));
                return this;
            }
            TrajectorySequenceBuilderWrapper turn(double angle){
                innerBuilder.turn(angle);
                return this;
            }

            TrajectorySequence build(){
                return innerBuilder.build();
            }

        }
        DriveShim innerDriveShim;
        double offsetX;
        double offsetY;
        double rotate;
        double scale;
        DriveWrapper(DriveShim drive, double offsetX, double offsetY, double rotate, double scale){
            this.innerDriveShim = drive;
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.rotate = rotate;
            this.scale = scale;
        }
        TrajectorySequenceBuilderWrapper trajectorySequenceBuilder(Pose2d startPose){
            return new TrajectorySequenceBuilderWrapper(startPose, innerDriveShim, offsetX, offsetY, rotate, scale);
        }

    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 25, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(47.5-30, -3+20, Math.toRadians(0)))
                        .setReversed(true)
                        //.splineToLinearHeading(new Pose2d(24-30, -24+20, Math.toRadians(90)), Math.toRadians(270))
                        .splineTo(new Vector2d(24-30, -24+20), Math.toRadians(270))
                        //.splineToSplineHeading(new Pose2d(24-30, -28+20, Math.toRadians(90)), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(72-30, -42+20), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(72-30, -57+20), Math.toRadians(180))
                        //.strafeLeft(52)
                        .splineToConstantHeading(new Vector2d(18-30, -57+20), Math.toRadians(180))

                        .build()
                );
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 25, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        new DriveWrapper(drive, 5, -65, Math.toRadians(90), 0.80)
                        .trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        //
                        .splineToConstantHeading(new Vector2d(47.5, -8), 0)
                        .setReversed(true)
                        //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        //.setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                        .splineTo(new Vector2d(24, -24), Math.toRadians(260)) //Rotate and go out
                        .splineToConstantHeading(new Vector2d(72, -42), Math.toRadians(0)) //Go Around the submersable
                        .splineToConstantHeading(new Vector2d(72, -52), Math.toRadians(180)) // Go up and around the block
                        //Go to wall
                        .splineToConstantHeading(new Vector2d(18, -52), Math.toRadians(180)) //Push the block in
                        .resetConstraints()
                        .splineToConstantHeading(new Vector2d(72, -52), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(72, -64), Math.toRadians(180))
                                //.splineToConstantHeading(new Vector2d(72, -64), Math.toRadians(180))
                        .build()
                );




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }

}