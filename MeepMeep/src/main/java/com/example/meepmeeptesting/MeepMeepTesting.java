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
    private static class drivewrapper{
        private class TrajectorySequenceBuilderWrapper {
            TrajectorySequenceBuilder innerBuilder;
            double offsetX;
            double offsetY;
            double rotate;
            private Pose2d rotatePose2d(Pose2d pose, double angle){
                Vector2d rotated = pose.vec().rotated(angle);
                return new Pose2d(rotated.getX(), rotated.getY(), pose.getHeading()+angle);
            }
            TrajectorySequenceBuilderWrapper(Pose2d startPose, DriveShim innerDriveShim, double offsetX, double offestY, double rotate){
                this.innerBuilder = innerDriveShim.trajectorySequenceBuilder(rotatePose2d(startPose, 0));
                this.offsetX = offsetX;
                this.offsetY = offestY;
                this.rotate = rotate;
            }
            TrajectorySequenceBuilderWrapper setReversed(Boolean reversed){
                innerBuilder.setReversed(reversed);
                return this;
            }
            TrajectorySequenceBuilderWrapper setTangent(double tangent){
                innerBuilder.setTangent(tangent);
                return this;
            }
            TrajectorySequenceBuilderWrapper resetConstraints() {
                innerBuilder.resetConstraints();
                return this;
            }
            TrajectorySequenceBuilderWrapper splineTo(Vector2d endPosition, double endTangent){
                innerBuilder.splineTo(endPosition, endTangent);
                return this;
            }
            TrajectorySequenceBuilderWrapper splineToConstantHeading(Vector2d endPosition, double endTangent) {
                innerBuilder.splineToConstantHeading(endPosition, endTangent);
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
        drivewrapper(DriveShim drive, double offsetX, double offsetY, double rotate){
            this.innerDriveShim = drive;
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.rotate = rotate;
        }
        TrajectorySequenceBuilderWrapper trajectorySequenceBuilder(Pose2d startPose){
            TrajectorySequenceBuilderWrapper builderWrapper = new TrajectorySequenceBuilderWrapper(startPose, innerDriveShim, offsetX, offsetY, rotate);

            return builderWrapper;
        }

    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
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
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 25, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> (new drivewrapper(drive, 0, 0, 0)).trajectorySequenceBuilder(new Pose2d(47.5-30, -8-6, Math.toRadians(0)))
                        .setReversed(true)
                        //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        //.setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                        .splineTo(new Vector2d(24-30, -24-6), Math.toRadians(260)) //Rotate and go out
                        .splineToConstantHeading(new Vector2d(72-30, -42-6), Math.toRadians(0)) //Go Around the submersable
                        .splineToConstantHeading(new Vector2d(72-30, -52-6), Math.toRadians(180)) // Go up and around the block
                        //Go to wall
                        .splineToConstantHeading(new Vector2d(18-30, -52-6), Math.toRadians(180)) //Push the block in
                        .resetConstraints()
                        .splineToConstantHeading(new Vector2d(72-30, -52-6), Math.toRadians(180))
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