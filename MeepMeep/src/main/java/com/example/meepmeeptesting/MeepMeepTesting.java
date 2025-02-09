package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        final Vector2d basketPosition = new Vector2d(-0.5, 35.25);
        final Vector2d sample1PickupPosition = new Vector2d(57.75, 2.25);
        final Vector2d sample2PickupPosition = new Vector2d(57.75, 16.875);
        final Vector2d sample3PickupPosition = new Vector2d(57.75, 28.625);
        final Pose2d ascentPosition = new Pose2d(80, -5, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 35, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> new DriveWrapper(drive, -30, -65, Math.toRadians(90), 0.70).trajectorySequenceBuilder(new Pose2d(0, Math.toRadians(0)))
                        //Preload
                        .splineTo(basketPosition, Math.toRadians(90))
                        .myMarker()
                        //First
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(sample1PickupPosition, 0)
                        .waitSeconds(0.8)
                        .myMarker()
                        .waitSeconds(0.4)
                        .waitSeconds(0.4)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(basketPosition, Math.toRadians(90))
                        //Second
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(sample2PickupPosition, 0)
                        .waitSeconds(0.9)
                        .waitSeconds(0.4)
                        .myMarker()
                        .waitSeconds(0.4)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(basketPosition, Math.toRadians(90))
                        //Third
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(sample3PickupPosition, 0)
                        .waitSeconds(.9)
                        .myMarker()
                        .waitSeconds(0.4)
                        .waitSeconds(0.4)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(basketPosition, Math.toRadians(90))
                        //Park
                        .setReversed(true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
                );

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 25, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        new DriveWrapper(drive, 5, -65, Math.toRadians(90), 0.70)
                                .trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                //Play Preload
                                .splineToConstantHeading(new Vector2d(48.10, -7), 0)
                                //Block
                                .setReversed(true)
                                .splineTo(new Vector2d(24, -24), Math.toRadians(270)) //Rotate and go out
                                .splineToConstantHeading(new Vector2d(72, -42), Math.toRadians(0)) //Go Around the submersable
                                .splineToConstantHeading(new Vector2d(72, -55), Math.toRadians(180)) // Go up and around the block
                                //Go to wall
                                .splineToConstantHeading(new Vector2d(68, -55), Math.toRadians(180)) //Finish pushing the block in
                                .strafeLeft(49)
                                .resetConstraints()
                                //Play
                                .setTangent(Math.toRadians(90))
                                .splineTo(new Vector2d(.5, -56), Math.toRadians(180))
                                .waitSeconds(0.3)
                                .waitSeconds(0.8)
                                //Go to chamber
                                //.back(40)
                                //.splineToLinearHeading(new Pose2d(47.25, -1, Math.toRadians(0)), 0)
                                //.setReversed(false)
                                //.splineTo(new Vector2d(47.25, -1), Math.toRadians(0))
                                .back(10)
                                .splineToSplineHeading(new Pose2d(47.25, -1, -0.0001), Math.toRadians(0))
                                //Play
                                .setReversed(true)
                                //.splineToLinearHeading(new Pose2d(0.5, -56, Math.toRadians(180)), Math.toRadians(180))
                                .back(40)
                                //.splineToLinearHeading(new Pose2d(47.25, -1, Math.toRadians(0)), 0)
                                .setReversed(false)
                                .splineTo(new Vector2d(0.5, -56), Math.toRadians(180))

                                .waitSeconds(0.3)
                                .waitSeconds(0.8)
                                .back(10)
                                //Go to chamber
                                .setTangent(90)
                                .splineToSplineHeading(new Pose2d(47.25, 4, Math.toRadians(0)), 0)
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(myBot)
                .addEntity(myBot2)
                .exportTrajectoryImage("C:\\Users\\tovs\\Documents\\MeepMeep.png")
                .start();
    }

}