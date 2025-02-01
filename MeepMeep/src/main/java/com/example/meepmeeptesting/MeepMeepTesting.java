package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(20, -40, Math.toRadians(90)))
                        //.splineTo(new Vector2d(-0.5, 37), Math.toRadians(90))
                        .lineTo(new Vector2d(47.5, -3))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(24, -24, Math.toRadians(90)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(72, -42, Math.toRadians(90)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(72, -57, Math.toRadians(90)))
                        .strafeLeft(52)
                        .strafeRight(52)
                        .back(15)
                        .strafeLeft(52)
                        .strafeRight(52)
                        .back(8)
                        .strafeLeft(52)
                        .strafeRight(52)
                        .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}