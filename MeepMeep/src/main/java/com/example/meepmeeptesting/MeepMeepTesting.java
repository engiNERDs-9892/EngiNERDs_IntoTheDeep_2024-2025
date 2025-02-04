package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 35, Math.toRadians(180), Math.toRadians(180), 15)
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



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}