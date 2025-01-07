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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .splineTo(new Vector2d(-0.5, 37), Math.toRadians(90))
                        .addTemporalMarker(0.75, () -> {
                            //motorLL.setTargetPosition(0);
                            //motorRR.setTargetPosition(0);
                        })
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(
                                new Vector2d(57, 32), 0
                        )
                        .addTemporalMarker(()->{
                            //servoArm.setPosition(servoPositions.ARM_DOWN);
                        })
                        .waitSeconds(2.0)
                        .addTemporalMarker(()->{
                            //7servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                            //s/ervoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                        })
                        .waitSeconds(2.0)
                        .addTemporalMarker(()->{
                            //servoArm.setPosition(servoPositions.ARM_UP);
                            //m//otorLL.setTargetPosition(SLIDE_TOP);
                            //motorRR.setTargetPosition(SLIDE_TOP);
                        })
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))
                        .waitSeconds(2)
                        .addTemporalMarker(()->{
                            //servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                            //servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                        })
                        .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}