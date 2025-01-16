package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_TOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class leftAutonomousRoadrunner extends myLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        motorFARM.setTargetPosition(myConstants.ARM_UP);
        servoWrist.setPosition(servoPositions.WRIST_B);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setPower(1.0);
        motorRR.setPower(1.0);
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(-0.5, 37), Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();
        TrajectorySequence trajectory18 = drive.trajectorySequenceBuilder(trajectory1.end())
                .addTemporalMarker(0.75, () -> {
                    motorLL.setTargetPosition(0);
                    motorRR.setTargetPosition(0);
                    servoWrist.setPosition(servoPositions.WRIST_B);
                })
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(57, 1), 0
                )

                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_DOWN);
                })
                .waitSeconds(0.9)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorLL.setTargetPosition(SLIDE_TOP);
                    motorRR.setTargetPosition(SLIDE_TOP);
                })
                .waitSeconds(0.8)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();

        TrajectorySequence trajectory17 = drive.trajectorySequenceBuilder(trajectory1.end())
                .addTemporalMarker(0.75, () -> {
                    motorLL.setTargetPosition(0);
                    motorRR.setTargetPosition(0);
                    servoWrist.setPosition(servoPositions.WRIST_B);
                })
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(57, 15.25), 0
                )

                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_DOWN);
                })
                .waitSeconds(0.9)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorLL.setTargetPosition(SLIDE_TOP);
                    motorRR.setTargetPosition(SLIDE_TOP);
                })
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();
        TrajectorySequence trajectory19 = drive.trajectorySequenceBuilder(trajectory1.end())
                .addTemporalMarker(0.75, () -> {
                    motorLL.setTargetPosition(0);
                    motorRR.setTargetPosition(0);
                    servoWrist.setPosition(servoPositions.WRIST_B);
                })
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(57, 27.125), 0
                )

                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_DOWN);
                })
                .waitSeconds(0.9)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorLL.setTargetPosition(SLIDE_TOP);
                    motorRR.setTargetPosition(SLIDE_TOP);
                })
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();


        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(() -> {
                    motorLL.setTargetPosition(SLIDE_BOTTOM);
                    motorRR.setTargetPosition(SLIDE_BOTTOM);
                })
                .lineToConstantHeading(
                        new Vector2d(35, -20),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToConstantHeading(new Vector2d(35, -135))
                .splineToConstantHeading(new Vector2d(10, -135), 270)
                .build();
        TrajectorySequence trajectory2b = drive.trajectorySequenceBuilder(trajectory1.end())
            .addTemporalMarker(0.75, () -> {
                motorLL.setTargetPosition(0);
                motorRR.setTargetPosition(0);
                servoWrist.setPosition(servoPositions.WRIST_B);
            })
                .back(30)
                .build();
        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();

        motorLL.setTargetPosition(SLIDE_TOP);
        motorRR.setTargetPosition(SLIDE_TOP);
        motorLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        drive.followTrajectorySequence(trajectory1);
        drive.followTrajectorySequence(trajectory18);
        drive.followTrajectorySequence(trajectory17);
        drive.followTrajectorySequence(trajectory19);
        //drive.followTrajectorySequence(trajectory2);
        myConstants.slidePositionSave = motorLL.getCurrentPosition();
        //DONE
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        drive.followTrajectorySequence(trajectory2b);
        sleep(5000);
        //motorFARM.setTargetPosition(servoPositions.ARM_UP);
        //drive.followTrajectorySequence(trajectory3);
    }
}
