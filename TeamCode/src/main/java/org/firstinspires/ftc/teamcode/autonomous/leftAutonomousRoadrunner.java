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
        servoArm.setPosition(servoPositions.ARM_UP);
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
        TrajectorySequence trajectory15 = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(() -> {
                    motorLL.setTargetPosition(0);
                    motorRR.setTargetPosition(0);
                })
                .lineToLinearHeading(
                        new Pose2d(31, 38, 0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(()->{
                    servoArm.setPosition(servoPositions.ARM_DOWN);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    servoArm.setPosition(servoPositions.ARM_UP);
                    motorLL.setTargetPosition(SLIDE_TOP);
                    motorRR.setTargetPosition(SLIDE_TOP);
                })
                .setReversed(true)
                .splineTo(new Vector2d(5, 19), Math.toRadians(270))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();

        TrajectorySequence trajectory16 = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(() -> {
                    motorLL.setTargetPosition(0);
                    motorRR.setTargetPosition(0);
                })
                .lineToLinearHeading(
                        new Pose2d(31, 22, 0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(()->{
                    servoArm.setPosition(servoPositions.ARM_DOWN);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    servoArm.setPosition(servoPositions.ARM_UP);
                    motorLL.setTargetPosition(SLIDE_TOP);
                    motorRR.setTargetPosition(SLIDE_TOP);
                })
                .setReversed(true)
                .splineTo(new Vector2d(5, 19), Math.toRadians(270))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))
                .waitSeconds(2)
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
        TrajectorySequence trajectory2B = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(() -> {
                    motorLL.setTargetPosition(1600);
                    motorRR.setTargetPosition(1600);
                })
                .lineToLinearHeading(
                        new Pose2d(80, 20, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .splineToConstantHeading(new Vector2d(80, -5), 270)
                .addTemporalMarker(()->{
                    servoArm.setPosition(0.45);
                })
                .build();
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory15.end())
                .back(20)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(
                        new Pose2d(20, 0, 0),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_BOTTOM);
                    motorRR.setTargetPosition(SLIDE_BOTTOM);
                    servoArm.setPosition(servoPositions.ARM_DOWN);
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .lineTo(
                        new Vector2d(0, 0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .waitSeconds(0.5)
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
        sleep(500);
        drive.followTrajectorySequence(trajectory16);
        sleep(500);
        drive.followTrajectorySequence(trajectory2B);
        myConstants.slidePositionSave = motorLL.getCurrentPosition();
        //DONE
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        servoArm.getController().pwmDisable();
        //sleep(5000);
        //servoArm.setPosition(servoPositions.ARM_UP);
        //drive.followTrajectorySequence(trajectory3);
    }
}
