package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        sleep(1000);
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 39), Math.toRadians(90))
                .build();
        TrajectorySequence trajectory15 = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(() -> {
                    motorLL.setTargetPosition(0);
                    motorRR.setTargetPosition(0);
                })
                .lineToLinearHeading(
                        new Pose2d(26, 36, 0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    servoArm.setPosition(servoPositions.ARM_DOWN);
                })
                .waitSeconds(2)
                .addDisplacementMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                    servoArm.setPosition(servoPositions.ARM_UP);
                })
                //.waitSeconds(2)
                //.splineTo(new Vector2d(0, 39), Math.toRadians(90))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(() -> {
                    motorLL.setTargetPosition(0);
                    motorRR.setTargetPosition(0);
                })
                .lineToConstantHeading(
                        new Vector2d(35, -20),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToConstantHeading(new Vector2d(35, -135))
                .splineToConstantHeading(new Vector2d(10, -135), 270)
                .build();
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory15.end())
                .lineToLinearHeading(
                        new Pose2d(20, 0, 0),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineTo(
                        new Vector2d(0, 0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();

        motorLL.setTargetPosition(5300);
        motorRR.setTargetPosition(5300);
        motorLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectorySequence(trajectory1);

        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        sleep(500);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);

        drive.followTrajectorySequence(trajectory15);
        sleep(2000);
        //drive.followTrajectorySequence(trajectory2);
        telemetry.addData("Time", timer.seconds());
        sleep(1000);
        drive.followTrajectorySequence(trajectory3);
    }
}
