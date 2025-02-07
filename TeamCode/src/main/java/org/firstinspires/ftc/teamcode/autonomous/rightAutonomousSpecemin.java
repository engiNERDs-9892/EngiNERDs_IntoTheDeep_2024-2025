package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.myConstants.ARM_UP;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_SPECEMIN_GRAB;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_HIGH_CHAMBER_PLAY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control")
public class rightAutonomousSpecemin extends myLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();//Initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFARM.setTargetPosition(myConstants.ARM_UP);
        motorFARM.setPower(0.6);
        servoWrist.setPosition(servoPositions.WRIST_A);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setPower(1.0);
        motorRR.setPower(1.0);
        //Trajectories
        TrajectorySequence trajectoryPlayPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToConstantHeading(new Vector2d(48.00, -7), 0)//Adjust x offset
                .build();
        TrajectorySequence trajectoryPush = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                //Go from Poles to spike marks
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineTo(new Vector2d(24, -24), Math.toRadians(270)) //Rotate and go out
                .splineToConstantHeading(new Vector2d(72, -42), Math.toRadians(0)) //Go Around the submersable
                .splineToConstantHeading(new Vector2d(72, -52), Math.toRadians(180)) // Go up and around the block
                //Go to wall
                .splineToConstantHeading(new Vector2d(18, -52), Math.toRadians(180)) //Push the block in
                .resetConstraints()
                .build();
        TrajectorySequence trajectoryPlayAfterPush = drive.trajectorySequenceBuilder(trajectoryPush.end())
                //Grab wall
                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB);
                    motorRR.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB);
                })
                .splineToLinearHeading(new Pose2d(10.9, -56, Math.toRadians(180)), Math.toRadians(180))
                .forward(10)
                .addTemporalMarker(()->{
                    servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
                    servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB+600);
                    motorRR.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB+600);
                })
                .waitSeconds(1.0)
                .back(10)
                //Go to chamber
                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_HIGH_CHAMBER);
                    motorRR.setTargetPosition(SLIDE_HIGH_CHAMBER);
                })
                .splineToLinearHeading(new Pose2d(47.25, -1, Math.toRadians(0)), 0)
                .addTemporalMarker(()->{
                    hangSpecemin();
                })
                //.back(10)


                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB);
                    motorRR.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB);
                })
                .splineToLinearHeading(new Pose2d(10.9, -56, Math.toRadians(180)), Math.toRadians(180))
                .forward(10)
                .addTemporalMarker(()->{
                    servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
                    servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB+600);
                    motorRR.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB+600);
                })
                .waitSeconds(1.0)
                .back(10)
                //Go to chamber
                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_HIGH_CHAMBER);
                    motorRR.setTargetPosition(SLIDE_HIGH_CHAMBER);
                })
                .splineToLinearHeading(new Pose2d(47.25, 4, Math.toRadians(0)), 0)
                .addTemporalMarker(()->{
                    hangSpecemin();
                })
                .back(10)
                .build();
        TrajectorySequence trajectoryPushPush = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineTo(new Vector2d(24, -24), Math.toRadians(260)) //Rotate and go out
                .splineToConstantHeading(new Vector2d(72, -42), Math.toRadians(0)) //Go Around the submersable
                .splineToConstantHeading(new Vector2d(72, -52), Math.toRadians(180)) // Go up and around the block
                //Go to wall
                .splineToConstantHeading(new Vector2d(18, -52), Math.toRadians(180)) //Push the block in
                //.resetConstraints()
                .splineToConstantHeading(new Vector2d(72, -60), Math.toRadians(270))//Go back around
                .splineToConstantHeading(new Vector2d(18, -64), Math.toRadians(180))//Push it again
                .strafeRight(54)
                .back(14)
                .strafeLeft(54)
                .strafeRight(54)
                .back(11)
                .strafeLeft(54)
                .build();
        TrajectorySequence trajectoryGrabSpeceminFromPreload = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .back(10)
                .addTemporalMarker(()->{
                    motorLL.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB);
                    motorRR.setTargetPosition(SLIDE_AUTO_SPECEMIN_GRAB);
                })
                .splineToSplineHeading(new Pose2d(11.6, -56, Math.toRadians(180)), Math.toRadians(180))
                .forward(10)
                .build();
        waitForStart();//waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();

        motorFARM.setTargetPosition(ARM_UP);
        motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLL.setTargetPosition(SLIDE_HIGH_CHAMBER);
        motorRR.setTargetPosition(SLIDE_HIGH_CHAMBER);
        motorLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        drive.followTrajectorySequence(trajectoryPlayPreload);
        hangSpecemin();
        drive.followTrajectorySequence(trajectoryPushPush);
        //drive.followTrajectorySequence(trajectoryPlayAfterPush);


        //DONE
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        sleep(5000);
    }

    //Functions
    private void hangSpecemin (){
        motorLL.setPower(0.7);
        motorRR.setPower(0.7);
        motorLL.setTargetPosition(SLIDE_HIGH_CHAMBER_PLAY);
        motorRR.setTargetPosition(SLIDE_HIGH_CHAMBER_PLAY);
        sleep(1500);
        motorLL.setPower(1.0);
        motorRR.setPower(1.0);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_OPEN);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_OPEN);
        sleep(500);
        motorRR.setTargetPosition(SLIDE_BOTTOM);
        motorLL.setTargetPosition(SLIDE_BOTTOM);
    }
}
