package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.myConstants.FARM_UP;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_SPECEMIN_GRAB;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_HIGH_CHAMBER_PLAY;
import static org.firstinspires.ftc.teamcode.opModeGroups.DEFAULT_TELEOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.VariableStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.motorsController;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.opModeGroups;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(group = opModeGroups.auto.ADVANCED, preselectTeleOp = DEFAULT_TELEOP)
public class rightAutonomousSpeceminPID extends myLinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();//Initialization
        drive = new SampleMecanumDrive(hardwareMap);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VariableStorage.hasRunOpmode = true;
        motorFARM.setTargetPosition(myConstants.FARM_UP);
        motorFARM.setPower(0.6);
        servoWrist.setPosition(servoPositions.WRIST_A);

        lift.setPID(.01, motorsController.I, motorsController.D);
        //Trajectories
        TrajectorySequence trajectoryPlayPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.FARM_UP);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToConstantHeading(new Vector2d(47.75, 6), 0)//Adjust x offset
                .build();
        TrajectorySequence trajectoryPush = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                .addTemporalMarker(1.0, ()->{
                    lift.setTarget(SLIDE_BOTTOM);
                })
                //Go from Poles to spike marks
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(20*3))
                .splineTo(new Vector2d(24, -24), Math.toRadians(270)) //Rotate and go out
                .splineToConstantHeading(new Vector2d(72, -42), Math.toRadians(0)) //Go Around the submersable
                .splineToConstantHeading(new Vector2d(72, -55), Math.toRadians(180)) // Go up and around the block
                //Go to wall
                .splineToConstantHeading(new Vector2d(68, -55), Math.toRadians(180)) //Finish pushing the block in
                .setReversed(false)
                .strafeLeft(52)
                .resetConstraints()
                .build();
        TrajectorySequence trajectoryPlayAfterPush = drive.trajectorySequenceBuilder(trajectoryPush.end())
                //Grab wall
                .addTemporalMarker(1.0, ()->{
                    lift.setTarget(SLIDE_AUTO_SPECEMIN_GRAB);
                })

                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(1.0, -38), Math.toRadians(180))
                .addTemporalMarker(()->{
                    servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
                    servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
                })
                .waitSeconds(0.3)
                //Go to chamber5
                .addTemporalMarker(()->{
                    lift.setTarget(SLIDE_HIGH_CHAMBER);
                })
                .splineToLinearHeading(new Pose2d(48.5, 3, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence trajectoryPushPush = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(20*3))
                .splineTo(new Vector2d(24, -24), Math.toRadians(270)) //Rotate and go out
                .splineToConstantHeading(new Vector2d(72, -42), Math.toRadians(0)) //Go Around the submersable
                .splineToConstantHeading(new Vector2d(72, -53.5), Math.toRadians(180))// Go up and around the block
                //Go to wall
                .splineToConstantHeading(new Vector2d(68, -53.5), Math.toRadians(180))//Start Block
                .strafeLeft(49)//Finish pushing the block in
                //Second block
                .splineToConstantHeading(new Vector2d(72, -56), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(68, -69), Math.toRadians(180))//Start Block
                .strafeLeft(52)//Finish pushing the block in
                //Third block
                .splineToConstantHeading(new Vector2d(77, -80), Math.toRadians(270))
                .strafeLeft(60)//Pushing the block in all the way
                .build();
        TrajectorySequence trajectoryPlayAfterPushPush = drive.trajectorySequenceBuilder(trajectoryPushPush.end())
                //Grab wall
                .addTemporalMarker(()->{
                    lift.setTarget(SLIDE_AUTO_SPECEMIN_GRAB);
                })

                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(.6, -38), Math.toRadians(180))
                .addTemporalMarker(()->{
                    servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
                    servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
                })
                .waitSeconds(0.3)
                //Go to chamber
                .addTemporalMarker(()->{
                    lift.setTarget(SLIDE_HIGH_CHAMBER);
                })
                .lineToLinearHeading(new Pose2d(48, 1, 0))
                .build();
        TrajectorySequence trajectoryPlay2AfterPush = drive.trajectorySequenceBuilder(trajectoryPlayAfterPush.end())
                .addTemporalMarker(1.0, ()->{
                    lift.setTarget(SLIDE_AUTO_SPECEMIN_GRAB);
                })
                //Go to grab
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(0.1, -36, Math.toRadians(180)), Math.toRadians(180))
                //Grab
                .addTemporalMarker(()->{
                    servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
                    servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
                })
                .waitSeconds(0.3)
                //Go to chamber
                .addTemporalMarker(()->{
                    lift.setTarget(SLIDE_HIGH_CHAMBER);
                })
                .splineToLinearHeading(new Pose2d(49.5-.25, -3, 0), 0)
                .build();
        TrajectorySequence trajectoryGrabSpeceminFromPreload = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .back(10)
                .addTemporalMarker(()->{
                    lift.setTarget(SLIDE_AUTO_SPECEMIN_GRAB);
                })
                .splineToSplineHeading(new Pose2d(11.6, -56, Math.toRadians(180)), Math.toRadians(180))
                .forward(10)
                .build();
        TrajectorySequence trajectoryPark = drive.trajectorySequenceBuilder(trajectoryPlay2AfterPush.end())
                .addTemporalMarker(1.0, ()->{
                    lift.setTarget(SLIDE_BOTTOM);
                })
                .lineTo(new Vector2d(1, -55))
                .build();
        waitForStart();//waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();

        motorFARM.setTargetPosition(FARM_UP);
        motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setActive(true);
        lift.setTarget(SLIDE_HIGH_CHAMBER);

        drive.followTrajectorySequenceAsync(trajectoryPlayPreload);
        updateEverything();
        hangSpecemin();
        sleep(500);
        drive.followTrajectorySequenceAsync(trajectoryPush);
        updateEverything();
        drive.followTrajectorySequenceAsync(trajectoryPlayAfterPush);
        updateEverything();
        hangSpecemin(-55);
        drive.followTrajectorySequenceAsync(trajectoryPlay2AfterPush);
        updateEverything();
        hangSpecemin();
        drive.followTrajectorySequenceAsync(trajectoryPark);
        updateEverything();
        //DONE
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        sleep(5000);
    }

    //Functions
    private void hangSpecemin (){
        lift.setTarget(SLIDE_HIGH_CHAMBER_PLAY);
        updateEverything(700);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_OPEN);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_OPEN);
    }
    private void hangSpecemin (double hangOffset){
        lift.setTarget(SLIDE_HIGH_CHAMBER_PLAY+hangOffset);
        updateEverything(700);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_OPEN);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_OPEN);
    }
    void updateEverything(){
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            lift.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Continually write pose to `PoseStorage`
            VariableStorage.currentPose = poseEstimate;
            telemetry.update();
        }
    }
    void updateEverything(double milliseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            drive.update();
            lift.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Continually write pose to `PoseStorage`
            VariableStorage.currentPose = poseEstimate;
            telemetry.update();
        }
    }
}
