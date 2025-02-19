package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.myConstants.FARM_AUTO_FRONT_SPECEMIN;
import static org.firstinspires.ftc.teamcode.myConstants.FARM_UP;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_FRONT_SPECEMIN;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_FRONT_SPECEMIN_PLAY;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_SPECEMIN_GRAB;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control")
public class rightAutonomousSpecemin2 extends myLinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();//Initialization
        drive = new SampleMecanumDrive(hardwareMap);
        //Servos
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_OPEN);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_OPEN);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        servoWrist.setPosition(servoPositions.WRIST_A);
        //Resetting encoders
        motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFARM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //PID stuff
        lift2.setP(0.01);

        PoseStorage.hasRunOpmode = true;

        //Trajectories
        TrajectorySequence trajectoryPlayPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(36, 6), 0)//Adjust x offset
                .build();
        TrajectorySequence trajectoryGrabSpeceminFromPreload = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(11.6, -56))
                .splineToConstantHeading(new Vector2d(0.5, -56), Math.toRadians(180))
                .build();
        TrajectorySequence trajectoryPush = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                .addTemporalMarker(1.0, ()->{
                    lift2.setTarget(SLIDE_BOTTOM);
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

        waitForStart();//waitForStart();
        ElapsedTime timer = new ElapsedTime();


        pidFARM.setSetPoint(FARM_AUTO_FRONT_SPECEMIN);
        pidBARN.setSetPoint(0);
        lift2.setSetPoint(SLIDE_AUTO_FRONT_SPECEMIN);
        pidFARM.setActive(true);
        pidBARN.setActive(true);
        lift2.setActive(true);

        drive.followTrajectorySequenceAsync(trajectoryPlayPreload);
        updateEverything();
        updateEverything(1000);
        lift2.setSetPoint(SLIDE_AUTO_FRONT_SPECEMIN_PLAY);
        updateEverything(1000);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        updateEverything(1000);
        lift2.setSetPoint(SLIDE_BOTTOM);
        pidFARM.setSetPoint(FARM_UP);
        drive.followTrajectorySequenceAsync(trajectoryGrabSpeceminFromPreload);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);

        updateEverything(10000);
        //DONE
        telemetry.setAutoClear(false);
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        telemetry.setAutoClear(true);
        sleep(5000);
    }

    //Functions

    void updateEverything(){
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            lift2.update();
            pidFARM.update();
            pidBARN.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
            telemetry.update();
        }
    }
    void updateEverything(double milliseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            drive.update();
            lift2.update();
            pidFARM.update();
            pidBARN.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
            telemetry.update();
        }
    }
}
