package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.myConstants.BARN_DOWN;
import static org.firstinspires.ftc.teamcode.myConstants.BARN_GRAB;
import static org.firstinspires.ftc.teamcode.myConstants.BARN_HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.myConstants.BARN_RADIANS_PER_TICK;
import static org.firstinspires.ftc.teamcode.myConstants.BARN_UP;
import static org.firstinspires.ftc.teamcode.myConstants.FARM_AUTO_FRONT_SPECEMIN;
import static org.firstinspires.ftc.teamcode.myConstants.FARM_UP;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_FRONT_SPECEMIN;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_FRONT_SPECEMIN_PLAY;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.opModeGroups.DEFAULT_TELEOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.VariableStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.motorsController;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opModeGroups;

@Autonomous(group = opModeGroups.auto.ADVANCED, preselectTeleOp = DEFAULT_TELEOP, name = "Right Auto (Specimen)")
public class rightAutonomousSpecemin2 extends myLinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();//Initialization
        drive = new SampleMecanumDrive(hardwareMap, 0.3);
        //Servos
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        servoWrist.setPosition(servoPositions.WRIST_A);
        //Resetting encoders
        motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFARM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //PID stuff
        lift2.setPID(0.003, motorsController.I, motorsController.D);
        pidBARN.setZeroSwitch(sensorBARN);
        //pidBARN.setArmFactors(0.02, 0.004408f);
        VariableStorage.hasRunOpmode = true;

        //Trajectories
        TrajectorySequence trajectoryPlayPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(38, 6))
                .build();
        TrajectorySequence trajectoryGrabSpeceminFromPreload = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(11.6, -28))
                .splineToConstantHeading(new Vector2d(0.5, -30), Math.toRadians(180))
                .build();
        TrajectorySequence trajectoryPlaySpecemin2 = drive.trajectorySequenceBuilder(trajectoryGrabSpeceminFromPreload.end())
                .lineToConstantHeading(new Vector2d(12, -10))
                .splineToConstantHeading(new Vector2d(47, 8), 0)
                .build();
        TrajectorySequence trajectoryPlaySpecemin2_2 = drive.trajectorySequenceBuilder(trajectoryPlaySpecemin2.end())
                .back(4)
                .addTemporalMarker(this::openClaw2)
                .back(10)
                .addTemporalMarker(() -> pidBARN.setTarget(BARN_UP))
                .splineToConstantHeading(new Vector2d(0.5, -30), Math.toRadians(180))

                .build();

        TrajectorySequence trajectoryPlaySpecemin3 = drive.trajectorySequenceBuilder(trajectoryGrabSpeceminFromPreload.end())
                .lineToConstantHeading(new Vector2d(12, -10))
                .splineToConstantHeading(new Vector2d(47, 8+3), 0)
                .build();
        TrajectorySequence trajectoryPlaySpecemin3_2 = drive.trajectorySequenceBuilder(trajectoryPlaySpecemin3.end())
                .back(4)
                .addTemporalMarker(this::openClaw2)
                .back(10)
                .build();

        TrajectorySequence trajectoryPush = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                //Go from Poles to spike marks
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                //.setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(24, -24), Math.toRadians(270)) //go out
                //.splineToConstantHeading(new Vector2d(76, -47), Math.toRadians(270)) //Go Around the submersable
                .splineToConstantHeading(new Vector2d(79, -60), Math.toRadians(280)) // Go up and around the block
                //Go to wall
                //.splineToConstantHeading(new Vector2d(68+4, -59), Math.toRadians(180)) //Finish pushing the block in
                .setReversed(false)
                .waitSeconds(0.1)
                .back(60)
                //.forward(60)
                //.splineToConstantHeading(new Vector2d(72, -59), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(79, -76), Math.toRadians(280))
                .waitSeconds(0.1)
                .back(60)
                .addTemporalMarker(()->{pidBARN.setTarget(BARN_UP);})
                //.splineToConstantHeading(new Vector2d(0.5, -30), Math.toRadians(180))
                .build();
        TrajectorySequence trajectoryGrabSpeceminFromPush = drive.trajectorySequenceBuilder(trajectoryPush.end())
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(0.5, -30), Math.toRadians(180))
                .build();

        waitForStart();//waitForStart();
        ElapsedTime timer = new ElapsedTime();


        pidFARM.setTarget(FARM_AUTO_FRONT_SPECEMIN);
        pidBARN.setTarget(0);
        lift2.setTarget(SLIDE_AUTO_FRONT_SPECEMIN);
        pidFARM.setActive(true);
        pidBARN.setActive(true);
        lift2.setActive(true);

        pidBARN.useTelemetry(telemetry, "pidBARN");


        //
        drive.followTrajectorySequenceAsync(trajectoryPlayPreload);
        updateEverything();
        lift2.setTarget(SLIDE_AUTO_FRONT_SPECEMIN_PLAY);
        updateEverything(800);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        updateEverything(300);
        lift2.setTarget(SLIDE_BOTTOM);
        pidFARM.setTarget(FARM_UP);
        openClaw2();
        drive.followTrajectorySequenceAsync(trajectoryPush);
        updateEverything();
        //updateEverything(30000);

        pidBARN.setTarget(BARN_UP);//Try not to knock specimen off of wall
        drive.followTrajectorySequenceAsync(trajectoryGrabSpeceminFromPush);
        updateEverything();
            updateEverything(500);
        closeClaw2();
            updateEverything(300);
        pidBARN.setTarget(BARN_DOWN);
        drive.followTrajectorySequenceAsync(trajectoryPlaySpecemin2);
            updateEverything();
        pidBARN.setTarget(BARN_HIGH_CHAMBER-60);//Compensate for current lack of a feedforward term
            updateEverything(500);
        drive.followTrajectorySequenceAsync(trajectoryPlaySpecemin2_2);
            updateEverything();


        pidBARN.setTarget(BARN_UP);
        updateEverything(500);
        closeClaw2();
        updateEverything(300);
        pidBARN.setTarget(BARN_DOWN);
        drive.followTrajectorySequenceAsync(trajectoryPlaySpecemin3);
        updateEverything();
        pidBARN.setTarget(BARN_HIGH_CHAMBER-60);//Compensate for current lack of a feedforward term
        updateEverything(500);
        drive.followTrajectorySequenceAsync(trajectoryPlaySpecemin3_2);
        updateEverything();
        //DONE
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        sleep(5000);
    }

    //Functions

    void updateEverything(){
        while (opModeIsActive() && drive.isBusy()) {
            updateIteration();
        }
    }
    void updateEverything(double milliseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            updateIteration();
        }
    }

    private void updateIteration() {
        drive.update();
        lift2.update();
        pidFARM.update();
        pidBARN.update();
        if(!sensorBARN.getState()){
            pidBARN.setPosition(0);
        }
        if(!sensorSlide.getState()){
            lift2.setPosition(0);
        }
        telemetry.addData("SensorSlide", !sensorSlide.getState());
        Pose2d poseEstimate = drive.getPoseEstimate();
        // Continually write pose to `PoseStorage`
        VariableStorage.currentPose = poseEstimate;
        telemetry.update();
    }
}
