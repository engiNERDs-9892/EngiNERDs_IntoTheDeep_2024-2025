package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_SAMPLE_GRAB;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_TOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control")
public class leftAutonomousRoadrunnerPID extends myLinearOpMode {
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        drive = new SampleMecanumDrive(hardwareMap);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_OPEN);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_OPEN);
        motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFARM.setTargetPosition(myConstants.ARM_UP);
        motorFARM.setPower(0.4);
        servoWrist.setPosition(servoPositions.WRIST_B);
        final Vector2d basketPosition = new Vector2d(-0.5, 35.25);
        final Vector2d sample1PickupPosition = new Vector2d(57.75, 2.25);
        final Vector2d sample2PickupPosition = new Vector2d(57.75, 16.875);
        final Vector2d sample3PickupPosition = new Vector2d(57.75, 28.625);
        final Pose2d ascentPosition = new Pose2d(80, -5, Math.toRadians(270));

        TrajectorySequence trajectoryPlayPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(basketPosition, Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();
        TrajectorySequence trajectoryPlaySample1 = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                /*.addTemporalMarker(0.2, () -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })*/
                .addTemporalMarker(1.0, () -> {
                    lift.setTarget(SLIDE_AUTO_SAMPLE_GRAB);
                    servoWrist.setPosition(servoPositions.WRIST_B);
                })
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(sample1PickupPosition, 0)

                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_AUTO_SAMPLE_GRAB);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTarget(SLIDE_TOP);
                })
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(basketPosition, Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();
        TrajectorySequence trajectoryPlaySample2 = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                /*.addTemporalMarker(0.2, () -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })*/
                .addTemporalMarker(1.0, () -> {
                    lift.setTarget(SLIDE_AUTO_SAMPLE_GRAB);
                    servoWrist.setPosition(servoPositions.WRIST_B);
                })
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(sample2PickupPosition, 0)

                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_AUTO_SAMPLE_GRAB);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .waitSeconds(0.9)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTarget(SLIDE_TOP);
                })
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(basketPosition, Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();
        TrajectorySequence trajectoryPlaySample3 = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                /*.addTemporalMarker(0.2, () -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })*/
                .addTemporalMarker(1.0, () -> {
                    lift.setTarget(SLIDE_AUTO_SAMPLE_GRAB);
                    servoWrist.setPosition(servoPositions.WRIST_B);
                })
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(sample3PickupPosition, 0)

                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_AUTO_SAMPLE_GRAB);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .waitSeconds(.9)
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTarget(SLIDE_TOP +50);
                })
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(basketPosition, Math.toRadians(90))
                .addTemporalMarker(()->{
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })
                .build();


        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                .back(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(() -> {
                    lift.setTarget(SLIDE_AUTO_SAMPLE_GRAB);
                })
                .lineToConstantHeading(
                        new Vector2d(35, -20),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToConstantHeading(new Vector2d(35, -135))
                .splineToConstantHeading(new Vector2d(10, -135), 270)
                .build();
        TrajectorySequence trajectory2b = drive.trajectorySequenceBuilder(trajectoryPlayPreload.end())
                /*.addTemporalMarker(0.2, () -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })*/
                .addTemporalMarker(0.5, () -> {
                    lift.setTarget(SLIDE_BOTTOM);
                    //servoWrist.setPosition(servoPositions.WRIST_B);
                })
                .setReversed(true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))//20 15 30
                //Not enough time to do a level 1 ascent
                //.splineTo(new Vector2d(20, 15), Math.toRadians(30))
                //.splineToSplineHeading(new Pose2d(20+.8660*20, 15+.5*20, Math.toRadians(230)), Math.toRadians(0))
                //.addTemporalMarker(()->{
                //    motorFARM.setTargetPosition(ARM_UP+50);
                //})
                //.splineToSplineHeading(ascentPosition, Math.toRadians(270))
                //.addTemporalMarker(()->{
                //    motorFARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //    motorFARM.setPower(0);
                //    motorFARM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //})
                .build();
        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTarget(SLIDE_TOP);
        updateEverything(700);


        drive.followTrajectorySequenceAsync(trajectoryPlayPreload);
        updateEverything();
        drive.followTrajectorySequenceAsync(trajectoryPlaySample1);
        updateEverything();
        drive.followTrajectorySequenceAsync(trajectoryPlaySample2);
        updateEverything();
        drive.followTrajectorySequenceAsync(trajectoryPlaySample3);
        updateEverything();

        //drive.followTrajectorySequence(trajectoryPlayPreload);
        //drive.followTrajectorySequence(trajectoryPlaySample1);
        //drive.followTrajectorySequence(trajectoryPlaySample2);
        //drive.followTrajectorySequence(trajectoryPlaySample3);

        telemetry.addData("Time1", timer.seconds());
        telemetry.update();
        drive.followTrajectorySequenceAsync(trajectory2b);
        updateEverything();
        //DONE
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        sleep(5000);
        //motorFARM.setTargetPosition(servoPositions.ARM_UP);
    }
    void updateEverything(){
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            lift.update();
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
            lift.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
            telemetry.update();
        }
    }
}
