package org.firstinspires.ftc.teamcode.PIDF_LS_TEST;

import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_TOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "advanced", preselectTeleOp = "enginerdsControl2")
public class LinearSlidePIDFWithAuto extends myLinearOpMode {
    private PIDController controller;

    // Variables for the LS motors
    public static double P = 0.021, I = 0, D = 0.0004;

    // Feedforward Component of the linear slides
    public static double f = 0;

    private double target;

    public final double ticks_in_degrees = 751.8 / 180;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        motorFARM.setTargetPosition(myConstants.ARM_UP);
        servoWrist.setPosition(servoPositions.WRIST_B);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        TrajectorySequence DropPreLoad = drive.trajectorySequenceBuilder(new Pose2d())
                // Drives over to the basket right after the start of auto
                .splineTo(new Vector2d(-0.5, 37), Math.toRadians(90))

                // Drops the Preloaded Sample into the top basket
                .addTemporalMarker(() -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })

                /////////
                // END //
                /////////
                .build();



        TrajectorySequence Sample1High = drive.trajectorySequenceBuilder(DropPreLoad.end())
                // Get the claws out of the top basket
                .addTemporalMarker(0.75, () -> {
                    servoWrist.setPosition(servoPositions.WRIST_B);
                    SlidesDown();
                })

                // Drive to the Far Right Sample
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(57, 1), 0
                )

                // Lower arm to pick up Sample
                .addTemporalMarker(() -> {
                    motorFARM.setTargetPosition(myConstants.ARM_DOWN);
                })

                // Grab the 1st sample
                .waitSeconds(0.9)
                .addTemporalMarker(() -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })

                // Raise Linear slides to top basket height + Raises arm up
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    SlidesUp();
                })

                // Drive over to the top basket
                .waitSeconds(0.8)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))

                // Drop the First Sample in the top basket
                .addTemporalMarker(() -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })

                /////////
                // END //
                /////////
                .build();




        TrajectorySequence Sample2High = drive.trajectorySequenceBuilder(DropPreLoad.end())
                // Removes the claws from the Top basket and lowers LS
                .addTemporalMarker(0.75, () -> {
                    servoWrist.setPosition(servoPositions.WRIST_B);
                    SlidesDown();
                })

                // Drives to pick up the Second sample
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(57, 15.25), 0
                )

                // Lowers front arm to pick up the sample
                .addTemporalMarker(() -> {
                    motorFARM.setTargetPosition(myConstants.ARM_DOWN);
                })

                // Closes the claws to pick up the sample
                .waitSeconds(0.9)
                .addTemporalMarker(() -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })

                // Raise Linear slides to top basket height + Raises Arm Up
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    SlidesUp();
                })

                // Drives over to the baskets
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))

                // Drops the Sample into the top basket
                .addTemporalMarker(() -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })

                /////////
                // END //
                /////////
                .build();


        TrajectorySequence Sample3High = drive.trajectorySequenceBuilder(DropPreLoad.end())

                // Removes the claws from the top basket
                .addTemporalMarker(0.75, () -> {
                    servoWrist.setPosition(servoPositions.WRIST_B);
                    SlidesDown();
                })

                // Drives over to the Last sample on the ground
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(57, 27.125), 0
                )

                // Lowers the arm to pick up the sample
                .addTemporalMarker(() -> {
                    motorFARM.setTargetPosition(myConstants.ARM_DOWN);
                })

                // Closes the claws and grabs the sample
                .waitSeconds(0.9)
                .addTemporalMarker(() -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
                })

                // Raises the Arm and LS to the Height of the Top Basket
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    motorFARM.setTargetPosition(myConstants.ARM_UP);
                    SlidesUp();
                })

                // Drives over to the baskets
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-0.5, 37), Math.toRadians(90))

                // Drops the sample in the top basket
                .addTemporalMarker(() -> {
                    servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
                    servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
                })

                /////////
                // END //
                /////////
                .build();





        TrajectorySequence EndAuto = drive.trajectorySequenceBuilder(DropPreLoad.end())
                // Removes the claws from the top basket after dropping sample, then lowers LS
                .addTemporalMarker(0.75, () -> {
                    servoWrist.setPosition(servoPositions.WRIST_B);
                    SlidesDown();
                })

                // Backs up along the wall so it doesn't lower on to the basket
                .back(30)

                /////////
                // END //
                /////////
                .build();




        waitForStart();
        if (isStopRequested()) return;

        // Raises the LS at the very beginning of Auto
        SlidesUp();
        sleep(500);

        // Drops Pre Loaded Sample in Top Basket
        drive.followTrajectorySequence(DropPreLoad);

        // Grabs and Drops 1st Sample in Top Basket
        drive.followTrajectorySequence(Sample1High);

        // Grabs and Drops 1st Sample in Top Basket
        drive.followTrajectorySequence(Sample2High);

        // Grabs and Drops 1st Sample in Top Basket
        drive.followTrajectorySequence(Sample3High);

        // Lowers LS and gets ready for auto
        drive.followTrajectorySequence(EndAuto);
        sleep(5000);

        //////////
        // DONE //
        //////////


        // Everything From Below this comment is code that the LS PID uses

        // All the code below this comment is how the PID runs
        while (opModeIsActive() && !isStopRequested()) {

            // We update drive continuously in the background, regardless of state
            drive.update();

            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
        }
    }
    // Sets the target of the LS to go all the way down
    public void SlidesDown() {target = 0;} // adjust

    // Sets the target of the LS to go all the way up
    public void SlidesUp() {target = 5500; } // adjust


    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            // Calculating variables
            controller = new PIDController(P, I,D);

            // Telemetry can be seen on FTC Dashboard
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            // Hardware maps for the LS motors (They have to be in the Lift hardware map)
            motorLL = hardwareMap.get(DcMotor.class,"motorLL");
            motorRR = hardwareMap.get(DcMotor.class,"motorRR");

            // Reverses the motors direction
            motorLL.setDirection(DcMotorSimple.Direction.REVERSE);

            // Start the Slides down during the initialization phase
            target = 0;
        }


        public void update() {
            // Added timer for telemetry
            ElapsedTime timer = new ElapsedTime();



            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift

            // Gives the controller specific values to use while calculating
            controller.setPID(P, I,D);

            // These two lines of code track where the motor current position to
            // calculate the proper power of the motors
            int LinearSlide_Pos1 = motorRR.getCurrentPosition();
            int LinearSlide_Pos2 = motorLL.getCurrentPosition();

            // this calculates the distance from how far the motor distance is from reaching the target in ticks
            double pid = controller.calculate(LinearSlide_Pos1,target);

            // Sets the LS Power
            motorRR.setPower(pid);
            motorLL.setPower(pid);

            // Telemetry making sure that everything is running as it should
            telemetry.addData("RR Pos", LinearSlide_Pos1);
            telemetry.addData("LL Pos", LinearSlide_Pos2);
            telemetry.addData("Target Pos", target);
            telemetry.addData("Time", timer.seconds());
            telemetry.update();




        }

    }
}


