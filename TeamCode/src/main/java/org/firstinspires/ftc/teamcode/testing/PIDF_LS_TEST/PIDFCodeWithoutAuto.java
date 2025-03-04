package org.firstinspires.ftc.teamcode.testing.PIDF_LS_TEST;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.VariableStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myLinearOpMode;

//@Disabled
@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control")
@Disabled
public class PIDFCodeWithoutAuto extends myLinearOpMode {

    // Sets a variable to reach out to the FTC Lib to help calculate the proper speed
    private PIDController controller;

    // Variables for the LS motors
    public static double P = 0.021, I = 0, D = 0.0004;

    // This allows you to change the LS position based on a function at the bottom of the code | "SlidesDown" & "SlidesUp"
    private double target;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize our lift function (Initialize our linear slides)

        motorsController lift = new motorsController(new DcMotor[]{motorLL, motorRR});

        // Set inital pose
        drive.setPoseEstimate(new Pose2d());


        ///////////////////////////////////////////////////////
        // this is where you build your trajectory sequences //
        ///////////////////////////////////////////////////////


        waitForStart();
        if (isStopRequested()) return;

        lift.setTarget(myConstants.SLIDE_TOP);
        sleep(500);

        ///////////////////////////////////////////////////////
        // this is where you run your trajectory sequences   //
        ///////////////////////////////////////////////////////






        // All the code below this comment is how the PID runs
        while (opModeIsActive()) {

            // We update drive continuously in the background, regardless of state
            drive.update();

            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            VariableStorage.currentPose = poseEstimate;
            telemetry.update();
        }
    }
    public void SlidesDown() {target = 0;} // adjust

    public void SlidesUp() {target = 5500; } // adjust
}
