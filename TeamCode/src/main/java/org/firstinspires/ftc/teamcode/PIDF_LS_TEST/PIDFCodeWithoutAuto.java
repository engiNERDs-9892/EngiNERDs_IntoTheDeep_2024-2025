package org.firstinspires.ftc.teamcode.PIDF_LS_TEST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myLinearOpMode;

@Disabled
@Autonomous(group = "advanced", preselectTeleOp = "enginerdsControl2")
public class PIDFCodeWithoutAuto extends myLinearOpMode {

    // Sets a variable to reach out to the FTC Lib to help calculate the proper speed
    private PIDController controller;

    // Variables for the LS motors
    public static double P = 0.021, I = 0, D = 0.0004;

    // This allows you to change the LS position based on a function at the bottom of the code | "SlidesDown" & "SlidesUp"
    private double target;

    // This is just a way to track the motors current position based on degrees rather than ticks
    // Note that the number of ticks your motor has varies from motor to motor, so make sure you have the right number inputted
    public final double ticks_in_degrees = 751.8  / 180;
//                                         Ticks / Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize our lift function (Initialize our linear slides)

        Lift lift = new Lift(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(new Pose2d());


        ///////////////////////////////////////////////////////
        // this is where you build your trajectory sequences //
        ///////////////////////////////////////////////////////


        waitForStart();
        if (isStopRequested()) return;

        ///////////////////////////////////////////////////////
        // this is where you run your trajectory sequences   //
        ///////////////////////////////////////////////////////






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
    public void SlidesDown() {target = 0;} // adjust

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
