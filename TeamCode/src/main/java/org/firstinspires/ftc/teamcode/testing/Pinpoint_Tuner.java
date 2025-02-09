package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(group="Testing")
@Disabled
public class Pinpoint_Tuner extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    double avgXArray[] = new double[10000];
    double avgYArray[] = new double[10000];
    int avgLen = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        double rotationPower = 0.1;

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        backRightMotor = hardwareMap.dcMotor.get("motorBR");

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        /////////////////////////////////////////////////////////////////////////////////
        //////////////        ADJUST HUB DIRECTIONS TO SET HEADINGS        //////////////
        /////////////////////////////////////////////////////////////////////////////////
        //Ignore comment above
        //Copied from gobilda sample code please adjust values
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Update the pinpoint odometry controller
            odo.update();

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                odo.resetPosAndIMU();
            }

            odo.update();
            Pose2D position = odo.getPosition();
            timer.reset();
            sleep(20);
            odo.update();
            Pose2D position2 = odo.getPosition();
            Pose2D velocity = new Pose2D(
                    DistanceUnit.INCH,
                    (position.getX(DistanceUnit.INCH) - position2.getX(DistanceUnit.INCH))/ timer.seconds(),
                    (position.getY(DistanceUnit.INCH) - position2.getY(DistanceUnit.INCH))/ timer.seconds(),
                    AngleUnit.RADIANS,
                    (position.getHeading(AngleUnit.RADIANS) - position2.getHeading(AngleUnit.RADIANS))/ timer.seconds()
            );
            Pose2D velocity2 = odo.getVelocity();

            double frontLeftPower = - rotationPower;
            double backLeftPower = - rotationPower;
            double frontRightPower = rotationPower;
            double backRightPower = rotationPower;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            double dxdtheta = velocity.getX(DistanceUnit.MM) / velocity.getHeading(AngleUnit.RADIANS);
            double dydtheta = velocity.getY(DistanceUnit.MM) / velocity.getHeading(AngleUnit.RADIANS);

            double radius = Math.sqrt(dxdtheta * dxdtheta + dydtheta * dydtheta);

            //Why does Math.atan2(y, x)?
            double angle = Math.atan2(dxdtheta / radius, -dydtheta / radius) - position.getHeading(AngleUnit.RADIANS);

            //In normal people coords so I can actually comprehend them
            double predicted_x = radius * Math.cos(angle);
            double predicted_y = radius * Math.sin(angle);
            avgXArray[avgLen] = predicted_x;
            avgYArray[avgLen] = predicted_y;
            double avgX;
            double avgY;
            avgX = 0;
            avgY = 0;
            avgLen += 1;
            for(int i=1;i<avgLen;i++){
                avgX += avgXArray[i];
                avgY += avgYArray[i];
            }
            telemetry.addData("Time", timer.seconds());
            telemetry.addData("Position",
                    "X: %.2f Y: %.2f H: %.2f",
                    position.getX(DistanceUnit.MM),
                    position.getY(DistanceUnit.MM),
                    position.getHeading(AngleUnit.RADIANS)
            );
            telemetry.addData("Position2",
                    "X: %.2f Y: %.2f H: %.2f",
                    position2.getX(DistanceUnit.MM),
                    position2.getY(DistanceUnit.MM),
                    position2.getHeading(AngleUnit.RADIANS)
            );
            telemetry.addData("Velocity",
                    "X: %.2f Y: %.2f H: %.2f",
                    velocity.getX(DistanceUnit.MM),
                    velocity.getY(DistanceUnit.MM),
                    velocity.getHeading(AngleUnit.RADIANS)
            );
            telemetry.addData("Velocity2",
                    "X: %.2f Y: %.2f H: %.2f",
                    velocity2.getX(DistanceUnit.MM),
                    velocity2.getY(DistanceUnit.MM),
                    velocity2.getHeading(AngleUnit.RADIANS)
            );
            telemetry.addData("Rotation Power", "%f", rotationPower);
            telemetry.addData("Intermediate Variables", "dx: %.2f dy %.2f rad: %.2f ang: %.2f", dxdtheta, dydtheta, radius, angle);
            telemetry.addData("Predicted Offsets", "X: %.2f Y: %.2f", predicted_x, predicted_y);
            telemetry.addData("Predicted Offsets Average", "X: %.2f Y: %.2f", avgX / avgLen, avgY / avgLen);


            telemetry.update();
        }
    }
}