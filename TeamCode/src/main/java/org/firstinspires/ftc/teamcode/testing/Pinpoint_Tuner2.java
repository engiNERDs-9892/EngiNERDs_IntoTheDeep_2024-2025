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
import org.firstinspires.ftc.teamcode.edgeButton;

import java.util.Arrays;


@TeleOp(group="Testing")
@Disabled
public class Pinpoint_Tuner2 extends LinearOpMode {
    public static DcMotor frontLeftMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backRightMotor;

    double xOffset = 0;
    double yOffset = 0;
    double xPositions[] = new double[1000];
    double yPositions[] = new double[1000];
    int index = 0;

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

        edgeButton incX = new edgeButton() {
            @Override
            public void trigger() {
                xOffset += 10;
            }
        };
        edgeButton decX = new edgeButton() {
            @Override
            public void trigger() {
                xOffset -= 10;
            }
        };
        edgeButton incY = new edgeButton() {
            @Override
            public void trigger() {
                yOffset += 10;
            }
        };
        edgeButton decY = new edgeButton() {
            @Override
            public void trigger() {
                yOffset -= 10;
            }
        };
        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Update the pinpoint odometry controller
            odo.update();
            telemetry.addLine("X vertical pod left right offset (left is positive)");
            telemetry.addLine("Y Horizontal pod forward back offset (forward is positive)");
            telemetry.addLine("incX: Y decX: X incY: B decY: A");
            telemetry.addData("xOffset", xOffset);
            telemetry.addData("yOffset", yOffset);
            incX.update(gamepad1.y);
            decX.update(gamepad1.x);
            incY.update(gamepad1.b);
            decY.update(gamepad1.a);
            if(gamepad1.start){
                odo.setOffsets(xOffset, yOffset);
                odo.resetPosAndIMU();
                index = 0;
                sleep(750);
                frontLeftMotor.setPower(-.4);
                backLeftMotor.setPower(-.4);
                frontRightMotor.setPower(.4);
                backRightMotor.setPower(.4);
                Pose2D position = odo.getPosition();
                timer.reset();
                while((timer.seconds() < 2.5 || timer.seconds() > 2.5 && position.getHeading(AngleUnit.RADIANS) < 0) && opModeIsActive()) {
                    odo.update();
                    position = odo.getPosition();
                    xPositions[index] = position.getX(DistanceUnit.INCH);
                    yPositions[index] = position.getY(DistanceUnit.INCH);
                    index += 1;
                    if(timer.seconds() > 3.5){
                        frontLeftMotor.setPower(-.1);
                        backLeftMotor.setPower(-.1);
                        frontRightMotor.setPower(.1);
                        backRightMotor.setPower(.1);
                    }
                    telemetry.addData("xOffset", xOffset);
                    telemetry.addData("yOffset", yOffset);
                    telemetry.addData("X", position.getX(DistanceUnit.INCH));
                    telemetry.addData("Y", position.getY(DistanceUnit.INCH));
                    telemetry.addData("H", position.getHeading(AngleUnit.RADIANS));
                    telemetry.addData("index", index);
                    telemetry.addData("Time", timer.seconds());
                    telemetry.update();
                }
                double avgx = 0;
                for(int i=0;i<index;i++){avgx += xPositions[i];}
                avgx /= index;
                double avgy = 0;
                for(int i=0;i<index;i++){avgy += yPositions[i];}
                avgy /= index;
                double avgdist = 0;
                for(int i=0;i<index;i++){avgdist += Math.sqrt(Math.pow(xPositions[i]-avgx, 2) + Math.pow(xPositions[i]-avgy, 2));}
                avgdist /= index;
                double madx = 0;
                for(int i=0;i<index;i++){madx += Math.abs(xPositions[i]-avgx);}
                madx /= index;
                double mady = 0;
                for(int i=0;i<index;i++){mady += Math.abs(yPositions[i]-avgy);}
                mady /= index;
                telemetry.addData("xOffset", xOffset);
                telemetry.addData("yOffset", yOffset);
                telemetry.addData("X", position.getX(DistanceUnit.INCH));
                telemetry.addData("Y", position.getY(DistanceUnit.INCH));
                telemetry.addData("H", position.getHeading(AngleUnit.RADIANS));
                telemetry.addData("index", index);
                telemetry.addData("Time", timer.seconds());
                telemetry.addData("avgX", avgx);
                telemetry.addData("avgY", avgy);
                telemetry.addData("madX", madx);
                telemetry.addData("madY", mady);
                telemetry.addData("avgDist", avgdist);
                telemetry.addData("Center dist", Math.sqrt(Math.pow(avgx, 2) + Math.pow(avgy, 2)));
                telemetry.update();
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                while(!gamepad1.back && opModeIsActive());
            }


            telemetry.addData("Time", timer.seconds());
            /*telemetry.addData("Position",
                    "X: %.2f Y: %.2f H: %.2f",
                    position.getX(DistanceUnit.MM),
                    position.getY(DistanceUnit.MM),
                    position.getHeading(AngleUnit.RADIANS)
            );*/

            telemetry.update();
        }
    }
}