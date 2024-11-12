package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class enginerdsControl extends LinearOpMode {
    private static DcMotor motorFL;
    private static DcMotor motorFR;
    private static DcMotor motorBL;
    private static DcMotor motorBR;
    private static DcMotor motorLL;//Lifty lift
    private static Servo servoBucket;
    private static Servo servoSlide;
    private static Servo servoIntake;
    private static Servo servoArm;

    @Override
    public void runOpMode(){
        //Initialization
        //hardwareMap
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorLL");
        servoArm = hardwareMap.servo.get("servoArm");
        servoBucket = hardwareMap.servo.get("servoBucket");
        servoIntake = hardwareMap.servo.get("servoIntake");
        servoSlide = hardwareMap.servo.get("servoSlide");
        //Initialize motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        //Initialize servos
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoBucket.setDirection(Servo.Direction.FORWARD);
        servoIntake.setDirection(Servo.Direction.FORWARD);
        servoSlide.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        //Run
        while(!isStopRequested()){
            //Wheels
            //Get gamepad input
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;

            double heading = 0;
            //Rotate the heading
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1);
            double motorFLPower = (rotY + rotX + r);
            double motorBLPower = (rotY - rotX + r);
            double motorFRPower = (rotY - rotX - r);
            double motorBRPower = (rotY + rotX - r);
            denominator = Math.max(Math.max(motorFLPower, motorFRPower), Math.max(motorBLPower, motorBRPower));
            motorFLPower /= denominator;
            motorFRPower /= denominator;
            motorBLPower /= denominator;
            motorBRPower /= denominator;

            motorFL.setPower(motorFLPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);
            //
            servoSlide.setPosition(gamepad2.left_stick_x);
            motorLL.setPower(gamepad2.right_stick_x);
            if(gamepad2.a){
                servoIntake.setPosition(0.5);
            }else{
                servoIntake.setPosition(0.2);
            }
            if(gamepad2.b){
                servoArm.setPosition(0.2);
            }else{
                servoArm.setPosition(0);
            }
            if(gamepad2.x){
                servoBucket.setPosition(0.2);
            }else{
                servoBucket.setPosition(0);
            }
        }
    }
}
