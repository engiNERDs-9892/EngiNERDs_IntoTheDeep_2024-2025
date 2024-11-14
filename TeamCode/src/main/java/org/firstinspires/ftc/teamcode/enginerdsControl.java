package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static boolean useFieldCentric;


    @Override
    public void runOpMode(){
        //Initialization
        //hardwareMap
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLL = hardwareMap.dcMotor.get("motorLL");
        servoArm = hardwareMap.servo.get("servoArm");
        servoBucket = hardwareMap.servo.get("servoBucket");
        servoIntake = hardwareMap.servo.get("servoIntake");
        servoSlide = hardwareMap.servo.get("servoSlide");
        //Initialize motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorLL.setDirection(DcMotor.Direction.FORWARD);
        //Initialize servos
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoBucket.setDirection(Servo.Direction.FORWARD);
        servoIntake.setDirection(Servo.Direction.FORWARD);
        servoSlide.setDirection(Servo.Direction.FORWARD);
        toggleServo bucketToggle = new toggleServo(servoBucket, servoPositions.BUCKET_IN, servoPositions.BUCKET_OUT);
        waitForStart();
        //Run
        while(opModeIsActive()){
            //Wheels
            //Get gamepad input
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;

            double heading = useFieldCentric ? 0.0 : 0;
            //Rotate the heading
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1);
            double motorFLPower = (rotY + rotX + r);
            double motorBLPower = (rotY - rotX + r);
            double motorFRPower = (rotY - rotX - r);
            double motorBRPower = (rotY + rotX - r);
            //denominator = Math.max(Math.max(Math.max(Math.abs(motorFLPower), Math.abs(motorFRPower)), Math.max(Math.abs(motorBLPower), Math.abs(motorBRPower))), 1);
            motorFLPower /= denominator;
            motorFRPower /= denominator;
            motorBLPower /= denominator;
            motorBRPower /= denominator;

            motorFL.setPower(motorFLPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);
            //
            servoSlide.setPosition((1+gamepad2.left_stick_x) * 0.5);
            motorLL.setPower(gamepad2.right_stick_y);
            servoIntake.setPosition((1 + gamepad2.right_trigger - gamepad2.left_trigger) * 0.5);
            if(gamepad2.x){
                servoArm.setPosition(servoPositions.ARM_INTAKE);
            }
            if(gamepad2.y){
                servoArm.setPosition(servoPositions.ARM_NEUTRAL);
            }
            if(gamepad2.b){
                servoArm.setPosition(servoPositions.ARM_OUTPUT);
            }
            bucketToggle.update(gamepad2.a);
        }
    }
}
