package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.testing.servoPositions;

public class myLinearOpMode extends LinearOpMode {
    protected static DcMotor motorFL;
    protected static DcMotor motorFR;
    protected static DcMotor motorBL;
    protected static DcMotor motorBR;
    protected static DcMotor motorLL;//Lifty lift
    protected static Servo servoBucket;
    protected static Servo servoSlide;
    protected static Servo servoIntake;
    protected static Servo servoArm;
    public static double unitsPerTick = 10;

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
        motorLL.setDirection(DcMotor.Direction.REVERSE);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Initialize servos
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoBucket.setDirection(Servo.Direction.FORWARD);
        servoIntake.setDirection(Servo.Direction.FORWARD);
        servoSlide.setDirection(Servo.Direction.FORWARD);
        servoBucket.setPosition(servoPositions.BUCKET_IN);
        servoArm.setPosition(servoPositions.ARM_OUTPUT);
        servoIntake.setPosition(0.5);
        servoSlide.setPosition(0.5);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Sets the run mode on all of the drive motors
     * calls DcMotor.setMode
     * @param mode the new run mode for the drive motors
     */
    public void setDriveMode(DcMotor.RunMode mode){
        motorFL.setMode(mode);
        motorFR.setMode(mode);
        motorBL.setMode(mode);
        motorBR.setMode(mode);
    }

    /**
     * Sets the power level of the drive motors
     *
     * @param power
     */
    public void setDrivePower(double power){
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);
    }

    /**
     * Moves the robot forward or backward
     * Negative values move backward
     * @param ticks How many ticks forward
     */
    public void Forward(double ticks){
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setTargetPosition((int)ticks);
        motorFR.setTargetPosition((int)ticks);
        motorBL.setTargetPosition((int)ticks);
        motorBR.setTargetPosition((int)ticks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(motorFL.isBusy()) sleep(100);
    }

    /**
     * Moves the robot right or left
     * Negative values move backwards
     * @param ticks How many ticks right
     */
    public void Right(double ticks){
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setTargetPosition( (int)ticks);
        motorFR.setTargetPosition(-(int)ticks);
        motorBL.setTargetPosition(-(int)ticks);
        motorBR.setTargetPosition( (int)ticks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(motorFL.isBusy()) sleep(100);
    }

    /**
     * Rotates the robot counterclockwise
     * Negative values move clockwise
     * @param ticks
     */
    public void Counterclockwise(double ticks){
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setTargetPosition(-(int)ticks);
        motorFR.setTargetPosition( (int)ticks);
        motorBL.setTargetPosition(-(int)ticks);
        motorBR.setTargetPosition( (int)ticks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(motorFL.isBusy()) sleep(100);
    }
}
