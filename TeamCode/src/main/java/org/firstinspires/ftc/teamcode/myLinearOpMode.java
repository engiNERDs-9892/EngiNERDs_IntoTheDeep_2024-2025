package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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
    protected static GoBildaPinpointDriver odo;
    protected static IMU imu;
    public static double unitsPerTick = 10;
    public void runOpMode(){
        initialize();
    }
    public void initialize(){
        initialize(true);
    }
    public void initialize(Boolean resetHeading){
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
        //imu = hardwareMap.get(IMU.class, "imu");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //Initialize motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorLL.setDirection(DcMotor.Direction.REVERSE);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Initialize servos
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoBucket.setDirection(Servo.Direction.FORWARD);
        servoIntake.setDirection(Servo.Direction.FORWARD);
        servoSlide.setDirection(Servo.Direction.FORWARD);
        servoBucket.setPosition(servoPositions.BUCKET_IN);
        servoArm.setPosition(servoPositions.ARM_OUTPUT);
        servoIntake.setPosition(0.5);
        servoSlide.setPosition(0.5);
        //Initialize pinpoint
        //TODO change offsets
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        if(resetHeading){
            odo.resetPosAndIMU();
        }

        /*
        //Initialize IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //*/
        //
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
