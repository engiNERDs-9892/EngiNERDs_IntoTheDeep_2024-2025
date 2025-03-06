package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class myLinearOpMode extends LinearOpMode {
    protected static DcMotor motorFL;
    protected static DcMotor motorFR;
    protected static DcMotor motorBL;
    protected static DcMotor motorBR;
    protected static DcMotor motorLL;//Lifty lift
    protected static DcMotor motorRR;//Risey Rise
    protected static DcMotor motorFARM;
    protected static DcMotor motorBARN;
    protected static Servo servoClawLeft;
    protected static Servo servoClawRight;
    protected static Servo servoClawLeft2;
    protected static Servo servoClawRight2;
    protected static Servo servoWrist;
    protected static GoBildaPinpointDriver odo;
    protected static IMU imu;
    protected static DigitalChannel sensorSlide;
    protected static DigitalChannel sensorBARN;
    protected static motorsController lift;
    protected static motorsController lift2;
    protected static motorsController pidFARM;
    protected static motorsController pidBARN;
    public static double unitsPerTick = 10;
    protected static double drivePower;
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        //hardwareMap
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLL = hardwareMap.dcMotor.get("motorLL");
        motorRR = hardwareMap.dcMotor.get("motorRR");
        motorFARM = hardwareMap.dcMotor.get("motorFARM");
        motorBARN = hardwareMap.dcMotor.get("motorBARN");
        servoClawLeft = hardwareMap.servo.get("servoClawLeft");
        servoClawRight = hardwareMap.servo.get("servoClawRight");
        servoClawLeft2 = hardwareMap.servo.get("servo2Left");
        servoClawRight2 = hardwareMap.servo.get("servo2Right");
        servoWrist = hardwareMap.servo.get("servoWrist");
        sensorSlide = hardwareMap.get(DigitalChannel.class, "sensorSlide");
        sensorBARN = hardwareMap.get(DigitalChannel.class, "sensorBARN");
        //imu = hardwareMap.get(IMU.class, "imu");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //Initialize motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFARM.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBARN.setDirection(DcMotor.Direction.FORWARD);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setDirection(DcMotor.Direction.REVERSE);
        motorLL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Initialize servos
        servoClawLeft.setDirection(Servo.Direction.FORWARD);
        servoClawRight.setDirection(Servo.Direction.REVERSE);
        servoClawLeft2.setDirection(Servo.Direction.REVERSE);
        servoClawRight2.setDirection(Servo.Direction.FORWARD);
        //Initialize digital ports
        //Values returned from port are reversed from what you may expect
        sensorBARN.setMode(DigitalChannel.Mode.INPUT);
        sensorSlide.setMode(DigitalChannel.Mode.INPUT);
        //Initialize pinpoint
        //TODO change offsets
        odo.setOffsets(myConstants.X_OFFSET, myConstants.Y_OFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(myConstants.xEncoder_DIRECTION, myConstants.yEncoder_DIRECTION);
        //

        //
        lift2 = new motorsController(new DcMotor[]{motorLL, motorRR});
        lift = lift2;
        pidBARN = new motorsController(new DcMotor[]{motorBARN});
        pidFARM = new motorsController(new DcMotor[]{motorFARM});
        //odo.resetPosAndIMU();
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
        drivePower = power;
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);
    }

    public void openClaw1(){
        servoClawLeft.setPosition(myConstants.servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(myConstants.servoPositions.CLAW_RIGHT_OPEN);
    }
    public void closeClaw1(){
        servoClawLeft.setPosition(myConstants.servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(myConstants.servoPositions.CLAW_RIGHT_CLOSED);
    }
    public void openClaw2(){
        servoClawLeft2.setPosition(myConstants.servoPositions.CLAW_LEFT_2_OPEN);
        servoClawRight2.setPosition(myConstants.servoPositions.CLAW_RIGHT_2_OPEN);
    }
    public void closeClaw2(){
        servoClawLeft2.setPosition(myConstants.servoPositions.CLAW_LEFT_2_CLOSED);
        servoClawRight2.setPosition(myConstants.servoPositions.CLAW_RIGHT_2_CLOSED);
    }

}
