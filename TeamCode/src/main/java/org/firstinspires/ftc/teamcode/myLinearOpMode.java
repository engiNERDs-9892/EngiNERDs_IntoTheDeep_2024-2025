package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class myLinearOpMode extends LinearOpMode {
    protected static DcMotor motorFL;
    protected static DcMotor motorFR;
    protected static DcMotor motorBL;
    protected static DcMotor motorBR;
    protected static DcMotor motorLL;//Lifty lift
    protected static DcMotor motorRR;//Risey Rise
    protected static DcMotor motorFARM;
    protected static Servo servoClawLeft;
    protected static Servo servoClawRight;
    protected static Servo servoClawLeft2;
    protected static Servo servoClawRight2;
    protected static Servo servoArm;
    protected static Servo servoWrist;
    protected static GoBildaPinpointDriver odo;
    protected static IMU imu;
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
        servoArm = hardwareMap.servo.get("servoArm");
        motorFARM = hardwareMap.dcMotor.get("motorFARM");
        servoClawLeft = hardwareMap.servo.get("servoClawLeft");
        servoClawRight = hardwareMap.servo.get("servoClawRight");
        servoClawLeft2 = hardwareMap.servo.get("servo2Left");
        servoClawRight2 = hardwareMap.servo.get("servo2Right");
        servoWrist = hardwareMap.servo.get("servoWrist");
        //imu = hardwareMap.get(IMU.class, "imu");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //Initialize motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFARM.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLL.setDirection(DcMotor.Direction.REVERSE);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoClawLeft.setDirection(Servo.Direction.FORWARD);
        servoClawRight.setDirection(Servo.Direction.FORWARD);
        //Initialize pinpoint
        //TODO change offsets
        odo.setOffsets(myConstants.X_OFFSET, myConstants.Y_OFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(myConstants.xEncoder_DIRECTION, myConstants.yEncoder_DIRECTION);
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

    /**
     * Rotates the robot counterclockwise
     * Negative values move clockwise
     * @param degrees
     */
    public void Counterclockwise_1(double degrees){
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        double targetHeading = odo.getHeading() + degrees * Math.PI / 180.0;
        pid PID = new pid(0.47, 0.15, 0.014, targetHeading);
        //noinspection ReassignedVariable
        double power;
        while(opModeIsActive() && PID.isBusy()){
            odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            power = PID.update(odo.getHeading());
            motorFL.setPower(-power * drivePower);
            motorFR.setPower( power * drivePower);
            motorBL.setPower(-power * drivePower);
            motorBR.setPower( power * drivePower);
        }
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivePower(drivePower);
    }
    public void TurnTo(double degrees){
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        double targetHeading = degrees * Math.PI / 180.0;
        pid PID = new pid(0.47, 0.15, 0.014, targetHeading);
        //noinspection ReassignedVariable
        double power;
        while(opModeIsActive() && PID.isBusy()){
            odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            power = PID.update(odo.getHeading());
            motorFL.setPower(-power);
            motorFR.setPower( power);
            motorBL.setPower(-power);
            motorBR.setPower( power);
        }
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivePower(drivePower);
    }
    public void Move_1(double xTarget, double yTarget, double hTarget){
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo.update();
        pid xPID = new pid(0.006, 0.03, 0.0003, odo.getPosX()+xTarget);
        pid yPID = new pid(0.006, 0.03, 0.0003, odo.getPosY()+yTarget);
        pid hPID = new pid(0.6, 0.7, 0.01, odo.getHeading()+hTarget*Math.PI/180.0);
        while(opModeIsActive() && (xPID.isBusy() || yPID.isBusy() || hPID.isBusy())){
            odo.update();
            double x = -yPID.update(odo.getPosY());
            double y = xPID.update(odo.getPosX());
            double r = -hPID.update(odo.getHeading());
            double heading = odo.getHeading();
            telemetry.addData("Xe", y);
            telemetry.addData("Ye", x);
            telemetry.addData("He", r);
            telemetry.addData("h", heading);
            telemetry.update();
            //Rotate the heading
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1) * 1.5;
            double motorFLPower = (rotY + rotX + r) / denominator;
            double motorFRPower = (rotY - rotX - r) / denominator;
            double motorBLPower = (rotY - rotX + r) / denominator;
            double motorBRPower = (rotY + rotX - r) / denominator;
            motorFL.setPower( motorFLPower);
            motorFR.setPower( motorFRPower);
            motorBL.setPower( motorBLPower);
            motorBR.setPower( motorBRPower);
        }
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivePower(drivePower);
    }

    class pid{
        double p;
        double i;
        double d;
        ElapsedTime timer;
        ElapsedTime stopTimer;
        double power, error, previous_error, int_error, dif_error, target;
        double time = 0;
        pid(double p, double i, double d, double currentPosition){
            this.p=p;
            this.i=i;
            this.d=d;
            timer = new ElapsedTime();
            stopTimer = new ElapsedTime();
            target = currentPosition;
        }
        public void setTarget(double target) {
            this.target = target;
        }
        public double update(double position){
            time = timer.seconds();
            timer.reset();
            error = target - position;
            int_error += error * time;
            dif_error = (error - previous_error) / time;
            power = error * p + int_error * i + dif_error * d;
            if(error >= 10){
                stopTimer.reset();
            }
            return power;
        }
        public boolean isBusy() {
            return stopTimer.seconds() < 1.0;
        }
    }
}
