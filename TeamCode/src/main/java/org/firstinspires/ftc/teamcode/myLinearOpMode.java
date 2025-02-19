package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class myLinearOpMode extends LinearOpMode {
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
    protected static PIDFLift lift;
    protected static PIDMotor lift2;
    protected static PIDMotor pidFARM;
    protected static PIDMotor pidBARN;
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
        //imu = hardwareMap.get(IMU.class, "imu");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //Initialize motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFARM.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBARN.setDirection(DcMotor.Direction.REVERSE);
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
        //Initialize pinpoint
        //TODO change offsets
        odo.setOffsets(myConstants.X_OFFSET, myConstants.Y_OFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(myConstants.xEncoder_DIRECTION, myConstants.yEncoder_DIRECTION);
        //
        lift = new PIDFLift();
        lift2 = new PIDMotor(new DcMotor[]{motorLL, motorRR});
        pidBARN = new PIDMotor(new DcMotor[]{motorBARN});
        pidFARM = new PIDMotor(new DcMotor[]{motorFARM});
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

    public class PIDFLift{
        public double P = 0.005, I = 0, D = 0.0004;
        PIDController controller;
        private boolean useTelemetry;

        public boolean isActive() {
            return isActive;
        }

        public void setActive(boolean active) {
            isActive = active;
        }

        private boolean isActive;
        public void setTarget(double target) {
            this.target = target;
        }
        private double target;
        public PIDFLift() {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            // Calculating variables
            controller = new PIDController(P, I, D);
            // Start the Slides down during the initialization phase
            target = 0;
            useTelemetry = true;
            isActive = true;
        }

        public void update() {
            if(isActive){
                // Beep boop this is the lift update function
                // Assume this runs some PID controller for the lift

                // Gives the controller specific values to use while calculating
                // Why do we have to feed it in every loop iteration?
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

                if(useTelemetry){
                    // Telemetry making sure that everything is running as it should
                    telemetry.addData("RR Pos", LinearSlide_Pos1);
                    telemetry.addData("LL Pos", LinearSlide_Pos2);
                    telemetry.addData("pid", pid);
                    telemetry.addData("Target Pos", target);
                }
            }
        }
    }

    public class PIDMotor extends PIDController{
        public static final double P = 0.005, I = 0, D = 0.0004;//Default values
        //PIDController controller;
        private boolean useTelemetry;
        private boolean isActive;
        private DcMotor[] motors;


        private double target;
        public boolean isActive() {
            return isActive;
        }

        @Override
        public void setSetPoint(double sp) {
            super.setSetPoint(sp);
            this.target = sp;
        }

        public void setActive(boolean active) {
            isActive = active;
        }
        @Deprecated
        public void setTarget(double target) {//setSetPoint
            this.target = target;
        }
        @Deprecated
        public double getTarget() {//setSetPoint
            return target;
        }
        public PIDMotor(DcMotor[] motors) {
            super(P, I, D);
            this.motors = motors;
            target = 0;
            useTelemetry = false;
            isActive = false;
        }
        public PIDMotor(DcMotor motor) {
            super(P, I, D);
            this.motors = new DcMotor[]{motor};
            target = 0;
            useTelemetry = false;
            isActive = false;
        }
        public PIDMotor(DcMotor[] motors, double P, double I, double D) {
            super(P, I, D);
            this.motors = motors;
            target = 0;
            useTelemetry = false;
            isActive = false;
        }
        public PIDMotor(DcMotor motor, double P, double I, double D) {
            super(P, I, D);
            this.motors = new DcMotor[]{motor};
            target = 0;
            useTelemetry = false;
            isActive = false;
        }

        public void update() {
            if(isActive){
                int motorPosition = motors[0].getCurrentPosition();
                double pid = calculate(motorPosition, target);
                for (DcMotor motor: motors) {
                    motor.setPower(pid);
                }

                if(useTelemetry){
                    // Telemetry making sure that everything is running as it should
                    telemetry.addData("Current Pos", motorPosition);
                    telemetry.addData("pid", pid);
                    telemetry.addData("Target Pos", target);
                }
            }
        }
    }

}
