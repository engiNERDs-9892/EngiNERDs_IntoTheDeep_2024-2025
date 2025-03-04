package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.myConstants.BARN_DOWN;
import static org.firstinspires.ftc.teamcode.myConstants.BARN_UP;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_TOP;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drive.VariableStorage;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.toggleButton;
import org.firstinspires.ftc.teamcode.toggleServo;

@TeleOp(group = "Beta")
public class EngiNERDs_Control extends myLinearOpMode {
    public static boolean useFieldCentric;
    public static boolean hangMode;
    public static boolean useMotorLimits;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //Initialization
        toggleButton hangToggle = new toggleButton() {
            @Override
            public void toggleOn() {
                hangMode = true;
                motorLL.setTargetPosition(motorLL.getCurrentPosition());
                motorRR.setTargetPosition(motorRR.getCurrentPosition());
                motorLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLL.setPower(0.75);
                motorRR.setPower(0.75);
            }

            @Override
            public void toggleOff() {
                hangMode = false;
                motorLL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorLL.setPower(0);
                motorRR.setPower(0);
            }
        };
        toggleServo clawLeftToggle = new toggleServo(servoClawLeft, myConstants.servoPositions.CLAW_LEFT_OPEN, myConstants.servoPositions.CLAW_LEFT_CLOSED);
        toggleServo clawRightToggle = new toggleServo(servoClawRight, myConstants.servoPositions.CLAW_RIGHT_OPEN, myConstants.servoPositions.CLAW_RIGHT_CLOSED);
        toggleServo clawLeft2Toggle = new toggleServo(servoClawLeft2, myConstants.servoPositions.CLAW_LEFT_2_OPEN, myConstants.servoPositions.CLAW_LEFT_2_CLOSED);
        toggleServo clawRight2Toggle = new toggleServo(servoClawRight2, myConstants.servoPositions.CLAW_RIGHT_2_OPEN, myConstants.servoPositions.CLAW_RIGHT_2_CLOSED);
        toggleServo wristToggle = new toggleServo(servoWrist, myConstants.servoPositions.WRIST_A, myConstants.servoPositions.WRIST_B);
        ElapsedTime loopTimer = new ElapsedTime();
        double loopTime;
        if(!VariableStorage.hasRunOpmode){
            VariableStorage.hasRunOpmode = true;
            motorBARN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBARN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFARM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        loopTimer.reset();
        useFieldCentric = false;
        hangMode = false;
        useMotorLimits = true;
        pidFARM.setActive(true);
        pidBARN.setActive(true);
        clawLeft2Toggle.activate();
        clawRight2Toggle.activate();
        clawLeftToggle.activate();
        clawRightToggle.activate();
        wristToggle.activate();

        //Run
        while(opModeIsActive()) {
            odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            loopTime = loopTimer.milliseconds();
            loopTimer.reset();

            //Wheels
            //Get gamepad input
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;

            double heading = useFieldCentric ? odo.getHeading() : 0;
            //Rotate the heading
            double rotX = x * cos(-heading) - y * sin(-heading);
            double rotY = x * sin(-heading) + y * cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1);
            double motorFLPower = (rotY + rotX + r);
            double motorBLPower = (rotY - rotX + r);
            double motorFRPower = (rotY - rotX - r);
            double motorBRPower = (rotY + rotX - r);
            denominator *= 1.0;
            if (gamepad1.left_bumper) {
                denominator *= 2;
            }
            if (gamepad1.right_bumper) {
                denominator *= 3;
            }
            if (gamepad1.x) {
                useFieldCentric = true;
            }
            if (gamepad1.y) {
                useFieldCentric = false;
            }
            if (gamepad1.back) {
                odo.resetPosAndIMU();
            }
            motorFLPower /= denominator;
            motorFRPower /= denominator;
            motorBLPower /= denominator;
            motorBRPower /= denominator;

            motorFL.setPower(motorFLPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);

            double barnTarget = pidBARN.getTarget();
            double farmTarget = pidFARM.getTarget();//Unused
            double farmStick = gamepad2.left_stick_y;
            double barnStick = -gamepad2.right_stick_y;
            double slideStick = gamepad2.right_trigger-gamepad2.left_trigger;
            if(farmStick != 0 || (motorFARM.getCurrentPosition() < 320 && !sensorSlide.getState())){
                motorFARM.setPower(
                        farmStick*0.4 +
                        -0.2*sin(motorFARM.getCurrentPosition()*myConstants.FARM_RADIANS_PER_TICK)
                );
            } else {
                motorFARM.setPower(0);
            }

            if(!sensorBARN.getState()){
                pidBARN.setPosition(0);
                pidBARN.setTarget(Math.max(pidBARN.getTarget() + 0.8*loopTime*barnStick, 0));
            }else if(BARN_UP < barnTarget && barnTarget < BARN_DOWN || !useMotorLimits){
                pidBARN.setTarget(barnTarget + 0.8*loopTime*barnStick);
            }else {
                pidBARN.setTarget(barnTarget + 0.1*loopTime*barnStick);
            }
            //pidBARN.setTarget(pidBARN.getTarget() - 0.5*gamepad2.left_stick_x*loopTime);
            //pidFARM.setTarget(pidFARM.getTarget() + 0.5*gamepad2.left_stick_y*loopTime);
            pidBARN.update();
            //pidFARM.update();
            //
            if(!hangMode) {
                if ((slideStick < 0 && lift2.getPosition() > SLIDE_BOTTOM) ||
                        (slideStick > 0 && lift2.getPosition() < SLIDE_TOP) || false) {
                    motorLL.setPower(slideStick);
                    motorRR.setPower(slideStick);
                } else {
                    motorLL.setPower(slideStick * 0.1);
                    motorRR.setPower(slideStick * 0.1);
                }
            }
            if(!sensorSlide.getState()){
                lift2.setPosition(0);
            }

            hangToggle.update(gamepad2.back);
            clawLeftToggle.update(gamepad2.a);
            clawRightToggle.update(gamepad2.a);
            clawLeft2Toggle.update(gamepad2.y);
            clawRight2Toggle.update(gamepad2.y);

            wristToggle.update(gamepad2.x);
            //
            //telemetry.addData("Lifty", motorLL.getCurrentPosition());
            //telemetry.addData("Risey", motorRR.getCurrentPosition());
            telemetry.addData("Slide", lift2.getPosition());
            telemetry.addData("FARM", motorFARM.getCurrentPosition());
            telemetry.addData("BARN", pidBARN.getPosition());
            telemetry.addData("BARN/1425.1", motorBARN.getCurrentPosition()/1425.1);
            telemetry.addData("Field Centric", useFieldCentric);
            telemetry.addData("barnS" ,sensorBARN.getState());
            telemetry.addData("SlideS" ,sensorSlide.getState());
            telemetry.update();
        }
    }
}
