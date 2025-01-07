package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_TOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.toggleButton;
import org.firstinspires.ftc.teamcode.toggleServo;

@TeleOp(group = "Beta")
public class enginerdsControl2 extends myLinearOpMode {
    public static boolean useFieldCentric;
    public static boolean hangMode;

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
        toggleServo armToggle = new toggleServo(servoArm, myConstants.servoPositions.ARM_UP, myConstants.servoPositions.ARM_DOWN);
        toggleServo clawLeftToggle = new toggleServo(servoClawLeft, myConstants.servoPositions.CLAW_LEFT_OPEN, myConstants.servoPositions.CLAW_LEFT_CLOSED);
        toggleServo clawRightToggle = new toggleServo(servoClawRight, myConstants.servoPositions.CLAW_RIGHT_OPEN, myConstants.servoPositions.CLAW_RIGHT_CLOSED);
        toggleServo wristToggle = new toggleServo(servoWrist, myConstants.servoPositions.WRIST_A, myConstants.servoPositions.WRIST_B);
        waitForStart();
        useFieldCentric = false;
        hangMode = false;
        //Run
        while(opModeIsActive()){
            odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            //Wheels
            //Get gamepad input
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;

            double heading = useFieldCentric ? odo.getHeading() : 0;
            //Rotate the heading
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1);
            double motorFLPower = (rotY + rotX + r);
            double motorBLPower = (rotY - rotX + r);
            double motorFRPower = (rotY - rotX - r);
            double motorBRPower = (rotY + rotX - r);
            denominator *= 1.5;
            if(gamepad1.left_bumper){
                denominator *= 2;
            }
            if(gamepad1.right_bumper){
                denominator *= 3;
            }
            if(gamepad1.x){
                useFieldCentric = true;
            }
            if(gamepad1.y){
                useFieldCentric = false;
            }
            if(gamepad1.back){
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
            //
            if(!hangMode) {
                if ((-gamepad2.right_stick_y < 0 && motorLL.getCurrentPosition() > SLIDE_BOTTOM) ||
                        (-gamepad2.right_stick_y > 0 && motorLL.getCurrentPosition() < SLIDE_TOP) || true) {
                    motorLL.setPower(-gamepad2.right_stick_y);
                    motorRR.setPower(-gamepad2.right_stick_y);
                } else {
                    motorLL.setPower(0);
                    motorRR.setPower(0);


                }
            }

            hangToggle.update(gamepad2.back);
            armToggle.update(gamepad2.b);
            clawLeftToggle.update(gamepad2.a);
            clawRightToggle.update(gamepad2.a);
            wristToggle.update(gamepad2.x);
            //
            telemetry.addData("Lifty", motorLL.getCurrentPosition());
            telemetry.addData("Risey", motorRR.getCurrentPosition());
            telemetry.addData("Field Centric", useFieldCentric);
            telemetry.update();
        }
    }
}
