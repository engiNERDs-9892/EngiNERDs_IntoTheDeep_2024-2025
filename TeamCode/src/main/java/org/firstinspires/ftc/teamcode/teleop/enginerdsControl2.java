package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.testing.servoPositions;
import org.firstinspires.ftc.teamcode.toggleServo;

@TeleOp
public class enginerdsControl2 extends myLinearOpMode {
    public static boolean useFieldCentric;

    @Override
    public void runOpMode(){
        super.runOpMode();
        //Initialization
        toggleServo bucketToggle = new toggleServo(servoBucket, servoPositions.BUCKET_IN, servoPositions.BUCKET_OUT);
        waitForStart();
        //Run
        while(opModeIsActive()){
            //Wheels
            //Get gamepad input
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
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
            motorLL.setPower(-gamepad2.right_stick_y);
            servoIntake.setPosition((1 + gamepad2.right_trigger - gamepad2.left_trigger) * 0.5);
            if(gamepad2.x){
                servoArm.setPosition(servoPositions.ARM_INTAKE);
            }
            
            if(gamepad2.b){
                servoArm.setPosition(servoPositions.ARM_OUTPUT);
            }
            bucketToggle.update(gamepad2.a);
            telemetry.addData("Lifty", motorLL.getCurrentPosition());
            telemetry.addData("Intake", servoIntake.getPosition());
            telemetry.update();
        }
    }
}
