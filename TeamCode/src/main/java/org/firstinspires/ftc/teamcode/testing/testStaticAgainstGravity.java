package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Testing")
public class testStaticAgainstGravity extends LinearOpMode {
    public static DcMotor motor;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "motorFARM");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("motorPosition", motor.getCurrentPosition());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            motor.setPower(gamepad1.left_stick_y*0.25 + 0.1*Math.sin(motor.getCurrentPosition() * .003491));
            telemetry.addData("motorPower", motor.getPower());
            telemetry.addData("motorPosition", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
