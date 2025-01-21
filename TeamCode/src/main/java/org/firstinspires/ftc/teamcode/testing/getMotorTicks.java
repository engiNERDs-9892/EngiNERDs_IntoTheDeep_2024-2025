package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.repeatButton;

@TeleOp(group = "Testing")
public class getMotorTicks extends LinearOpMode {
    public static DcMotor motor;
    public static int increment;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "motorFARM");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("motorTarget", motor.getTargetPosition());
        telemetry.addData("motorPosition", motor.getCurrentPosition());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("\u0394", increment);
            telemetry.addData("motorTarget", motor.getTargetPosition());
            telemetry.addData("motorPosition", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
