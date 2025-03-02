package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Testing")

public class testStaticAgainstGravity extends LinearOpMode {
    public static DcMotor motor;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "motorBARN");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        VoltageSensor volageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry.addData("motorPosition", motor.getCurrentPosition());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            //1,425.1 PPR at the Output Shaft

            //motor.setPower(gamepad2.right_stick_y*0.3 - 0.15*Math.sin(motor.getCurrentPosition() * .003491f));
            motor.setPower(gamepad2.right_stick_y*0.3 + 0.15*Math.cos(motor.getCurrentPosition() * 0.004408f));
            telemetry.addData("motorPower", motor.getPower());
            telemetry.addData("motorPosition", motor.getCurrentPosition());
            telemetry.addData("Voltage", volageSensor.getVoltage());
            telemetry.addData("Motor current", ((DcMotorEx)motor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
