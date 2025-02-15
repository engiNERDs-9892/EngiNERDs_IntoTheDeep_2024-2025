package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Testing")
@Config
public class testTeleopPID extends LinearOpMode {
    public static DcMotor motor;
    public PIDController controller;
    public ElapsedTime loopTimer;
    public double loopTime;
    @Override
    public void runOpMode(){
        controller = new PIDController(0.01, 0, 0.0005);

        motor = hardwareMap.get(DcMotor.class, "motorBARN");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageSensor volageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry.addData("motorPosition", motor.getCurrentPosition());
        telemetry.update();
        loopTimer = new ElapsedTime();
        waitForStart();
        loopTimer.reset();
        while(opModeIsActive()){
            //1,425.1 PPR at the Output Shaft
            loopTime = loopTimer.milliseconds();
            loopTimer.reset();
            controller.setSetPoint(controller.getSetPoint() - 5*gamepad2.right_stick_y);
            motor.setPower(controller.calculate(motor.getCurrentPosition()));
            telemetry.addData("motorPower", motor.getPower());
            telemetry.addData("motorPosition", motor.getCurrentPosition());
            telemetry.addData("motorPosition / 1425.1", motor.getCurrentPosition() / 1425.1);
            telemetry.addData("Voltage", volageSensor.getVoltage());
            telemetry.addData("Motor current", ((DcMotorEx)motor).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Setpoint", controller.getSetPoint());
            telemetry.update();
        }
    }
}
