package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.myLinearOpMode;

@TeleOp(group = "Testing")
@Config
public class testTeleopPID extends myLinearOpMode {
    public PIDMotor controller;
    public ElapsedTime loopTimer;
    public double loopTime;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        controller = new PIDMotor(motorBARN);

        VoltageSensor volageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry.addData("motorPosition", motorBARN.getCurrentPosition());
        telemetry.update();

        loopTimer = new ElapsedTime();
        waitForStart();
        loopTimer.reset();
        while(opModeIsActive()){
            //1,425.1 PPR at the Output Shaft
            loopTime = loopTimer.milliseconds();
            loopTimer.reset();

            controller.setTarget(controller.getTarget() - 0.5*gamepad2.right_stick_y*loopTime);
            controller.update();

            telemetry.addData("motorPower", motorBARN.getPower());
            telemetry.addData("motorPosition", motorBARN.getCurrentPosition());
            telemetry.addData("motorPosition / 1425.1", motorBARN.getCurrentPosition() / 1425.1);
            telemetry.addData("Voltage", volageSensor.getVoltage());
            telemetry.addData("Motor current", ((DcMotorEx) motorBARN).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Setpoint", controller.getSetPoint());
            telemetry.addData("LoopTime", loopTime);
            telemetry.update();
        }
    }
}
