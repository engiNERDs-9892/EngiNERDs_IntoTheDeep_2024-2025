package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.myLinearOpMode;

@TeleOp(group = "Testing")
@Config
public class testStaticAgainstGravity extends myLinearOpMode {
    double stick;
    public static double factor = 0.02;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        motorBARN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBARN.setDirection(DcMotorSimple.Direction.FORWARD);
        VoltageSensor volageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry.addData("motorPosition", pidBARN.getPosition());
        telemetry.update();
        pidBARN.setArmFactors(0.02, 0.004408f);
        waitForStart();
        while(opModeIsActive()){
            //1,425.1 PPR at the Output Shaft

            //motor.setPower(gamepad2.right_stick_y*0.3 - 0.15*Math.sin(motor.getCurrentPosition() * .003491f));
            stick = -gamepad2.right_stick_y*0.3;
            //motorBARN.setPower(stick + factor*Math.cos(pidBARN.getPosition() * 0.004408f));
            pidBARN.setAdjustedPower(stick);
            if(!sensorBARN.getState()){
                pidBARN.setPosition(0);
            }
            telemetry.addData("motorPower", pidBARN.getPosition());
            telemetry.addData("motorPosition", pidBARN.getPosition());
            telemetry.addData("Voltage", volageSensor.getVoltage());
            telemetry.addData("Motor current", ((DcMotorEx)motorBARN).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
