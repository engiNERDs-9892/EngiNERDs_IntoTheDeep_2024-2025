package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
@TeleOp()
public class generalPositionTester extends myLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        motorLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBARN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(opModeInInit()){
            odo.update();
            telemetry.addData("FARM", motorFARM.getCurrentPosition());
            telemetry.addData("BARN", motorBARN.getCurrentPosition());
            telemetry.addData("LiftyLift", motorLL.getCurrentPosition());
            telemetry.addData("RiseyRise", motorRR.getCurrentPosition());
            telemetry.addData("odoHeading", odo.getHeading());
            telemetry.addData("odoPosition", odo.getPosition());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
            odo.update();
            telemetry.addData("FARM", motorFARM.getCurrentPosition());
            telemetry.addData("BARN", motorBARN.getCurrentPosition());
            telemetry.addData("LiftyLift", motorLL.getCurrentPosition());
            telemetry.addData("RiseyRise", motorRR.getCurrentPosition());
            telemetry.addData("odoHeading", odo.getHeading());
            telemetry.addData("odoPosition", odo.getPosition());
            telemetry.update();
        }
    }
}
