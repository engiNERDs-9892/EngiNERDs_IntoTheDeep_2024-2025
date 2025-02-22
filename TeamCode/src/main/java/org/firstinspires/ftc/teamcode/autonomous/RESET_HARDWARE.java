package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.opModeGroups.DEFAULT_TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.opModeGroups;

@Autonomous(name="RESET HARDWARE", group = opModeGroups.auto.BASIC, preselectTeleOp = DEFAULT_TELEOP)
public class RESET_HARDWARE extends myLinearOpMode {
    public void reset(){
        motorBARN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBARN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFARM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo.resetPosAndIMU();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        reset();
        waitForStart();
    }
}
