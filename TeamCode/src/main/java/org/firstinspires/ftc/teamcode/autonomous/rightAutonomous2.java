package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.servoPositions;

@Autonomous
public class rightAutonomous2 extends myLinearOpMode {
    @Override
    public void runOpMode(){
        super.runOpMode();
        waitForStart();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivePower(0.5);
        Right(2800);
        //servoArm.setPosition(servoPositions.ARM_INTAKE);
        sleep(200);
    }
}
