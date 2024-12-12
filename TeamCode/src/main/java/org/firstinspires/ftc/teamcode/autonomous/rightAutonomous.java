package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.servoPositions;

@Autonomous
public class rightAutonomous extends myLinearOpMode {
    @Override
    public void runOpMode(){
        super.runOpMode();
        servoIntake.setPosition(0.5);
        servoSlide.setPosition(0.5);
        waitForStart();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivePower(0.5);
        Right(1200);
        Forward(200);
        Counterclockwise(-1000);
        Right(200);
        Forward(100);
        //servoArm.setPosition(servoPositions.ARM_INTAKE);
        sleep(200);
    }
}
