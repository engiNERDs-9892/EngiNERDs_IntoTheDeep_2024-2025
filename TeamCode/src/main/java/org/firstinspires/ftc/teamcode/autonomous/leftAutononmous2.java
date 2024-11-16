package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.servoPositions;

@Autonomous
public class leftAutononmous2 extends myLinearOpMode {

    @Override
    public void runOpMode(){
        super.runOpMode();
        waitForStart();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setTargetPosition(0);
        motorLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(0.5);
        motorLL.setPower(0.4);

        servoSlide.setPosition(0.4);
        sleep(500);
        servoSlide.setPosition(0.5);
        motorLL.setTargetPosition(4100);
        Forward(250);
        Right(-1025);
        Counterclockwise(-490);
        Right(-290);
        Forward(-270);
        while(motorLL.isBusy())sleep(100);
        servoBucket.setPosition(servoPositions.BUCKET_OUT);
        sleep(500);
        Right(290);
        Forward(280);
        motorLL.setTargetPosition(200);
        servoBucket.setPosition(servoPositions.BUCKET_IN);
        Counterclockwise(490);
        setDrivePower(0.8);
        Right(1200);
        Forward(-250);
        servoArm.setPosition(servoPositions.ARM_INTAKE);
        sleep(200);

        //GOAT
    }
}
