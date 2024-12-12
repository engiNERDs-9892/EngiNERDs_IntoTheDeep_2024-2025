package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.servoPositions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class leftAutononmous extends myLinearOpMode {

    @Override
    public void runOpMode(){
        super.runOpMode();
        servoIntake.setPosition(0.5);
        servoSlide.setPosition(0.5);
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
        motorLL.setTargetPosition(4400);
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
        Forward(1100);
        Right(4800);
        setDrivePower(0.2);
        Forward(-700);
        servoArm.setPosition(servoPositions.ARM_INTAKE);
        sleep(200);

        //GOAT
        /*
        Move(directions.FORWARDS,5,.5);
        Move(directions.RIGHT,50,.5);
        Move(directions.CLOCKWISE,10,.4);
        HS_Out(.4,200);
        VS_Up(.5,-2000);
        servoBucket.setPosition(0);
        sleep(1000);
        Move(directions.FORWARDS,10,.5);
        Move(directions.CLOCKWISE,10,.5);
        Move(directions.FORWARDS,90,.5);
        servoBucket.setPosition(.3);
        VS_Up(.5,2000);
        */
    }
}
