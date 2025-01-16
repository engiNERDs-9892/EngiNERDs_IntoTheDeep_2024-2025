package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class leftAutononmous extends myLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        servoArm.setPosition(myConstants.ARM_UP);
        odo.resetPosAndIMU();
        waitForStart();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivePower(0.5);
        motorLL.setTargetPosition(5300);
        motorRR.setTargetPosition(5300);
        motorLL.setPower(0.6);
        motorRR.setPower(0.6);
        motorLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Forward(250);
        Counterclockwise_1(90);
        Right(-410);
        TurnTo(90);
        Forward(1100);
        TurnTo(90);
        Right(-100);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        sleep(500);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        sleep(500);
        Right(100);
        Forward(-200);
        motorLL.setTargetPosition(0);
        motorRR.setTargetPosition(0);
        motorLL.setPower(0.6);
        motorRR.setPower(0.6);
        Forward(-800);
        Right(1000);
        TurnTo(0);

        Right(5000);
        setDrivePower(0.2);
        Forward(-500);
        servoClawLeft.setPosition(myConstants.servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(myConstants.servoPositions.CLAW_RIGHT_OPEN);
       /*setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setTargetPosition(0);
        motorLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(0.5);
        motorLL.setPower(0.4);

        sleep(500);
        motorLL.setTargetPosition(4100);
        Forward(250);
        Right(-1025);
        Counterclockwise(-490);
        Right(-290);
        Forward(-270);
        while(motorLL.isBusy())sleep(100);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        sleep(500);
        Right(290);
        Forward(280);
        motorLL.setTargetPosition(200);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        Counterclockwise(490);
        setDrivePower(0.8);
        Right(1200);
        Forward(-250);
        Forward(1000);
        Right(4800);
        setDrivePower(0.2);
        Forward(-700);
        servoArm.setPosition(servoPositions.ARM_UP);
        sleep(200);
        */
    }

}
