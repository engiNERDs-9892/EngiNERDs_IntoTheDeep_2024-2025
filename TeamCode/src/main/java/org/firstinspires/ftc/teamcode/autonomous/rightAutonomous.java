package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;

@Autonomous
public class rightAutonomous extends myLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_CLOSED);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_CLOSED);
        servoArm.setPosition(myConstants.ARM_UP);
        odo.resetPosAndIMU();
        waitForStart();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivePower(0.5);
        Right(1200);
        Forward(200);
        Counterclockwise_1(-90);
        Right(200);
        Forward(100);
        //servoClawLeft.setPosition(myConstants.servoPositions.CLAW_LEFT_OPEN);
        //servoClawRight.setPosition(myConstants.servoPositions.CLAW_RIGHT_OPEN);
        //servoArm.setPosition(servoPositions.ARM_DOWN);
        sleep(200);
    }
}
