package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.myConstants.ARM_UP;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_AUTO_SPECEMIN_GRAB;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.myConstants.SLIDE_HIGH_CHAMBER_PLAY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.myConstants.servoPositions;
import org.firstinspires.ftc.teamcode.myLinearOpMode;
import org.firstinspires.ftc.teamcode.teleop.EngiNERDs_Control;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "testing", preselectTeleOp = "EngiNERDs_Control")
@Disabled
public class distanceTester extends myLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();//Initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoClawLeft2.setPosition(servoPositions.CLAW_LEFT_2_CLOSED);
        servoClawRight2.setPosition(servoPositions.CLAW_RIGHT_2_CLOSED);
        servoClawLeft.setPosition(servoPositions.CLAW_LEFT_OPEN);
        servoClawRight.setPosition(servoPositions.CLAW_RIGHT_OPEN);
        motorFARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFARM.setTargetPosition(myConstants.ARM_UP);
        motorFARM.setPower(0.6);
        servoWrist.setPosition(servoPositions.WRIST_A);
        motorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLL.setPower(1.0);
        motorRR.setPower(1.0);
        //Trajectories
        TrajectorySequence trajectoryTest = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(24*4/0.68)
                .back(24*3/0.68)
                .strafeLeft(24*2/0.68)
                .build();
        waitForStart();//waitForStart();
        if (isStopRequested()) return;
        motorFARM.setTargetPosition(ARM_UP);
        motorFARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectorySequence(trajectoryTest);
    }
}
