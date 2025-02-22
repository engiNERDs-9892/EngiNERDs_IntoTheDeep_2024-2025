package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.opModeGroups.DEFAULT_TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opModeGroups;

@Autonomous(group = opModeGroups.auto.BASIC, preselectTeleOp = DEFAULT_TELEOP)
@Disabled
public class dummyAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive());
    }
}
