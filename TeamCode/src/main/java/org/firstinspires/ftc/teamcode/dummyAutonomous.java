package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous
public class dummyAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive());
    }
}
