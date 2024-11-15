package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.myLinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class leftAutononmous extends myLinearOpMode {
    @Override
    public void runOpMode(){
        super.runOpMode();
        waitForStart();
        Forward(100);
        sleep(1000);
    }
}
