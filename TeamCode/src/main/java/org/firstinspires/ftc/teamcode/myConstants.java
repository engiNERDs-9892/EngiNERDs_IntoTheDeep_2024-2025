package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
public class myConstants {
    public static DcMotorSimple.Direction motorFL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction motorFR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction motorLR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction motorRR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection xEncoder_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection yEncoder_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static double X_OFFSET = 139;
    public static double Y_OFFSET = -158;
    /*Circle direction
    Left	-	X too big
    Right	-	X too small
    Up  	-	Y too big
    Down	-	Y too small
    */
}
