package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
public class myConstants {
    public static DcMotorSimple.Direction motorFL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction motorFR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction motorLR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction motorRR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection xEncoder_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yEncoder_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static double X_OFFSET = 139;//139
    public static double Y_OFFSET = -158 ;//-158
    public static int SLIDE_BOTTOM = 10;
    public static int SLIDE_TOP = 5500;

    /*Circle direction
    Left	-	X too big
    Right	-	X too small
    Up  	-	Y too big
    Down	-	Y too small
    */

    public static class servoPositions {
        public static final double ARM_INTAKE = 0.88;
        public static final double ARM_OUTPUT = 0.35;
        public static final double BUCKET_OUT = .15;
        public static final double BUCKET_IN = .6;
        public static final double CLAW_LEFT_OPEN = 0.85;
        public static final double CLAW_RIGHT_OPEN = 0.15;
        public static final double CLAW_LEFT_CLOSED = 0.51;
        public static final double CLAW_RIGHT_CLOSED = 0.49+0.1;
        public static final double ARM_UP = 0.525;
        public static final double ARM_DOWN = 0.2;

    }
}
