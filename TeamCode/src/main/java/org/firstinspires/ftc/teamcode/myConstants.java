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
    public static double X_OFFSET = 139;
    public static double Y_OFFSET = -158;
    public static double FARM_RADIANS_PER_TICK = 0.004408f;
    public static double BARN_RADIANS_PER_TICK = 0.004408f; // 2 * pi / 1425.1 // 50.9:1
    public static int SLIDE_BOTTOM = 60;
    public static int SLIDE_AUTO_SAMPLE_GRAB = 100;
    public static int SLIDE_AUTO_SPECEMIN_GRAB = 1185;
    public static int SLIDE_AUTO_ASCENT = 1700;
    public static int SLIDE_HIGH_BASKET = 5300;
    public static int SLIDE_TOP = 5300;
    public static int SLIDE_HIGH_CHAMBER = 3700;
    public static int SLIDE_HIGH_CHAMBER_PLAY = 2960;
    public static final int ARM_UP = 10;
    public static final int ARM_DOWN = 340;
    public static final int ARM_AUTO_SAMPLE_GRAB = 380-10;


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
        public static final double CLAW_LEFT_CLOSED = 0.43; //.47
        public static final double CLAW_RIGHT_CLOSED = 0.575; //.535

        public static final double CLAW_LEFT_2_OPEN = 0.75;
        public static final double CLAW_RIGHT_2_OPEN = 0.3;
        public static final double CLAW_LEFT_2_CLOSED = 0.535-0.005;
        public static final double CLAW_RIGHT_2_CLOSED = 0.515+0.05;
        public static final double CLAW_LEFT_2_LOOSE = 0.55;
        public static final double CLAW_RIGHT_2_LOOSE = 0.5;
        public static final double WRIST_A = 0.5;
        public static final double WRIST_B = 0.825;

    }
}
