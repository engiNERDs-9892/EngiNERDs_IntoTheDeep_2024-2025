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
    public static int SLIDE_AUTO_FRONT_SPECEMIN = 2000;
    public static int SLIDE_AUTO_FRONT_SPECEMIN_PLAY = 1000;
    public static int SLIDE_TOP = 5300;
    public static int SLIDE_HIGH_CHAMBER = 3700;
    public static int SLIDE_HIGH_CHAMBER_PLAY = 2960;
    public static final int FARM_UP = 10;
    public static final int FARM_DOWN = 340;
    public static final int FARM_AUTO_SAMPLE_GRAB = 380-10;
    public static int FARM_AUTO_FRONT_SPECEMIN = 60;
    public static int BARN_DOWN = 0;
    //public static int BARN_UP;


    /*Circle direction
    Left	-	X too big
    Right	-	X too small
    Up  	-	Y too big
    Down	-	Y too small
    */

    public static class servoPositions {
        public static final double CLAW_LEFT_OPEN = 0.8;
        public static final double CLAW_RIGHT_OPEN = 0.8;
        public static final double CLAW_LEFT_CLOSED = 0.485;
        public static final double CLAW_RIGHT_CLOSED = 0.485;

        public static final double CLAW_LEFT_2_OPEN = 0.8;
        public static final double CLAW_RIGHT_2_OPEN = 0.8;
        public static final double CLAW_LEFT_2_CLOSED = 0.465;
        public static final double CLAW_RIGHT_2_CLOSED = 0.465;
        public static final double CLAW_LEFT_2_LOOSE = 0.55;
        public static final double CLAW_RIGHT_2_LOOSE = 0.5;
        public static final double WRIST_A = 0.5;
        public static final double WRIST_B = 0.825;

    }
}
