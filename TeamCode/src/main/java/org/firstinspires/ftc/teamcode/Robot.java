package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Robot {
    public DcMotor motorFR = null; // Front Right
    public DcMotor motorFL = null; // Front Left
    public DcMotor motorBR = null; // Back Right
    public DcMotor motorBL = null; // Back Left

    public DcMotorEx motorSlider; // Slider
    public DcMotorEx motorArm; // Elbow or Arm
    public DcMotorEx motorHang; // Hanging

    public Servo servoCR; // Claw Right
    public Servo servoCL; // Claw left
    public Servo servoDrone; // Drone
    public Telemetry telemetry;
    public BHI260IMU imu;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motorFL = hardwareMap.get(DcMotor.class, "front_left");
        motorFR = hardwareMap.get(DcMotor.class, "front_right");
        motorBL = hardwareMap.get(DcMotor.class, "back_left");
        motorBR = hardwareMap.get(DcMotor.class, "back_right");
        // change the direction for the FR and BR so that all positive direction is forward
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlider = hardwareMap.get(DcMotorEx.class, "sliders");
        motorArm = hardwareMap.get(DcMotorEx.class, "elbow");
        motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        motorHang = hardwareMap.get(DcMotorEx.class, "hanging");
        servoCL = hardwareMap.get(Servo.class, "claw_left");
        servoCR = hardwareMap.get(Servo.class, "claw_right");
        servoDrone = hardwareMap.get(Servo.class, "drone");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        );
    }

    public double getYaw() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        return Yaw;
        //double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        //double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

}