package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class motorsController {
    /**
     * Default values for PID coefficients
     */
    public static final double P = 0.005, I = 0, D = 0.0004;//Default values

    private Telemetry telemetry;
    private PIDController controller;
    public boolean useTelemetry;
    private String telemetryName;
    private boolean isActive;
    private DcMotor[] motors;
    private int offset; // Offset for encoder values
    private double target;

    public void setArmFactors(double armFactor, double armFactor2) {
        this.armFactor = armFactor;
        this.armFactor2 = armFactor2;
    }

    private double armFactor;
    private double armFactor2;

    public boolean isActive() {
        return isActive;
    }

    public void useTelemetry(Telemetry telemetry, String telemetryName) {
        this.telemetryName = telemetryName;
        this.telemetry = telemetry;
        this.useTelemetry = true;
    }

    public void useTelemetry() {
        this.useTelemetry = false;
    }

    public void setPosition(int position) {
        offset = position - getCurrentPosition();
    }

    public int getPosition() {
        return getCurrentPosition() + offset;
    }

    private int getCurrentPosition() {
        return motors[0].getCurrentPosition();
    }

    public void setActive(boolean active) {
        isActive = active;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget() {
        return this.target;
    }

    public void setPID(double P, double I, double D) {
        this.controller.setPID(P, I, D);
    }

    public motorsController(DcMotor[] motors) {
        controller = new PIDController(P, I, D);
        this.motors = motors;
        target = 0;
        useTelemetry = false;
        isActive = false;
    }

    public motorsController(DcMotor motor) {
        controller = new PIDController(P, I, D);
        this.motors = new DcMotor[]{motor};
        target = 0;
        useTelemetry = false;
        isActive = false;
    }

    public motorsController(DcMotor[] motors, double P, double I, double D) {
        controller = new PIDController(P, I, D);
        this.motors = motors;
        target = 0;
        useTelemetry = false;
        isActive = false;
    }

    public motorsController(DcMotor motor, double P, double I, double D) {
        controller = new PIDController(P, I, D);
        this.motors = new DcMotor[]{motor};
        target = 0;
        useTelemetry = false;
        isActive = false;
    }

    public void update() {
        if (isActive) {
            int motorPosition = getPosition();
            double pid = controller.calculate(motorPosition, target);

            setPower(pid);

            if (useTelemetry) {
                // Telemetry making sure that everything is running as it should
                telemetry.addData(telemetryName + " Current Pos", motorPosition);
                telemetry.addData(telemetryName + " pid", pid);
                telemetry.addData(telemetryName + " Target Pos", target);
            }
        }
    }

    public void setPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void setAdjustedPower(double power) {
        double adjustedPower = power;
        if (armFactor != 0) {
            adjustedPower += armFactor * Math.cos(getPosition() * armFactor2 + 0.0);
        }
        setPower(adjustedPower);
    }
}
