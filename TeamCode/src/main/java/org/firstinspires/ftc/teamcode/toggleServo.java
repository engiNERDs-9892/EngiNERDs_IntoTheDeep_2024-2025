package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class toggleServo extends toggleButton{
    Servo servo;
    double onPosition;
    double offPosition;
    public toggleServo(Servo servo, double offPosition, double onPosition){
        super(Edge.RISING);
        this.servo = servo;
        this.onPosition = onPosition;
        this.offPosition = offPosition;
    }
    public toggleServo(Servo servo, double onPosition, double offPosition, Edge edge){
        super(edge);
        this.servo = servo;
        this.onPosition = onPosition;
        this.offPosition = offPosition;
    }
    @Override
    public void toggleOn() {
        servo.setPosition(onPosition);
    }

    @Override
    public void toggleOff() {
        servo.setPosition(offPosition);
    }
}
