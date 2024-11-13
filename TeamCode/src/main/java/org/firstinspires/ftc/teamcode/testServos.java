package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServos extends LinearOpMode {
    public static Servo[] servoList = new Servo[12];
    public static String[] servoNames = {"servoBucket", "servoLift", "servoArm", "servo"};
    //public static String[] servoNames = {"servo0", "servo1", "servo2", "servo3", "servo4", "servo5"};
    public static int currentServo;

    @Override
    public void runOpMode(){
        for(int i = 0; i < servoNames.length; i++){
            servoList[i] = hardwareMap.get(Servo.class, servoNames[i]);
            servoList[i].setPosition(0);
        }
        currentServo = 0;
        edgeButton a = new edgeButton() {
            @Override
            public void trigger() {
                currentServo += 1;
                if(currentServo >= servoNames.length){
                    currentServo = 0;
                }
            }
        };
        edgeButton b = new edgeButton() {
            @Override
            public void trigger() {
                currentServo -= 1;
                if(currentServo < 0){
                    currentServo = servoNames.length - 1;
                }
            }
        };
        repeatButton x = new repeatButton() {
            @Override
            public void trigger() {
                Servo servo = servoList[currentServo];
                servo.setPosition(servo.getPosition() + 0.05);
            }
        };
        repeatButton y = new repeatButton() {
            @Override
            public void trigger() {
                Servo servo = servoList[currentServo];
                servo.setPosition(servo.getPosition() - 0.05);
            }
        };
        waitForStart();
        while(opModeIsActive()){
            a.update(gamepad1.a);
            b.update(gamepad1.b);
            x.update(gamepad1.x);
            y.update(gamepad1.y);
            telemetry.addLine("Servo positions");
            for(int i = 0; i < servoNames.length; i++){
                telemetry.addData(servoNames[i], servoList[i].getPosition());
            }
            telemetry.addData("Current servo", currentServo);
            telemetry.update();
        }
    }
}
