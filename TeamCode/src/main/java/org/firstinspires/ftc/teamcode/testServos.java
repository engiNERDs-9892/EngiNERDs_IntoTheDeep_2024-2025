package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class testServos extends LinearOpMode {
    public static Servo[] servoList;
    public static String[] servoNames = {"servoBucket", "servoLift", "servoArm", "servo"};
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
                if(currentServo >= servoList.length){
                    currentServo = 0;
                }
            }
        };
        edgeButton b = new edgeButton() {
            @Override
            public void trigger() {
                currentServo -= 1;
                if(currentServo < 0){
                    currentServo = servoList.length - 1;
                }
            }
        };
        edgeButton x = new edgeButton() {
            @Override
            public void trigger() {
                Servo servo = servoList[currentServo];
                servo.setPosition(servo.getPosition() + 0.05);
            }
        };
        edgeButton y = new edgeButton() {
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
            for(int i = 0; i<4; i++){
                telemetry.addData(servoNames[i], servoList[i].getPosition());
            }
            telemetry.addData("Current servo", currentServo);
            telemetry.update();
        }
    }
}
