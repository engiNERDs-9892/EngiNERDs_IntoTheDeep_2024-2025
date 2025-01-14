package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.edgeButton;
import org.firstinspires.ftc.teamcode.repeatButton;

@TeleOp(group = "Testing")
public class testServos extends LinearOpMode {
    public static Servo[] servoList = new Servo[12];
    public static String[] servoNames = {"servoClawRight", "servoClawLeft", "servoArm", "servoWrist"};
    //public static String[] servoNames = {"servo0", "servo1", "servo2", "servo3", "servo4", "servo5"};
    public static int currentServo;
    public static double increment;

    @Override
    public void runOpMode(){
        for(int i = 0; i < servoNames.length; i++){
            servoList[i] = hardwareMap.get(Servo.class, servoNames[i]);
            servoList[i].setPosition(0.5);
        }
        currentServo = 0;
        increment = 0.05;
        edgeButton next = new edgeButton() {
            @Override
            public void trigger() {
                currentServo += 1;
                if(currentServo >= servoNames.length){
                    currentServo = 0;
                }
            }
        };
        edgeButton prev = new edgeButton() {
            @Override
            public void trigger() {
                currentServo -= 1;
                if(currentServo < 0){
                    currentServo = servoNames.length - 1;
                }
            }
        };
        repeatButton forward = new repeatButton() {
            @Override
            public void trigger() {
                Servo servo = servoList[currentServo];
                servo.setPosition(servo.getPosition() + increment);
            }
        };
        repeatButton backward = new repeatButton() {
            @Override
            public void trigger() {
                Servo servo = servoList[currentServo];
                servo.setPosition(servo.getPosition() - increment);
            }
        };
        telemetry.addLine("Controls\n  Bumpers to switch between servos\n  dpad to change servo positions");
        telemetry.addLine("Controls");
        telemetry.addLine("  Bumpers .to switch between servos");
        telemetry.addLine("  dpad to change servo positions");

        telemetry.addLine("Servo names  -  Default position is 0.5");
        for(int i = 0; i < servoNames.length; i++){
            telemetry.addData(servoNames[i], servoList[i].getPosition());
        }
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            next.update(gamepad1.left_bumper);
            prev.update(gamepad1.right_bumper);
            forward.update(gamepad1.dpad_right);
            backward.update(gamepad1.dpad_left);
            if(gamepad1.a){
                increment = 0.2;
            }
            if(gamepad1.b){
                increment = 0.05;
            }
            if(gamepad1.x){
                increment = 0.01;
            }
            if(gamepad1.y){
                increment = 0.005;
            }
            telemetry.addData("\u0394", increment);
            telemetry.addData("Current Servo", "%s %f", servoNames[currentServo], servoList[currentServo].getPosition());
            telemetry.addLine("Servo positions");
            for(int i = 0; i < servoNames.length; i++){
                telemetry.addData(servoNames[i], servoList[i].getPosition());
            }
            telemetry.update();
        }
    }
}
