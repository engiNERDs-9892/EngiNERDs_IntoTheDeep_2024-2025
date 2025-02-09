package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Testing")
@Disabled
public class gamepadTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("Gamepad1 (Start + A)");
            telemetry.addData("Triggers", "L: %.2f R: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("Bumper", "L: %b R: %b", gamepad1.left_bumper, gamepad1.right_bumper);
            telemetry.addData("Buttons", "A: %b B: %b X: %b Y: %b", gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);
            telemetry.addData("DPad", "R: %b U: %b L: %b D: %b", gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left, gamepad1.dpad_down);
            telemetry.addData("Left Stick", "L/R: %.2f U/d: %.2f Button: %b", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_button);
            telemetry.addData("Right Stick", "L/R: %.2f U/d: %.2f Button: %b", gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_button);
            telemetry.addData("Other Buttons", "Back: %b Start: %b", gamepad1.back, gamepad1.start);
            telemetry.update();
        }
    }
}