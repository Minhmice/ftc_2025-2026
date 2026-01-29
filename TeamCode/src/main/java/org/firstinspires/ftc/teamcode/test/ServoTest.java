package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Servo Test (Robust)", group = "Test")
public class ServoTest extends LinearOpMode {
     // always works if config is correct
    private ServoImplEx servo;  // optional, only if available

    private double pos = 0.5;
    private boolean lastLeft = false, lastRight = false;

    @Override
    public void runOpMode() {
        // 1) Use generic Servo first (most reliable)
        servo = hardwareMap.get(ServoImplEx.class, "angle_servo");



        // 3) Only set PWM range if we really have ServoImplEx
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 20000));

        servo.setPosition(pos);

        telemetry.addLine("Servo Test (Robust)");
        telemetry.addLine("Gamepad2 Left Stick X: continuous control");
        telemetry.addLine("Dpad Left/Right: step -/+");
        telemetry.addLine("A: 0.0 | X: 0.5 | B: 1.0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Discrete jumps for hardware proof ---
            if (gamepad2.a) pos = 0.0;
            if (gamepad2.x) pos = 0.5;
            if (gamepad2.b) pos = 1.0;

            // --- Step control ---
            boolean left = gamepad2.dpad_left;
            boolean right = gamepad2.dpad_right;
            if (left && !lastLeft) pos -= 0.02;
            if (right && !lastRight) pos += 0.02;
            lastLeft = left;
            lastRight = right;

            // --- Continuous control (like your original) ---
            double stick = gamepad2.left_stick_x; // -1..1
            // If you want stick to override only when moved significantly:
            if (Math.abs(stick) > 0.05) {
                pos = (stick + 1.0) * 0.5; // -1..1 -> 0..1
            }

            pos = clamp(pos, 0.0, 1.0);

            servo.setPosition(pos);

            telemetry.addData("stickX", "%.3f", stick);
            telemetry.addData("cmd pos", "%.3f", pos);
            telemetry.addData("read pos", "%.3f", servo.getPosition());
            telemetry.update();
        }
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
