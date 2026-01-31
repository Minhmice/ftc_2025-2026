package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadController {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public double driver_gamepad_x, driver_gamepad_y, driver_gamepad_rotate;
    public boolean shooter_bumper_left, shooter_bumper_right;
    public double shooter_turret_rotate;

    public GamepadController(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void update_gamepad() {
        // Gamepad 1 (lái xe)
        driver_gamepad_x = gamepad1.left_stick_x;
        driver_gamepad_y = -gamepad1.left_stick_y; // Đảo ngược trục Y là phổ biến
        driver_gamepad_rotate = gamepad1.right_stick_x;

        // Gamepad 2 (điều khiển phụ)
        shooter_bumper_left = gamepad2.left_bumper;
        shooter_bumper_right = gamepad2.right_bumper;
        shooter_turret_rotate = gamepad2.right_stick_x; // Giả sử joystick phải của gamepad 2 xoay tháp pháo
    }
}
