package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Movement {
    private final RobotHardware robot;
    private final GamepadController gamepadController;

    private double deadzone = 0.1;
    private double maxSpeed = 1.0;

    // ===== Commands for Odometry (ROBOT FRAME) =====
    // Conventions expected by Odometry.setDriveCommand():
    // forward: +forward
    // strafe : +left
    // turn   : +CCW
    private double cmdForward = 0.0;
    private double cmdStrafe  = 0.0;
    private double cmdTurn    = 0.0;

    public Movement(RobotHardware robot, GamepadController gamepadController) {
        this.robot = robot;
        this.gamepadController = gamepadController;

        robot.motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor_3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor_4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move_robot() {
        // IMU yaw (rad). Assume +CCW
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.RADIANS);

        // Read sticks (after deadzone)
        // Your assumption:
        // driver_gamepad_x: strafe right +
        // driver_gamepad_y: forward up +
        // driver_gamepad_rotate: right + (usually CW+)
        double x = applyDeadzone(gamepadController.driver_gamepad_x);        // right +
        double y = applyDeadzone(gamepadController.driver_gamepad_y);        // forward +
        double rotate = applyDeadzone(gamepadController.driver_gamepad_rotate); // right +

        // ===== Save ROBOT-FRAME intent for Odometry =====
        // Odometry expects: strafe +left, turn +CCW
        cmdForward = y;          // forward +
        cmdStrafe  = -x;         // left + = -right
        cmdTurn    = -rotate;    // CCW + = -CW (common). If your turn is reversed, flip sign here.

        // ===== Field-centric transform for driving =====
        // We want to drive based on field, so rotate joystick (robot frame) by -heading
        // Using x=right+, y=forward+
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double rotatedX = x * cos + y * sin;     // right +
        double rotatedY = -x * sin + y * cos;    // forward +

        // ===== Mecanum mixing (robot-centric power) =====
        double frontLeftPower  = rotatedY + rotatedX + rotate;
        double frontRightPower = rotatedY - rotatedX - rotate;
        double backLeftPower   = rotatedY - rotatedX + rotate;
        double backRightPower  = rotatedY + rotatedX - rotate;

        // Normalize to maxSpeed
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower),  Math.abs(backRightPower))
        );

        if (maxPower > 1e-6) {
            double scale = 1.0;
            if (maxPower > maxSpeed) scale = maxSpeed / maxPower;

            frontLeftPower  *= scale;
            frontRightPower *= scale;
            backLeftPower   *= scale;
            backRightPower  *= scale;
        } else {
            frontLeftPower = 0;
            frontRightPower = 0;
            backLeftPower = 0;
            backRightPower = 0;
        }

        // Motor mapping: 1 FL, 2 BL, 3 FR, 4 BR
        robot.motor_1.setPower(frontLeftPower);
        robot.motor_2.setPower(backLeftPower);
        robot.motor_3.setPower(frontRightPower);
        robot.motor_4.setPower(backRightPower);
    }

    private double applyDeadzone(double value) {
        return (Math.abs(value) < deadzone) ? 0.0 : value;
    }

    // ===== Getters for Odometry =====
    public double getCmdForward() { return cmdForward; }
    public double getCmdStrafe()  { return cmdStrafe; }
    public double getCmdTurn()    { return cmdTurn; }

    // Tuning
    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = Math.max(0.1, Math.min(1.0, maxSpeed));
    }

    public void setDeadzone(double deadzone) {
        this.deadzone = Math.max(0.0, Math.min(0.5, deadzone));
    }
}
