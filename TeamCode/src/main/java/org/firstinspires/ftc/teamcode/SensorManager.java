package org.firstinspires.ftc.teamcode;

public class SensorManager {

    private RobotHardware robot;

    public SensorManager(RobotHardware robot) {
        this.robot = robot;
    }

    /** Slot không còn đọc màu (color sensor đã bỏ). Luôn trả về 0. */
    public int get_artifact_color(int slotIndex) {
        return 0;
    }

    /** IR sensor đã bỏ. Luôn trả về false (không phát hiện bóng). */
    public boolean get_ir_state() {
        return false;
    }

    public boolean isTurretAtLeftLimit() {
        return !robot.turret_end_stop_left.getState();
    }

    public boolean isTurretAtRightLimit() {
        return !robot.turret_end_stop_right.getState();
    }
}
