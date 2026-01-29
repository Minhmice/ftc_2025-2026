package org.firstinspires.ftc.teamcode;

public class SensorManager {

    private RobotHardware robot;

    public SensorManager(RobotHardware robot) {
        this.robot = robot;
    }

    // --- PHƯƠNG THỨC ĐÃ SỬA ĐỔI: Lấy màu từ một khe cắm cụ thể ---
    /**
     * Trả về màu của artifact tại một khe cắm được chỉ định.
     * @param slotIndex Chỉ số của cảm biến (0, 1, hoặc 2).
     * @return 1 cho Xanh lá, 2 cho Tím, 0 cho không xác định.
     */
    public int get_artifact_color(int slotIndex) {
        // Lấy dữ liệu màu đã được chuẩn hóa từ cảm biến được chỉ định
        int[] colors = robot.getNormalizedColors(slotIndex);
        int red = colors[0];
        int green = colors[1];
        int blue = colors[2];

        // Ngưỡng phát hiện màu Xanh lá
        if (green > red && green > blue && green >= 100) {
            return 1; // 1 là Xanh lá
        }

        // Ngưỡng phát hiện màu Tím (Đỏ và Xanh dương cùng cao)
        if (red > green && blue > green) {
            return 2; // 2 là Tím
        }
        
        return 0; // Không xác định
    }

    public boolean get_ir_state() {
        return robot.ir_sensor.getState();
    }

    public boolean isTurretAtLeftLimit() {
        return !robot.turret_end_stop_left.getState();
    }

    public boolean isTurretAtRightLimit() {
        return !robot.turret_end_stop_right.getState();
    }
}
