package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TagProcessing;

@TeleOp(name = "Test AprilTag", group = "Test")
public class Test extends LinearOpMode {

    RobotHardware robot;
    TagProcessing tagProcessing;
    
    // --- LOGIC MỚI: Thêm bộ đếm thời gian cho hiệu ứng rung ---
    private final ElapsedTime rumbleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.initAprilTag();
        tagProcessing = new TagProcessing(robot);

        telemetry.addLine("AprilTag Initialization Complete. Ready to start.");
        telemetry.update();

        waitForStart();
        
        // Bắt đầu đếm giờ sau khi nhấn Start
        rumbleTimer.reset();

        while (opModeIsActive()) {
            // Cập nhật bộ xử lý AprilTag, giả sử đội Xanh (team_color = 2)
            tagProcessing.update(2);

            telemetry.addData("Tag Detected", tagProcessing.isDetected());
            telemetry.addData("Distance to Center", "%.2f inches", tagProcessing.getDistanceToCenter());
            // Chỉ kiểm tra và thực hiện rung nếu đã qua 500ms
            if (rumbleTimer.milliseconds() > 500) {
                if (tagProcessing.isDetected()) {
                    // Lấy giá trị khoảng cách ngang (tính bằng inch)
                    double distanceToCenter = tagProcessing.getDistanceToCenter();

                    // Rung tay cầm trong 50ms dựa trên vị trí
                    // Ngưỡng 2.0 inch là một ví dụ, bạn có thể điều chỉnh
                    if (distanceToCenter > 50.0) {
                        gamepad1.rumble(0, 0.2, 50); // Rung bên phải nếu tag ở bên phải
                    } else if (distanceToCenter < -50.0) {
                        gamepad1.rumble(0.2, 0, 50); // Rung bên trái nếu tag ở bên trái
                    } else {
                        gamepad1.rumble(0.2, 0.2, 50); // Rung cả hai nếu ở giữa
                    }
                    
                    // Đặt lại bộ đếm sau khi đã rung
                    rumbleTimer.reset();
                }
            }
            
            // Hiển thị Telemetry liên tục
            if(tagProcessing.isDetected()){
                telemetry.addData("Bearing", "%.2f degrees", tagProcessing.getBearing());
                telemetry.addData("Range", "%.2f inches", tagProcessing.getRangeInches());
            }
            telemetry.update();

            // Giữ một khoảng sleep nhỏ để tránh lãng phí CPU
            sleep(20);
        }
    }
}
