package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Automove;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "AUTOMOVE", group = "TEST")
public class AutomoveTest extends LinearOpMode {

    RobotHardware robot;
    Odometry odometry;
    Automove automove;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        // Start pose (0,0). Theta = 0 (IMU đã bỏ)
        odometry = new Odometry(robot, 0, 0);

        automove = new Automove(robot, odometry, this);

        // Tuning an toàn để test
        automove.setGains(0.06, 2.2);                 // kPos, kTurn
        automove.setPowerLimits(0.25, 0.25, 0.12, 0.10); // maxTrans, maxTurn, minTrans, minTurn
        automove.setSlewRates(4.0, 6.0);

        // Pre-start telemetry
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Odom", "X=%.1f Y=%.1f", odometry.getX(), odometry.getY());
            telemetry.addData("Theta", "%.1f deg (no IMU)", Math.toDegrees(odometry.getThetaRad()));
            telemetry.addLine("Ready. Press START.");
            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) return;

        // (1) Đi tới (10cm, 0cm)
        automove.driveToPoint(10, 10);

        sleep(200);

        // (2) Xoay tại chỗ tới 90°
        automove.turnToHeading(90);

        sleep(200);

        // (3) Ví dụ: đi tới (10, 10) và giữ heading 90° trong khi chạy
        // automove.driveToPoint(10, 10, 90.0);

        // Kết thúc
        automove.stopMotors();
    }
}
