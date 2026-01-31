package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoRunner;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * DECODE 2026 – Auto Red Left.
 * 30s: nhặt bóng, sắp xếp đúng màu (MOTIF từ AprilTag 21/22/23), bắn 9 bóng vào goal.
 * Chỉnh waypoints theo sân (Competition Manual).
 */
@Autonomous(name = "AUTO RED LEFT", group = "DECODE 2026")
public class AutoRedLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        // Red Left: chỉnh theo Competition Manual
        double startX = 0, startY = 0;
        double intakeX = 0, intakeY = 50;
        double shootX = 70, shootY = 50;
        double parkX = 70, parkY = 70;

        AutoRunner runner = new AutoRunner(robot, this, 1,
                startX, startY, intakeX, intakeY, shootX, shootY, parkX, parkY);

        telemetry.addData("Alliance", "RED LEFT");
        telemetry.addLine("Press START. 30s: collect, sort, shoot 9.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        runner.run30Seconds();
    }
}
