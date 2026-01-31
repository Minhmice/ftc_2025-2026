package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoRunner;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * DECODE 2026 – Auto Red Right.
 * 30s: nhặt bóng, sắp xếp đúng màu (MOTIF), bắn 9 bóng vào goal.
 */
@Autonomous(name = "AUTO RED RIGHT", group = "DECODE 2026")
public class AutoRedRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        // Red Right: chỉnh theo Competition Manual (khác Red Left nếu cần)
        double startX = 0, startY = 0;
        double intakeX = 0, intakeY = 50;
        double shootX = 70, shootY = 50;
        double parkX = 70, parkY = 70;

        AutoRunner runner = new AutoRunner(robot, this, 1,
                startX, startY, intakeX, intakeY, shootX, shootY, parkX, parkY);

        telemetry.addData("Alliance", "RED RIGHT");
        telemetry.addLine("Press START. 30s: collect, sort, shoot 9.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        runner.run30Seconds();
    }
}
