package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Test Odometry: lái robot, xem X/Y/theta, nút Back = reset pose (0,0).
 * Kiểm tra: đi thẳng ~30cm → Y tăng ~30, X ~0; xoay 90° → theta ~90.
 */
@TeleOp(name = "Odometry Test", group = "TEST")
public class OdometryTest extends LinearOpMode {

    RobotHardware robot;
    GamepadController gamepadController;
    Movement movement;
    Odometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        gamepadController = new GamepadController(gamepad1, gamepad2);
        movement = new Movement(robot, gamepadController);
        odometry = new Odometry(robot, 0, 0);

        movement.setMaxSpeed(0.4);
        movement.setDeadzone(0.1);

        waitForStart();

        boolean lastBack = false;
        while (opModeIsActive()) {
            gamepadController.update_gamepad();

            if (gamepad1.back && !lastBack) {
                odometry.resetPose(0, 0);
            }
            lastBack = gamepad1.back;

            odometry.setDriveCommand(
                    movement.getCmdForward(),
                    movement.getCmdStrafe(),
                    movement.getCmdTurn()
            );
            odometry.update();
            movement.move_robot();

            telemetry.addData("X (cm)", "%.2f", odometry.getX());
            telemetry.addData("Y (cm)", "%.2f", odometry.getY());
            telemetry.addData("Theta (deg)", "%.1f", odometry.getTheta(AngleUnit.DEGREES));
            telemetry.addData("ScrubX", "%.2f", odometry.getScrubX());
            telemetry.addData("ScrubY", "%.2f", odometry.getScrubY());
            telemetry.addLine("Back = reset pose (0,0)");
            telemetry.update();
        }
    }
}
