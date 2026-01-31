package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SortIndexer;

/**
 * Test sort bóng: servo_sort (REV Smart Servo 360°) + SortIndexer.
 * Dpad Left = quay 1 step CW, Dpad Right = quay 1 step CCW.
 * Chỉnh TIME_MS_SLOT0_TO_SLOT1, TIME_MS_SLOT1_TO_SLOT2, TIME_MS_SLOT2_TO_SLOT0 trong SortIndexer để căn 3 ô.
 */
@TeleOp(group = "TEST")
public class SortingTest extends LinearOpMode {

    private RobotHardware robot;
    private SortIndexer sortIndexer;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        sortIndexer = new SortIndexer(robot.servo_sort);

        waitForStart();
        while (opModeIsActive()) {
            sortIndexer.update();
            boolean left = gamepad2.dpad_left;
            boolean right = gamepad2.dpad_right;
            if (left && !lastLeft && sortIndexer.isIdle()) {
                sortIndexer.requestRotate(true, 1);
            }
            if (right && !lastRight && sortIndexer.isIdle()) {
                sortIndexer.requestRotate(false, 1);
            }
            lastLeft = left;
            lastRight = right;
            telemetry.addData("Slot", sortIndexer.getCurrentSlot());
            telemetry.addData("Idle", sortIndexer.isIdle());
            telemetry.update();
        }
    }
}
