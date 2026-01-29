package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(group = "TEST")
public class SortingTest extends LinearOpMode {

    private DcMotor motor_sorting;

    private static final double GOBILDA_5202_TICKS_PER_REV = 537.7;
    private static final double DEGREES_PER_TICK = 360.0 / GOBILDA_5202_TICKS_PER_REV;

    private static final int TICKS_FOR_60_DEGREES = (int) (60 / DEGREES_PER_TICK);
    boolean pressed;

    @Override
    public void runOpMode() {
        motor_sorting = hardwareMap.get(DcMotor.class, "motor_sorting");
        motor_sorting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad2.dpad_left && !pressed) {
                rotate_60_degrees(true, 0.01);
                pressed = true;
            } else {
                pressed = false;
            }
        }
    }
    public void rotate_60_degrees(boolean clockwise, double power) {

        if (!motor_sorting.isBusy()) {
            int direction = clockwise ? 1 : -1;
            int currentPosition = motor_sorting.getCurrentPosition();
            int targetPosition = currentPosition + (direction * TICKS_FOR_60_DEGREES);

            motor_sorting.setTargetPosition(targetPosition);
            motor_sorting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor_sorting.setPower(Math.abs(power));
        }
    }
}
