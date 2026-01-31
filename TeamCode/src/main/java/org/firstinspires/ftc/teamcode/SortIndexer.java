package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class SortIndexer {

    // ====== Servo spec / config ======
    // Servo angular range (spec): 270 deg
    private static final double SERVO_RANGE_DEG = 270.0;

    // Speed: 0.14s / 60deg @6V  -> seconds per degree:
    private static final double SEC_PER_DEG = 0.14 / 60.0;

    // Pause time between steps
    private static final double PAUSE_SEC = 0.5;

    // Add a small safety margin so servo has time to settle
    private static final double MOVE_MARGIN_SEC = 0.05;

    // ====== Calibration (IMPORTANT) ======
    // "0 deg" position of your mechanism. Start with 0.0, then tune.
    // If your mechanism's "home angle" corresponds to servo pos 0.5, set ZERO_POS = 0.5.
    private static final double ZERO_POS = 0.0;

    // If servo direction is reversed mechanically, set DIR = -1
    private static final int DIR = +1;

    // ====== Cycle steps ======
    private static final double STEP_LEFT_DEG  = 120.0;
    private static final double STEP_RIGHT_DEG = 60.0;

    private enum Mode { NONE, LEFT, RIGHT }
    private enum State { IDLE, MOVING, PAUSING }

    private final Servo servo;

    private Mode mode = Mode.NONE;
    private State state = State.IDLE;

    // We track last commanded angle (estimate). Servo doesn't report true angle.
    private double currentAngleDeg = 0.0;
    private double targetAngleDeg = 0.0;

    private final ElapsedTime timer = new ElapsedTime();
    private double plannedMoveSec = 0.0;

    public SortingServoCycle(Servo servo) {
        this.servo = servo;
        // Initialize at currentAngleDeg = 0 -> position ZERO_POS
        servo.setPosition(angleDegToPos(currentAngleDeg));
    }

    /**
     * Call this every loop (TeleOp).
     * @param holdLeft  gamepad2.dpad_left
     * @param holdRight gamepad2.dpad_right
     */
    public void update(boolean holdLeft, boolean holdRight) {
        Mode newMode = Mode.NONE;
        if (holdLeft && !holdRight) newMode = Mode.LEFT;
        else if (holdRight && !holdLeft) newMode = Mode.RIGHT;

        // If mode changed (pressed/released/switch), reset state cleanly
        if (newMode != mode) {
            mode = newMode;
            state = (mode == Mode.NONE) ? State.IDLE : State.MOVING;
            timer.reset();

            if (mode == Mode.NONE) {
                // Stop cycling: just hold current position
                servo.setPosition(angleDegToPos(currentAngleDeg));
                return;
            }
        }

        if (mode == Mode.NONE) return;

        double stepDeg = (mode == Mode.LEFT) ? STEP_LEFT_DEG : STEP_RIGHT_DEG;

        switch (state) {
            case MOVING: {
                // If just entered MOVING (timer ~ 0), compute a new target and command servo
                if (timer.seconds() < 1e-6) {
                    targetAngleDeg = wrapAngleDeg(currentAngleDeg + DIR * stepDeg);
                    servo.setPosition(angleDegToPos(targetAngleDeg));

                    plannedMoveSec = Math.abs(stepDeg) * SEC_PER_DEG + MOVE_MARGIN_SEC;
                }

                // After the estimated move time, we consider it reached
                if (timer.seconds() >= plannedMoveSec) {
                    currentAngleDeg = targetAngleDeg;  // commit
                    state = State.PAUSING;
                    timer.reset();
                }
                break;
            }

            case PAUSING: {
                if (timer.seconds() >= PAUSE_SEC) {
                    state = State.MOVING;
                    timer.reset();
                }
                break;
            }

            default:
            case IDLE:
                // should not happen while mode != NONE
                state = State.MOVING;
                timer.reset();
                break;
        }
    }

    // ===== helpers =====

    private double angleDegToPos(double angleDeg) {
        // Map [0..270] deg -> [0..1] around ZERO_POS
        double pos = ZERO_POS + (angleDeg / SERVO_RANGE_DEG);
        return clamp(pos, 0.0, 1.0);
    }

    private double wrapAngleDeg(double angleDeg) {
        // keep in [0..SERVO_RANGE_DEG)
        double r = angleDeg % SERVO_RANGE_DEG;
        if (r < 0) r += SERVO_RANGE_DEG;
        return r;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Optional: if you want to set currentAngle when you "home" the mechanism
    public void setCurrentAngleDeg(double angleDeg) {
        currentAngleDeg = wrapAngleDeg(angleDeg);
        servo.setPosition(angleDegToPos(currentAngleDeg));
    }

    public double getCurrentAngleDeg() {
        return currentAngleDeg;
    }
}
