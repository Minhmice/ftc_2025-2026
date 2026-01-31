package org.firstinspires.ftc.teamcode;
 /**minhdeptraidacodefilenay */
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/** SortIndexer: servo sort 3 ô. requestRotate = continuous+timing; Dpad = positional (270°) nếu HW hỗ trợ. */
public class SortIndexer {

    private static final boolean USE_CONTINUOUS_FOR_DPAD = false;

    private final ServoImplEx servo;

    public static long TIME_MS_SLOT0_TO_SLOT1 = 1000;
    public static long TIME_MS_SLOT1_TO_SLOT2 = 1000;
    public static long TIME_MS_SLOT2_TO_SLOT0 = 1000;
    public static long TIME_MS_HALF_SLOT = 500;
    public static long TIME_MS_120_DEG = 280;
    public static long TIME_MS_60_DEG = 140;
    public static long TIME_MS_DPAD_REST = 500;
    public static final double ANGLE_RANGE_DEG = 270.0;
    private static final double SLOT_ANGLE_DEG = 120.0;
    private static final double SERVO_POS_STOP = 0.5;
    private static final double SERVO_POS_CW = 0.25;
    private static final double SERVO_POS_CCW = 0.75;
    private int currentSlot = 0;

    private enum State { IDLE, ROTATING, ROTATING_120, RESTING, ROTATING_60 }
    private State state = State.IDLE;
    private final ElapsedTime rotateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private long rotateDurationMs = 0;
    private int nextSlotAfterRotate = 0;

    private int pendingSteps = 0;
    private boolean pendingClockwise = true;

    private boolean pendingHalfStep = false;
    private boolean pendingHalfClockwise = true;
    private boolean isCurrentRotationHalf = false;
    private boolean dpadLeftHeld = false;
    private boolean dpadRightRequested = false;
    private double currentAngleDeg = 135.0;
    private double targetAngleDeg = 135.0;

    public SortIndexer(ServoImplEx servo) {
        this.servo = servo;
        servo.setPosition(SERVO_POS_STOP);
        currentAngleDeg = 135.0;
        servo.setPosition(angleToPosition(currentAngleDeg));
    }

    private double angleToPosition(double angleDeg) {
        double a = wrapAngle(angleDeg);
        return Math.max(0, Math.min(1, a / ANGLE_RANGE_DEG));
    }

    private int angleToSlot(double angleDeg) {
        double a = wrapAngle(angleDeg);
        return (int) (Math.round(a / SLOT_ANGLE_DEG) % 3);
    }

    private double wrapAngle(double angleDeg) {
        double r = angleDeg % ANGLE_RANGE_DEG;
        if (r < 0) r += ANGLE_RANGE_DEG;
        return Math.max(0, Math.min(ANGLE_RANGE_DEG, r));
    }

    public void requestRotate(boolean clockwise, int steps) {
        if (steps <= 0) return;
        if (pendingSteps == 0) { pendingClockwise = clockwise; pendingSteps = steps; }
        else if (pendingClockwise == clockwise) pendingSteps += steps;
        else { pendingClockwise = clockwise; pendingSteps = steps; }
    }

    public void requestRotateHalf(boolean clockwise) {
        pendingHalfStep = true;
        pendingHalfClockwise = clockwise;
    }

    public void setDpadLeftHeld(boolean held) { dpadLeftHeld = held; }
    public void requestDpadRight60() { dpadRightRequested = true; }

    public void update() {
        if (state == State.ROTATING) {
            if (rotateTimer.milliseconds() >= rotateDurationMs) {
                servo.setPosition(SERVO_POS_STOP);
                if (!isCurrentRotationHalf) currentSlot = nextSlotAfterRotate;
                state = State.IDLE;
                if (pendingSteps > 0) startNextStep();
                else if (pendingHalfStep) startHalfStep();
            }
            return;
        }
        if (state == State.ROTATING_120) {
            if (rotateTimer.milliseconds() >= TIME_MS_120_DEG) {
                currentAngleDeg = targetAngleDeg;
                currentSlot = angleToSlot(currentAngleDeg);
                if (dpadLeftHeld) {
                    state = State.RESTING;
                    rotateTimer.reset();
                } else {
                    state = State.IDLE;
                }
            }
            return;
        }
        if (state == State.RESTING) {
            if (rotateTimer.milliseconds() >= TIME_MS_DPAD_REST) {
                if (dpadLeftHeld) {
                    targetAngleDeg = wrapAngle(currentAngleDeg + 120);
                    servo.setPosition(angleToPosition(targetAngleDeg));
                    state = State.ROTATING_120;
                    rotateTimer.reset();
                } else {
                    state = State.IDLE;
                }
            }
            return;
        }
        if (state == State.ROTATING_60) {
            if (rotateTimer.milliseconds() >= TIME_MS_60_DEG) {
                currentAngleDeg = targetAngleDeg;
                currentSlot = angleToSlot(currentAngleDeg);
                state = State.IDLE;
            }
            return;
        }
        if (state == State.IDLE) {
            if (pendingSteps > 0) {
                startNextStep();
                return;
            }
            if (pendingHalfStep) {
                startHalfStep();
                return;
            }
            if (dpadRightRequested) {
                dpadRightRequested = false;
                targetAngleDeg = wrapAngle(currentAngleDeg - 60);
                servo.setPosition(angleToPosition(targetAngleDeg));
                rotateTimer.reset();
                state = State.ROTATING_60;
                return;
            }
            if (dpadLeftHeld) {
                targetAngleDeg = wrapAngle(currentAngleDeg + 120);
                servo.setPosition(angleToPosition(targetAngleDeg));
                rotateTimer.reset();
                state = State.ROTATING_120;
                return;
            }
        }
    }

    private void startNextStep() {
        if (pendingSteps <= 0) return;
        isCurrentRotationHalf = false;
        int nextSlot = nextSlotFrom(currentSlot, pendingClockwise);
        long durationMs = getDurationMs(currentSlot, nextSlot);
        servo.setPosition(pendingClockwise ? SERVO_POS_CW : SERVO_POS_CCW);
        rotateDurationMs = durationMs;
        nextSlotAfterRotate = nextSlot;
        rotateTimer.reset();
        state = State.ROTATING;
        pendingSteps--;
    }

    private void startHalfStep() {
        if (!pendingHalfStep) return;
        isCurrentRotationHalf = true;
        pendingHalfStep = false;
        servo.setPosition(pendingHalfClockwise ? SERVO_POS_CW : SERVO_POS_CCW);
        rotateDurationMs = TIME_MS_HALF_SLOT;
        nextSlotAfterRotate = currentSlot;
        rotateTimer.reset();
        state = State.ROTATING;
    }

    private int nextSlotFrom(int from, boolean cw) {
        return cw ? (from + 1) % 3 : (from + 2) % 3;
    }

    private long getDurationMs(int fromSlot, int toSlot) {
        if (fromSlot == 0 && toSlot == 1) return TIME_MS_SLOT0_TO_SLOT1;
        if (fromSlot == 1 && toSlot == 2) return TIME_MS_SLOT1_TO_SLOT2;
        if (fromSlot == 2 && toSlot == 0) return TIME_MS_SLOT2_TO_SLOT0;
        if (fromSlot == 1 && toSlot == 0) return TIME_MS_SLOT0_TO_SLOT1;
        if (fromSlot == 2 && toSlot == 1) return TIME_MS_SLOT1_TO_SLOT2;
        if (fromSlot == 0 && toSlot == 2) return TIME_MS_SLOT2_TO_SLOT0;
        return 1000;
    }

    public boolean isIdle() {
        return state == State.IDLE && pendingSteps == 0 && !pendingHalfStep;
    }

    public int getCurrentSlot() {
        return currentSlot;
    }
}
