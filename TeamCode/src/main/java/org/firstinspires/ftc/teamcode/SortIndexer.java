package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * SortIndexer - Điều khiển servo sort 360° (REV Smart Servo) theo thời gian để căn 3 ô bóng.
 * Mỗi "step" = quay từ ô hiện tại sang ô kế tiếp (0→1, 1→2, 2→0). Timing tùy chỉnh cho từng bước.
 */
public class SortIndexer {

    private final ServoImplEx servo;

    // ----- TODO: chỉnh trên robot cho đúng 3 ô (ms) -----
    /** Thời gian quay từ ô 1 sang ô 2. */
    public static long TIME_MS_SLOT0_TO_SLOT1 = 1000;
    /** Thời gian quay từ ô 2 sang ô 3. */
    public static long TIME_MS_SLOT1_TO_SLOT2 = 1000;
    /** Thời gian quay từ ô 3 về ô 1. */
    public static long TIME_MS_SLOT2_TO_SLOT0 = 1000;

    /** Thời gian quay nửa ô – căn bóng vào vị trí kicker (intake và kicker đối nhau). TODO: chỉnh trên robot. */
    public static long TIME_MS_HALF_SLOT = 500;

    /** Dpad manual: 120° theo 0.14 s/60° = 280 ms; nghỉ 0.5 s giữa các lần; 60° = 140 ms. */
    public static long TIME_MS_120_DEG = 280;
    public static long TIME_MS_60_DEG = 140;
    public static long TIME_MS_DPAD_REST = 500;

    /** Servo positional: range 270° (angle 0..270 -> position 0..1). */
    public static final double ANGLE_RANGE_DEG = 270.0;

    /** Continuous servo: 0.5 = stop, <0.5 = one dir, >0.5 = other (like Turret). */
    private static final double SERVO_POS_STOP = 0.5;
    private static final double SERVO_POS_CW = 0.25;
    private static final double SERVO_POS_CCW = 0.75;

    /** Ô hiện tại đang ở vị trí bắn (0, 1, 2). */
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

    /** Dpad Left held: set mỗi frame từ Main/ArtifactProcessing. */
    private boolean dpadLeftHeld = false;
    /** Dpad Right pressed (edge): request một lần 60°. */
    private boolean dpadRightRequested = false;

    /** Góc hiện tại khi điều khiển Dpad (positional servo 0..270°). Giữ tại góc mới, không set 0.5. */
    private double currentAngleDeg = 135.0;
    /** Góc đích khi đang ROTATING_120 hoặc ROTATING_60 (để cập nhật currentAngleDeg khi xong). */
    private double targetAngleDeg = 135.0;

    public SortIndexer(ServoImplEx servo) {
        this.servo = servo;
        servo.setPosition(SERVO_POS_STOP);
        currentAngleDeg = 135.0;
        servo.setPosition(angleToPosition(currentAngleDeg));
    }

    /** Góc (0..270°) -> position [0, 1] cho servo positional. */
    private double angleToPosition(double angleDeg) {
        double a = angleDeg;
        while (a < 0) a += ANGLE_RANGE_DEG;
        while (a > ANGLE_RANGE_DEG) a -= ANGLE_RANGE_DEG;
        return Math.max(0, Math.min(1, a / ANGLE_RANGE_DEG));
    }

    /** Wrap góc về [0, ANGLE_RANGE_DEG]. */
    private double wrapAngle(double angleDeg) {
        double a = angleDeg;
        while (a < 0) a += ANGLE_RANGE_DEG;
        while (a > ANGLE_RANGE_DEG) a -= ANGLE_RANGE_DEG;
        return a;
    }

    /** Yêu cầu quay N bước (mỗi bước = sang ô kế tiếp theo chiều cw/ccw). */
    public void requestRotate(boolean clockwise, int steps) {
        if (steps <= 0) return;
        if (pendingSteps == 0) {
            pendingClockwise = clockwise;
            pendingSteps = steps;
        } else {
            if (pendingClockwise == clockwise) pendingSteps += steps;
            else {
                pendingClockwise = clockwise;
                pendingSteps = steps;
            }
        }
    }

    /** Yêu cầu quay nửa ô (căn bóng vào kicker; không đổi currentSlot). */
    public void requestRotateHalf(boolean clockwise) {
        pendingHalfStep = true;
        pendingHalfClockwise = clockwise;
    }

    /** Main/ArtifactProcessing gọi mỗi frame khi đọc gamepad. */
    public void setDpadLeftHeld(boolean held) {
        dpadLeftHeld = held;
    }

    /** Main gọi một lần khi nhấn Dpad Right (edge). */
    public void requestDpadRight60() {
        dpadRightRequested = true;
    }

    /** Gọi mỗi vòng lặp. */
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
                state = State.IDLE;
            }
            return;
        }
        if (state == State.IDLE) {
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
            if (pendingSteps > 0) {
                startNextStep();
            } else if (pendingHalfStep) {
                startHalfStep();
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
        if (cw) return (from + 1) % 3;
        return (from + 2) % 3; // CCW = -1 mod 3
    }

    private long getDurationMs(int fromSlot, int toSlot) {
        if (fromSlot == 0 && toSlot == 1) return TIME_MS_SLOT0_TO_SLOT1;
        if (fromSlot == 1 && toSlot == 2) return TIME_MS_SLOT1_TO_SLOT2;
        if (fromSlot == 2 && toSlot == 0) return TIME_MS_SLOT2_TO_SLOT0;
        // Reverse direction: use same timing (assume symmetric)
        if (fromSlot == 1 && toSlot == 0) return TIME_MS_SLOT0_TO_SLOT1;
        if (fromSlot == 2 && toSlot == 1) return TIME_MS_SLOT1_TO_SLOT2;
        if (fromSlot == 0 && toSlot == 2) return TIME_MS_SLOT2_TO_SLOT0;
        return 1000;
    }

    public boolean isIdle() {
        if (state == State.ROTATING_120 || state == State.RESTING || state == State.ROTATING_60)
            return false;
        return state == State.IDLE && pendingSteps == 0 && !pendingHalfStep;
    }

    public int getCurrentSlot() {
        return currentSlot;
    }
}
