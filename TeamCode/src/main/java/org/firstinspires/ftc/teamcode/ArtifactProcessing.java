package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * ArtifactProcessing - TeleOp-safe (non-blocking)
 *
 * Slot mapping (per user):
 *  - slot[0] : vị trí bóng vừa được intake vào (cửa vào)
 *  - slot[1] : bên trái slot0 (nhìn theo hướng vào của bóng)
 *  - slot[2] : bên phải slot0 (nhìn theo hướng vào của bóng)
 *
 * Notes:
 *  - KHÔNG dùng while blocking trong TeleOp.
 *  - Mọi rotate đều chạy theo cơ chế "request -> execute khi motor idle".
 */
public class ArtifactProcessing {

    private final RobotHardware robot;
    private final SensorManager sensorManager;

    // 0 = empty/unknown; 1 = green; 2 = purple (theo SensorManager)
    public int[] artifact_slots = new int[]{0, 0, 0};

    // Queue 3 viên theo artifact_order (0 = none)
    public int[] artifact_queue = new int[]{0, 0, 0};

    public int artifact_order = 1;

    // Motor tick constants (GoBILDA 5202 series encoder ~537.7 ticks/rev)
    private static final double GOBILDA_5202_TICKS_PER_REV = 537.7;
    private static final double DEGREES_PER_TICK = 360.0 / GOBILDA_5202_TICKS_PER_REV;

    // Bạn đang dùng bước 60° (và gọi 2 lần để ra ~120°)
    private static final int TICKS_FOR_60_DEGREES = (int) Math.round(60.0 / DEGREES_PER_TICK);

    private static final double SORTING_POWER = 1.0;

    // Tolerance để coi motor đã xong (đề phòng isBusy() không nhả đúng)
    private static final int POSITION_TOLERANCE_TICKS = 10;

    // Debounce intake
    private static final long INTAKE_LATCH_TIMEOUT_MS = 800;

    // Pre-shoot timeout (fail-safe)
    private static final long PRE_SHOOT_TIMEOUT_MS = 2000;

    // Public telemetry flag (giữ nguyên thói quen dùng từ Main)
    public boolean sorting_busy = false;

    // Intake latch
    private boolean intakeLatched = false;
    private final ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Rotation request
    private int pendingSteps = 0; // each step = 60 deg
    private boolean pendingClockwise = true;

    // Pre-shoot state machine
    private enum ShootState { IDLE, ALIGNING_TO_SLOT1, ROTATE_TO_SHOOT_POS, READY_TO_KICK, POST_ROTATE_BACK, POST_WAIT }
    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ArtifactProcessing(RobotHardware robot, SensorManager sensorManager) {
        this.robot = robot;
        this.sensorManager = sensorManager;

        // Ensure sorting motor mode is valid for RUN_TO_POSITION
        robot.motor_sorting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor_sorting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor_sorting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Call mỗi vòng lặp TeleOp */
    public void update() {
        update_artifact_slot();

        // Update motor busy state (safe idle detection)
        sorting_busy = !isSortingIdle();

        // Intake logic (non-blocking)
        handleIntakeLatch();

        // Execute pending rotation if any
        processRotationRequest();

        // Execute shooting state machine if requested
        processShootState();
    }

    // ----------------------------
    // Tag order / queue
    // ----------------------------

    public void determine_artifact_order_from_tags() {
        List<AprilTagDetection> detections = robot.april_tag.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == 21) { artifact_order = 1; return; }
                if (tag.id == 22) { artifact_order = 2; return; }
                if (tag.id == 23) { artifact_order = 3; return; }
            }
        }
    }
    public void set_artifact_queue_from_order() {
        if (artifact_order == 1) init_artifact_queue(1, 2, 2);
        else if (artifact_order == 3) init_artifact_queue(2, 2, 1);
        else init_artifact_queue(2, 1, 2);
    }

    private void init_artifact_queue(int a1, int a2, int a3) {
        artifact_queue[0] = a1;
        artifact_queue[1] = a2;
        artifact_queue[2] = a3;
    }

    /** Sau khi bắn xong 1 viên, đẩy queue lên */
    public void advance_queue_after_shot() {
        artifact_queue[0] = artifact_queue[1];
        artifact_queue[1] = artifact_queue[2];
        artifact_queue[2] = 0;
    }

    // ----------------------------
    // Slots / sensors
    // ----------------------------

    public void update_artifact_slot() {
        for (int i = 0; i <= 2; i++) {
            artifact_slots[i] = sensorManager.get_artifact_color(i);
        }
    }

    /** Bản sao 3 slot artifact (0=empty, 1=green, 2=purple). */
    public int[] getArtifactSlots() {
        return new int[]{artifact_slots[0], artifact_slots[1], artifact_slots[2]};
    }

    /** Bản sao hàng đợi artifact (thứ tự chuẩn bị bắn). */
    public int[] getArtifactQueue() {
        return new int[]{artifact_queue[0], artifact_queue[1], artifact_queue[2]};
    }

    private int find_artifact(int color) {
        if (color == 0) return -1;
        for (int i = 0; i <= 2; i++) {
            if (artifact_slots[i] == color) return i;
        }
        return -1;
    }

    // ----------------------------
    // Collector
    // ----------------------------

    public void run_collector() {
        robot.motor_collector.setPower(1);
    }

    public void stop_collector() {
        robot.motor_collector.setPower(0);
    }

    // ----------------------------
    // Intake handling
    // ----------------------------

    private void handleIntakeLatch() {
        boolean ir = sensorManager.get_ir_state();

        // Start latch when IR sees something AND slot0 is currently empty
        if (ir && !intakeLatched && artifact_slots[0] == 0) {
            intakeLatched = true;
            intakeTimer.reset();
        }

        if (!ir) {
            // reset latch when beam clears
            intakeLatched = false;
            return;
        }

        if (intakeLatched) {
            // If a ball is now detected at slot0 -> index/rotate to free intake
            if (artifact_slots[0] != 0) {
                // Move to next slot (120° = 2*60°) only when motor idle and not in shooting
                if (shootState == ShootState.IDLE) {
                    requestRotate(true, 2, SORTING_POWER);
                }
                intakeLatched = false;
                return;
            }

            // Fail-safe: if latch too long, release
            if (intakeTimer.milliseconds() > INTAKE_LATCH_TIMEOUT_MS) {
                intakeLatched = false;
            }
        }
    }

    // ----------------------------
    // Rotation
    // ----------------------------

    /**
     * Request rotate in steps (each step = 60 degrees).
     * Will execute only when motor is idle.
     */
    private void requestRotate(boolean clockwise, int steps, double power) {
        if (steps <= 0) return;
        if (pendingSteps == 0) {
            pendingClockwise = clockwise;
            pendingSteps = steps;
        } else {
            // If same direction, accumulate, else override (simple + predictable)
            if (pendingClockwise == clockwise) pendingSteps += steps;
            else {
                pendingClockwise = clockwise;
                pendingSteps = steps;
            }
        }
    }

    private void processRotationRequest() {
        if (pendingSteps <= 0) return;

        if (isSortingIdle()) {
            rotate_steps(pendingClockwise, pendingSteps, SORTING_POWER);
            pendingSteps = 0;
        }
    }

    private void rotate_steps(boolean clockwise, int steps, double power) {
        int direction = clockwise ? 1 : -1;
        int currentPosition = robot.motor_sorting.getCurrentPosition();
        int targetPosition = currentPosition + (direction * steps * TICKS_FOR_60_DEGREES);

        robot.motor_sorting.setTargetPosition(targetPosition);
        robot.motor_sorting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_sorting.setPower(Math.abs(power));
    }

    private boolean isSortingIdle() {
        if (robot.motor_sorting.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return true;

        if (!robot.motor_sorting.isBusy()) return true;

        int err = Math.abs(robot.motor_sorting.getTargetPosition() - robot.motor_sorting.getCurrentPosition());
        return err <= POSITION_TOLERANCE_TICKS;
    }

    // ----------------------------
    // Shooting sequence (TeleOp)
    // ----------------------------

    /** Start pre-shoot sequence (non-blocking). */
    public void start_pre_shoot() {
        if (shootState != ShootState.IDLE) return;
        if (artifact_queue[0] == 0) return;

        shootState = ShootState.ALIGNING_TO_SLOT1;
        shootTimer.reset();
    }

    /** True when wheel is at "ready to kick" position */
    public boolean is_ready_to_kick() {
        return shootState == ShootState.READY_TO_KICK;
    }

    /** Request rotate back after kick (non-blocking). */
    public void request_post_shoot() {
        if (shootState != ShootState.READY_TO_KICK) return;
        shootState = ShootState.POST_ROTATE_BACK;
        shootTimer.reset();
    }

    private void processShootState() {
        if (shootState == ShootState.IDLE) return;

        // Fail-safe timeout
        if (shootTimer.milliseconds() > PRE_SHOOT_TIMEOUT_MS) {
            shootState = ShootState.IDLE;
            return;
        }

        // Wait motor finish before deciding next
        if (!isSortingIdle()) return;

        int targetColor = artifact_queue[0];
        if (targetColor == 0) {
            shootState = ShootState.IDLE;
            return;
        }

        switch (shootState) {
            case ALIGNING_TO_SLOT1: {
                // Goal: artifact_slots[1] == targetColor
                if (artifact_slots[1] == targetColor) {
                    shootState = ShootState.ROTATE_TO_SHOOT_POS;
                    // rotate 60° clockwise into shoot position (the same as your old code)
                    requestRotate(true, 1, SORTING_POWER);
                    return;
                }

                // If not in slot1, move it into slot1 using 120° (2*60°) like your old move_to_artifact_slot()
                int slot = find_artifact(targetColor);
                if (slot == -1) {
                    // not found: stop sequence
                    shootState = ShootState.IDLE;
                    return;
                }

                if (slot == 0) {
                    requestRotate(true, 2, SORTING_POWER);
                } else if (slot == 2) {
                    requestRotate(false, 2, SORTING_POWER);
                } else {
                    // slot == 1 handled above
                }
                return;
            }

            case ROTATE_TO_SHOOT_POS: {
                // After the 60° rotation completes, we are ready
                // If there is no pending rotation and motor is idle => ready
                if (pendingSteps == 0 && isSortingIdle()) {
                    shootState = ShootState.READY_TO_KICK;
                }
                return;
            }

            case READY_TO_KICK:
                // Wait for Main to call request_post_shoot() after kicker finishes
                return;

            case POST_ROTATE_BACK: {
                // Rotate back 60° counter-clockwise (same as your post_shoot())
                requestRotate(false, 1, SORTING_POWER);
                shootState = ShootState.POST_WAIT;
                return;
            }

            case POST_WAIT: {
                // Wait until the rotate-back finishes, then advance queue and go idle
                if (pendingSteps == 0 && isSortingIdle()) {
                    advance_queue_after_shot();
                    shootState = ShootState.IDLE;
                }
                return;
            }

            default:
                shootState = ShootState.IDLE;
        }
    }
}
