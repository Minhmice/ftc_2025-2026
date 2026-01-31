package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ArtifactProcessing - Sort bóng chỉ theo timing (FIFO), không cảm biến / AprilTag.
 *
 * - slotHasBall[3]: ô 0=intake, 1=trái, 2=phải; cập nhật theo intake (timer), permute khi quay, xóa khi kick.
 * - Intake: collector chạy >= INTAKE_COMMIT_MS -> slotHasBall[0]=true, request 2 CW giải phóng cửa.
 * - Bắn: rotate để ô có bóng tới kicker -> half align -> kick -> post half, clear slot.
 */
public class ArtifactProcessing {

    private final RobotHardware robot;
    private final SensorManager sensorManager;
    private final SortIndexer sortIndexer;

    /** Ô i có bóng hay không (0=intake, 1=trái, 2=phải). Chỉ cập nhật theo hành động. */
    private final boolean[] slotHasBall = new boolean[]{false, false, false};

    /** Thời gian collector chạy liên tục để coi 1 bóng đã vào (ms). */
    public static final long INTAKE_COMMIT_MS = 1000;

    private static final long PRE_SHOOT_TIMEOUT_MS = 2000;

    public boolean sorting_busy = false;

    /** Collector đang chạy (Main gọi setCollectorRunning khi Circle hold/release). */
    private boolean collectorRunning = false;
    private final ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private int pendingSteps = 0;
    private boolean pendingClockwise = true;

    /** Sau khi gửi rotation tới SortIndexer, ghi nhận để permute slotHasBall khi xong. */
    private int pendingPermuteSteps = 0;
    private boolean pendingPermuteCW = true;
    /** Rotation vừa gửi là của chuỗi bắn (ROTATE_TO_KICKER); chỉ khi đó mới chuyển HALF_ALIGN + half khi xong. */
    private boolean pendingPermuteWasForShoot = false;
    /** Được set true chỉ khi processShootState (ROTATE_TO_KICKER) gọi requestRotate; processRotationRequest đọc rồi gán pendingPermuteWasForShoot. */
    private boolean rotationRequestedFromShoot = false;

    private enum ShootState { IDLE, ROTATE_TO_KICKER, HALF_ALIGN, READY_TO_KICK, POST_ROTATE_BACK, POST_WAIT }
    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /** Đã request half trong POST_ROTATE_BACK để tránh lặp. */
    private boolean postHalfRequested = false;

    /** Dpad Left held (Main set mỗi frame). */
    private boolean dpadLeftHeld = false;
    /** Dpad Right pressed edge (Main gọi requestDpadRight60 một lần). */
    private boolean dpadRightRequested = false;

    public ArtifactProcessing(RobotHardware robot, SensorManager sensorManager) {
        this.robot = robot;
        this.sensorManager = sensorManager;
        this.sortIndexer = new SortIndexer(robot.servo_sort);
    }

    /** Main gọi khi Circle hold (true) / release (false). */
    public void setCollectorRunning(boolean running) {
        if (running && !collectorRunning) intakeTimer.reset();
        collectorRunning = running;
    }

    /** Main gọi mỗi frame khi đọc gamepad (Dpad Left held). */
    public void setDpadLeftHeld(boolean held) {
        dpadLeftHeld = held;
    }

    /** Main gọi một lần khi nhấn Dpad Right (edge). */
    public void requestDpadRight60() {
        dpadRightRequested = true;
    }

    public void update() {
        sortIndexer.setDpadLeftHeld(dpadLeftHeld);
        if (dpadRightRequested) {
            sortIndexer.requestDpadRight60();
            dpadRightRequested = false;
        }
        sortIndexer.update();

        applyPermuteWhenRotationDone();
        handleIntakeTiming();
        processRotationRequest();
        processShootState();

        sorting_busy = !isSortingIdle();
    }

    private void applyPermuteWhenRotationDone() {
        if (!sortIndexer.isIdle() || pendingPermuteSteps <= 0) return;
        permuteSlotHasBall(pendingPermuteSteps, pendingPermuteCW);
        boolean wasForShoot = pendingPermuteWasForShoot;
        pendingPermuteSteps = 0;
        pendingPermuteWasForShoot = false;
        if (wasForShoot && shootState == ShootState.ROTATE_TO_KICKER) {
            shootState = ShootState.HALF_ALIGN;
            requestRotateHalf(true);
        }
    }

    /** 1 CW: new[i] = old[(i-1+3)%3]. N CW: new[i] = old[(i-N+3)%3] (mod 3). */
    private void permuteSlotHasBall(int steps, boolean cw) {
        int shift = cw ? ((3 - steps % 3) % 3) : (steps % 3);
        if (shift == 0) return;
        boolean[] old = new boolean[]{slotHasBall[0], slotHasBall[1], slotHasBall[2]};
        for (int i = 0; i < 3; i++)
            slotHasBall[i] = old[(i + shift) % 3];
    }

    private void handleIntakeTiming() {
        if (!collectorRunning) return;
        if (intakeTimer.milliseconds() < INTAKE_COMMIT_MS) return;
        slotHasBall[0] = true;
        intakeTimer.reset();
        if (isShootIdle()) requestRotate(true, 2);
    }

    public boolean isShootIdle() {
        return shootState == ShootState.IDLE;
    }

    public int getBallCount() {
        int n = 0;
        for (int i = 0; i < 3; i++) if (slotHasBall[i]) n++;
        return n;
    }

    public boolean hasBallToShoot() {
        return getBallCount() > 0;
    }

    /** Slots 0/1 (có bóng hay không). */
    public int[] getArtifactSlots() {
        return new int[]{
                slotHasBall[0] ? 1 : 0,
                slotHasBall[1] ? 1 : 0,
                slotHasBall[2] ? 1 : 0
        };
    }

    /** FIFO: queue[0]=1 nếu còn bóng để bắn. */
    public int[] getArtifactQueue() {
        int b = hasBallToShoot() ? 1 : 0;
        return new int[]{b, 0, 0};
    }

    public void run_collector() {
        robot.motor_collector.setPower(1);
    }

    public void stop_collector() {
        robot.motor_collector.setPower(0);
    }

    public void requestManualRotate(boolean clockwise, int steps) {
        requestRotate(clockwise, steps);
    }

    private void requestRotate(boolean clockwise, int steps) {
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

    private void processRotationRequest() {
        if (pendingSteps <= 0) return;
        if (!isSortingIdle()) return;
        pendingPermuteSteps = pendingSteps;
        pendingPermuteCW = pendingClockwise;
        pendingPermuteWasForShoot = rotationRequestedFromShoot;
        rotationRequestedFromShoot = false;
        sortIndexer.requestRotate(pendingClockwise, pendingSteps);
        pendingSteps = 0;
    }

    private boolean isSortingIdle() {
        return sortIndexer.isIdle();
    }

    private void requestRotateHalf(boolean clockwise) {
        sortIndexer.requestRotateHalf(clockwise);
    }

    public void start_pre_shoot() {
        if (shootState != ShootState.IDLE) return;
        if (!hasBallToShoot()) return;
        shootState = ShootState.ROTATE_TO_KICKER;
        shootTimer.reset();
    }

    public boolean is_ready_to_kick() {
        return shootState == ShootState.READY_TO_KICK;
    }

    public void request_post_shoot() {
        if (shootState != ShootState.READY_TO_KICK) return;
        shootState = ShootState.POST_ROTATE_BACK;
        postHalfRequested = false;
        shootTimer.reset();
    }

    private void processShootState() {
        if (shootState == ShootState.IDLE) return;
        if (shootTimer.milliseconds() > PRE_SHOOT_TIMEOUT_MS) {
            shootState = ShootState.IDLE;
            return;
        }
        if (!isSortingIdle()) return;

        int k = sortIndexer.getCurrentSlot();
        if (!hasBallToShoot()) {
            shootState = ShootState.IDLE;
            return;
        }

        switch (shootState) {
            case ROTATE_TO_KICKER:
                if (slotHasBall[k]) {
                    shootState = ShootState.HALF_ALIGN;
                    requestRotateHalf(true);
                } else {
                    int stepsCW = -1, stepsCCW = -1;
                    for (int s = 1; s <= 2; s++) {
                        if (slotHasBall[(k + s) % 3]) { stepsCW = s; break; }
                    }
                    for (int s = 1; s <= 2; s++) {
                        if (slotHasBall[(k - s + 3) % 3]) { stepsCCW = s; break; }
                    }
                    if (stepsCW < 0 && stepsCCW < 0) {
                        shootState = ShootState.IDLE;
                        return;
                    }
                    if (stepsCW >= 0 && (stepsCCW < 0 || stepsCW <= stepsCCW)) {
                        rotationRequestedFromShoot = true;
                        requestRotate(true, stepsCW);
                    } else {
                        rotationRequestedFromShoot = true;
                        requestRotate(false, stepsCCW);
                    }
                }
                return;

            case HALF_ALIGN:
                if (isSortingIdle()) shootState = ShootState.READY_TO_KICK;
                return;

            case READY_TO_KICK:
                return;

            case POST_ROTATE_BACK:
                if (!postHalfRequested) {
                    requestRotateHalf(false);
                    postHalfRequested = true;
                    shootState = ShootState.POST_WAIT;
                }
                return;

            case POST_WAIT:
                if (isSortingIdle()) {
                    slotHasBall[sortIndexer.getCurrentSlot()] = false;
                    shootState = ShootState.IDLE;
                }
                return;

            default:
                shootState = ShootState.IDLE;
        }
    }
}
