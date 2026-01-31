package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Main TeleOp – điều khiển robot FTC trong chế độ lái tay.
 * Tích hợp: di chuyển, turret, intake/artifact, shooter (flywheel + kick), vision (AprilTag).
 *
 * Mapping nút (gamepad1 = lái, gamepad2 = phụ):
 * - Drive: gamepad1 left/right stick
 * - Intake: Circle = chạy collector, Square = dừng
 * - Flywheel: LB = bật, RB = tắt
 * - Bắn: Triangle = bắt đầu chuỗi bắn
 * - Sort: Dpad Left (giữ) = 120° + nghỉ 0.5s lặp; Dpad Right (nhấn) = 60° một lần
 */
@TeleOp(name = "TELEOP")
public class Main extends LinearOpMode implements KickStateSetter {

    // ================== SHOOTER STATE ==================
    /** Cờ yêu cầu kick (đẩy pixel vào flywheel). */
    public boolean kick;

    @Override
    public void setKick(boolean value) { kick = value; }
    /** Bật/tắt flywheel (LB = bật, RB = tắt). */
    public boolean shooter_active = false;

    /** Công suất flywheel cố định 90% – không chỉnh bằng dpad. */
    public static final double SHOOTER_POWER_FIXED = 0.90;

    /** Giá trị mặc định team_color (1 hoặc 2 theo đội) – dùng cho vision/tag. */
    private static final int TEAM_COLOR_DEFAULT = 1;

    /** Tốc độ tối đa drive mặc định (55%). */
    private static final double DEFAULT_MAX_SPEED = 0.55;
    /** Turbo: gamepad1 RB. Slow: gamepad1 LB. */
    private static final double TURBO_SPEED = 0.80;
    private static final double SLOW_SPEED = 0.20;
    /** Deadzone joystick (0.1 = 10%). */
    private static final double DEFAULT_DEADZONE = 0.1;

    // ================== SYSTEM COMPONENTS ==================
    /** Màu đội (dùng cho vision/tag). Red=1, Blue=2; subclass set trước super.runOpMode(). */
    public int team_color = TEAM_COLOR_DEFAULT;
    /** Truy cập phần cứng (motor, servo, ...). */
    RobotHardware robot;
    /** Đọc cảm biến (khoảng cách, limit, ...). */
    SensorManager sensorManager;
    /** Di chuyển (drive) theo gamepad1. */
    Movement movement;
    /** Đọc và lưu trạng thái gamepad1/gamepad2. */
    GamepadController gamepadController;
    /** Xử lý thu thập/đẩy artifact (intake, queue, kick prep). */
    ArtifactProcessing artifactProcessing;
    /** Điều khiển turret (xoay theo tag hoặc tay). */
    Turret turret;
    /** Nhận diện AprilTag và khoảng cách (phục vụ turret). */
    TagProcessing tagProcessing;
    /** Ước lượng vị trí robot (x, y) từ encoder. */
    Odometry odometry;
    /** Flywheel + kick. */
    Shooter shooter;

    /** Chuỗi bắn: Triangle -> pre_shoot -> kick -> post_shoot (inline, không dùng ShootCoordinator). */
    private boolean lastTriangle = false;
    private boolean inshoot = false;
    private boolean kickStarted = false;
    /** Edge-trigger cho Dpad Right (60° một lần). */
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- INIT: khởi tạo phần cứng và các module ----------
        robot = new RobotHardware(hardwareMap);
        robot.init();

        sensorManager = new SensorManager(robot);
        gamepadController = new GamepadController(gamepad1, gamepad2);
        movement = new Movement(robot, gamepadController);
        artifactProcessing = new ArtifactProcessing(robot, sensorManager);
        tagProcessing = new TagProcessing(robot);
        turret = new Turret(robot, sensorManager, gamepadController, this);
        odometry = new Odometry(robot, 0, 0);
        shooter = new Shooter(robot, (KickStateSetter) this);

        waitForStart();

        movement.setMaxSpeed(DEFAULT_MAX_SPEED);
        movement.setDeadzone(DEFAULT_DEADZONE);

        // ---------- MAIN LOOP: chạy mỗi vòng lặp khi OpMode đang chạy ----------
        while (opModeIsActive()) {

            // ---------- INPUT: đọc và lưu trạng thái gamepad ----------
            gamepadController.update_gamepad();

            // ---------- DRIVE SPEED: gamepad1 RB = turbo, LB = slow, else default ----------
            if (gamepad1.right_bumper) movement.setMaxSpeed(TURBO_SPEED);
            else if (gamepad1.left_bumper) movement.setMaxSpeed(SLOW_SPEED);
            else movement.setMaxSpeed(DEFAULT_MAX_SPEED);

            // ---------- VISION: AprilTag phục vụ turret / range ----------
            tagProcessing.update(team_color);

            // ---------- SORT Dpad: Left (giữ) = 120° + nghỉ 0.5s; Right (nhấn) = 60° ----------
            artifactProcessing.setDpadLeftHeld(gamepad2.dpad_left);
            if (gamepad2.dpad_right && !lastDpadRight) {
                artifactProcessing.requestDpadRight60();
            }
            lastDpadRight = gamepad2.dpad_right;

            // ---------- ARTIFACT: slotHasBall (timing), chuẩn bị kick ----------
            artifactProcessing.update();

            // ---------- DRIVE: di chuyển robot theo gamepad1 ----------
            movement.move_robot();

            // ---------- ODOMETRY: cập nhật vị trí (x, y) từ encoder ----------
            odometry.setDriveCommand(
                    movement.getCmdForward(),
                    movement.getCmdStrafe(),
                    movement.getCmdTurn()
            );
            odometry.update();

            // ---------- TURRET: xoay turret theo tag hoặc điều khiển tay ----------
            turret.update();

            // ---------- INTAKE: Circle = chạy collector; không chạy khi 3 slot đầy (timing-only) ----------
            boolean slotsFull = artifactProcessing.getBallCount() >= 3;
            if (gamepad2.circle && !slotsFull) {
                artifactProcessing.setCollectorRunning(true);
                artifactProcessing.run_collector();
            } else {
                artifactProcessing.setCollectorRunning(false);
                artifactProcessing.stop_collector();
            }

            // ---------- FLYWHEEL: LB = bật, RB = tắt; không bấm thì auto bật khi có bóng để bắn ----------
            if (gamepad2.left_bumper) {
                shooter_active = true;
            } else if (gamepad2.right_bumper) {
                shooter_active = false;
            } else if (artifactProcessing.hasBallToShoot()) {
                shooter_active = true;
            }

            if (shooter_active) {
                shooter.run_flywheel_motor();
            } else {
                shooter.stop_flywheel_motor();
            }

            // ---------- SHOOT SEQUENCE: Triangle -> pre_shoot -> kick -> post_shoot ----------
            boolean triangle = gamepad2.triangle;
            if (triangle && !lastTriangle) {
                inshoot = true;
                kickStarted = false;
                artifactProcessing.start_pre_shoot();
            }
            lastTriangle = triangle;
            if (inshoot) {
                if (artifactProcessing.is_ready_to_kick()) shooter.request_kick();
                if (shooter.isKicking()) kickStarted = true;
                if (kickStarted && !shooter.isKicking()) {
                    artifactProcessing.request_post_shoot();
                    inshoot = false;
                    kickStarted = false;
                }
            }
            shooter.update();

            // ---------- TELEMETRY: hiển thị trạng thái lên Driver Station ----------
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("TagDetected", tagProcessing.isDetected());
        telemetry.addData("TagRange(in)", "%.1f", tagProcessing.getRangeInches());
        telemetry.addData("Flywheel", shooter_active ? "ON" : "OFF");
        telemetry.addData("X (cm)", "%.2f", odometry.getX());
        telemetry.addData("Y (cm)", "%.2f", odometry.getY());
        int[] slots = artifactProcessing.getArtifactSlots();
        telemetry.addData("Slots [0,1,2]", "%d %d %d", slots[0], slots[1], slots[2]);
        telemetry.addData("BallCount", artifactProcessing.getBallCount());
        boolean readyToKick = artifactProcessing.is_ready_to_kick();
        String shootStateLabel = readyToKick ? "READY" : (inshoot ? "PREP" : "IDLE");
        telemetry.addData("ShootState", shootStateLabel);
        telemetry.update();
    }
}
