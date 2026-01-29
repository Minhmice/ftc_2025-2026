package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Main TeleOp – điều khiển robot FTC trong chế độ lái tay.
 * Tích hợp: di chuyển, turret, intake/artifact, shooter (flywheel + kick), vision (AprilTag).
 *
 * Mapping nút (gamepad1 = lái, gamepad2 = phụ):
 * - Drive: gamepad1 left/right stick
 * - Intake: Circle = chạy collector, Square = dừng
 * - Flywheel: LB = bật, RB = tắt
 * - Bắn: Triangle = bắt đầu chuỗi bắn
 * - Góc bắn: Y = +trim, A = -trim, Back = bật/tắt auto-angle theo AprilTag
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

    /** Góc servo bắn thực tế – dùng cho telemetry/debug. */
    public double shooting_angle = 0;

    /** Công suất flywheel cố định 90% – không chỉnh bằng dpad. */
    public static final double SHOOTER_POWER_FIXED = 0.90;

    /** Giá trị mặc định team_color (1 hoặc 2 theo đội) – dùng cho vision/tag. */
    private static final int TEAM_COLOR_DEFAULT = 1;

    /** Tốc độ tối đa drive (0.25 = 25%). */
    private static final double DEFAULT_MAX_SPEED = 0.25;
    /** Deadzone joystick (0.1 = 10%). */
    private static final double DEFAULT_DEADZONE = 0.1;

    /** Trạng thái nút trước đó – dùng edge-trigger (chỉ xử lý khi nhấn lần đầu). */
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastBack = false;

    // ================== SYSTEM COMPONENTS ==================
    /** Màu đội (dùng cho vision/tag). Red=1, Blue=2; subclass set trước super.runOpMode(). */
    public int team_color = TEAM_COLOR_DEFAULT;
    /** Truy cập phần cứng (motor, servo, IMU, ...). */
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
    /** Nhận diện AprilTag và khoảng cách (phục vụ góc bắn). */
    TagProcessing tagProcessing;
    /** Ước lượng vị trí robot (x, y) từ encoder/IMU. */
    Odometry odometry;
    /** Flywheel + servo góc + kick + auto-angle. */
    Shooter shooter;
    /** Điều phối chuỗi bắn (Triangle -> pre_shoot -> kick -> post_shoot). */
    ShootCoordinator shootCoordinator;

    /** UDP: gửi log + camera tới laptop (khi udp_config.txt hợp lệ). */
    private UdpConfig udpConfig;
    private UdpLogger udpLogger;
    private FrameCaptureProcessor frameCaptureProcessor;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- INIT: khởi tạo phần cứng và các module ----------
        robot = new RobotHardware(hardwareMap);
        udpConfig = new UdpConfig(UdpConfig.DEFAULT_PATH);
        if (udpConfig.isValid()) {
            frameCaptureProcessor = new FrameCaptureProcessor(udpConfig);
            robot.setFrameCaptureProcessor(frameCaptureProcessor);
        }
        robot.init();

        sensorManager = new SensorManager(robot);
        gamepadController = new GamepadController(gamepad1, gamepad2);
        movement = new Movement(robot, gamepadController);
        artifactProcessing = new ArtifactProcessing(robot, sensorManager);
        tagProcessing = new TagProcessing(robot);
        turret = new Turret(robot, sensorManager, gamepadController, this);
        odometry = new Odometry(robot, 0, 0);
        shooter = new Shooter(robot, (KickStateSetter) this);
        shootCoordinator = new ShootCoordinator(artifactProcessing, shooter);

        waitForStart();

        if (udpConfig != null && udpConfig.isValid()) {
            udpLogger = new UdpLogger(udpConfig);
            udpLogger.start();
        }

        movement.setMaxSpeed(DEFAULT_MAX_SPEED);
        movement.setDeadzone(DEFAULT_DEADZONE);

        // ---------- MAIN LOOP: chạy mỗi vòng lặp khi OpMode đang chạy ----------
        while (opModeIsActive()) {

            // ---------- INPUT: đọc và lưu trạng thái gamepad ----------
            gamepadController.update_gamepad();

            // ---------- VISION: cập nhật phát hiện AprilTag và khoảng cách ----------
            tagProcessing.update(team_color);

            // ---------- AUTO SHOOTING ANGLE (gamepad2) ----------
            // Y: tăng trim góc (+0.01), A: giảm trim (-0.01) – chỉ khi nhấn lần đầu (edge-trigger)
            boolean y = gamepad2.y;
            boolean a = gamepad2.a;

            if (y && !lastY) shooter.nudgeAngleTrim(+0.01);
            if (a && !lastA) shooter.nudgeAngleTrim(-0.01);

            lastY = y;
            lastA = a;

            // Back: bật/tắt chế độ auto-angle (góc bắn theo khoảng cách tag)
            boolean back = gamepad2.back;
            if (back && !lastBack) {
                shooter.setAutoAngleEnabled(!shooter.isAutoAngleEnabled());
            }
            lastBack = back;

            // Tính và set góc servo bắn dựa trên khoảng cách AprilTag (nếu auto-angle bật)
            shooter.updateAutoAngle(tagProcessing);

            // Lấy góc servo thực tế cho telemetry
            shooting_angle = robot.angle_servo.getPosition();

            // ---------- ARTIFACT: cập nhật queue, trạng thái slot, chuẩn bị kick ----------
            artifactProcessing.update();

            // ---------- DRIVE: di chuyển robot theo gamepad1 ----------
            movement.move_robot();

            // ---------- ODOMETRY: cập nhật vị trí (x, y) từ encoder + IMU ----------
            odometry.setDriveCommand(
                    movement.getCmdForward(),
                    movement.getCmdStrafe(),
                    movement.getCmdTurn()
            );
            odometry.update();

            // ---------- TURRET: xoay turret theo tag hoặc điều khiển tay ----------
            turret.update();

            // ---------- INTAKE: Circle = chạy collector, Square = dừng ----------
            if (gamepad2.circle) {
                artifactProcessing.run_collector();
            } else if (gamepad2.square) {
                artifactProcessing.stop_collector();
            }

            // ---------- FLYWHEEL: LB = bật, RB = tắt; khi bật chạy ở 90% (ổn định pin) ----------
            if (gamepad2.left_bumper) {
                shooter_active = true;
            } else if (gamepad2.right_bumper) {
                shooter_active = false;
            }

            if (shooter_active) {
                shooter.run_flywheel_motor();
            } else {
                shooter.stop_flywheel_motor();
            }

            // ---------- SHOOT SEQUENCE: Triangle bắt đầu chuỗi bắn ----------
            shootCoordinator.update(gamepad2.triangle);

            // ---------- TELEMETRY: hiển thị trạng thái lên Driver Station ----------
            updateTelemetry();

            // ---------- UDP: gửi log + frame tới laptop ----------
            if (udpLogger != null) {
                udpLogger.pushLog(buildTelemetryString());
                if (frameCaptureProcessor != null) {
                    byte[] jpeg = frameCaptureProcessor.getLastJpeg();
                    if (jpeg != null) udpLogger.pushJpeg(jpeg);
                }
            }
        }
        if (udpLogger != null) udpLogger.stop();
    }

    /** Chuỗi telemetry (cùng nội dung updateTelemetry) để gửi UDP. */
    private String buildTelemetryString() {
        int[] slots = artifactProcessing.getArtifactSlots();
        int[] queue = artifactProcessing.getArtifactQueue();
        String senderIp = (udpLogger != null) ? udpLogger.getSenderIp() : "N/A";
        return String.format(
                "AutoAngle: %s\nAnglePos: %.3f\nTagDetected: %s\nTagRange(in): %.1f\nFlywheel: %s\nYaw (deg): %.1f\nX (cm): %.2f\nY (cm): %.2f\nSlots [0,1,2]: %d %d %d\nQueue [0,1,2]: %d %d %d\nShootState: %s\nUDP sender IP: %s",
                shooter.isAutoAngleEnabled(), shooting_angle, tagProcessing.isDetected(),
                tagProcessing.getRangeInches(), shooter_active ? "ON" : "OFF",
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                odometry.getX(), odometry.getY(),
                slots[0], slots[1], slots[2], queue[0], queue[1], queue[2],
                shootCoordinator.getShootStateLabel(artifactProcessing.is_ready_to_kick()), senderIp);
    }

    private void updateTelemetry() {
        telemetry.addData("AutoAngle", shooter.isAutoAngleEnabled());
        telemetry.addData("AnglePos", "%.3f", shooting_angle);
        telemetry.addData("TagDetected", tagProcessing.isDetected());
        telemetry.addData("TagRange(in)", "%.1f", tagProcessing.getRangeInches());
        telemetry.addData("Flywheel", shooter_active ? "ON" : "OFF");
        telemetry.addData("Yaw (deg)",
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("X (cm)", "%.2f", odometry.getX());
        telemetry.addData("Y (cm)", "%.2f", odometry.getY());
        int[] slots = artifactProcessing.getArtifactSlots();
        telemetry.addData("Slots [0,1,2]", "%d %d %d", slots[0], slots[1], slots[2]);
        int[] queue = artifactProcessing.getArtifactQueue();
        telemetry.addData("Queue [0,1,2]", "%d %d %d", queue[0], queue[1], queue[2]);
        telemetry.addData("ShootState",
                shootCoordinator.getShootStateLabel(artifactProcessing.is_ready_to_kick()));
        if (udpLogger != null) {
            telemetry.addData("UDP sender IP", udpLogger.getSenderIp());
        }
        telemetry.update();
    }
}
