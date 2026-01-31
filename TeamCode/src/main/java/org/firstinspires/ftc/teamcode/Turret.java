package org.firstinspires.ftc.teamcode;

/**
 * Turret - Continuous rotation servo (360°) turret control with endstops.
 *
 * Control signal convention:
 *  - cmd in [-1..1]
 *  - cmd < 0 : rotate LEFT
 *  - cmd > 0 : rotate RIGHT
 *
 * Continuous servo mapping:
 *  - position = 0.5 => stop
 *  - position < 0.5 => one direction
 *  - position > 0.5 => opposite direction
 */
public class Turret {
    private final RobotHardware robot;
    private final SensorManager sensorManager;
    private final GamepadController gamepadController;
    private final Main main;
    /** Cho Auto: dùng trực tiếp TagProcessing khi main == null. */
    private final TagProcessing tagProcessingRef;

    // ===== TUNING (theo em Vinh - đã chỉnh đúng) =====
    private static final double AUTO_GAIN = 0.7;
    private static final double MANUAL_DEADZONE = 0.10;
    private static final double CMD_DEADBAND = 0.03;
    private static final double MAX_CMD = 0.85;
    private static final double CMD_RAMP_STEP = 0.08;
    private static final double SERVO_NEUTRAL = 0.5;
    private static final double DIR = 1.0;
    private static final double TAG_HALF_WIDTH_PX = 240.0; // 480px width => half = 240

    /** Nội suy tuyến tính: alpha càng lớn càng bám tag nhanh (0..1) */
    private static final double AIM_LERP_ALPHA = 0.25;
    private double lastAutoCmd = 0.0;
    private double lastCmd = 0.0;

    public Turret(RobotHardware robot,
                  SensorManager sensorManager,
                  GamepadController gamepadController,
                  Main main) {
        this.robot = robot;
        this.sensorManager = sensorManager;
        this.gamepadController = gamepadController;
        this.main = main;
        this.tagProcessingRef = null;
    }

    /** Constructor cho Auto: chỉ bám AprilTag, không dùng gamepad. */
    public Turret(RobotHardware robot, SensorManager sensorManager, TagProcessing tagProcessing) {
        this.robot = robot;
        this.sensorManager = sensorManager;
        this.gamepadController = null;
        this.main = null;
        this.tagProcessingRef = tagProcessing;
    }

    public void update() {
        handleTurretRotation();
    }

    private void handleTurretRotation() {
        // 1) AUTO COMMAND (Apriltag) + nội suy tuyến tính để mượt
        double rawAutoCmd = 0.0;
        TagProcessing tag = (main != null && main.tagProcessing != null) ? main.tagProcessing : tagProcessingRef;
        if (tag != null && tag.isDetected()) {
            double px = tag.getDistanceToCenter();
            double norm = clamp(px / TAG_HALF_WIDTH_PX, -1.0, 1.0);
            rawAutoCmd = norm * AUTO_GAIN;
        }
        double autoCmd = lerp(lastAutoCmd, rawAutoCmd, AIM_LERP_ALPHA);
        lastAutoCmd = autoCmd;

        // 2) MANUAL OVERRIDE (TeleOp only)
        double manual = (gamepadController != null) ? gamepadController.shooter_turret_rotate : 0.0;
        double cmd = autoCmd;
        if (gamepadController != null && Math.abs(manual) > MANUAL_DEADZONE) {
            cmd = manual;
        }

        // 3) APPLY DIRECTION, CLAMPS, DEADBAND
        cmd *= DIR;
        cmd = clamp(cmd, -MAX_CMD, MAX_CMD);

        if (Math.abs(cmd) < CMD_DEADBAND) cmd = 0.0;

        // 4) ENDSTOP LOGIC
        // If at right limit, block further RIGHT motion (cmd > 0)
        if (sensorManager.isTurretAtRightLimit() && cmd > 0.0) {
            cmd = 0.0;
        }
        // If at left limit, block further LEFT motion (cmd < 0)
        if (sensorManager.isTurretAtLeftLimit() && cmd < 0.0) {
            cmd = 0.0;
        }

        // 5) RAMP (SMOOTHING)
        cmd = ramp(lastCmd, cmd, CMD_RAMP_STEP);
        lastCmd = cmd;

        // 6) MAP [-1..1] -> [0..1] for continuous servo
        setContinuousServo(cmd);
    }

    private void setContinuousServo(double cmd) {
        // cmd = -1..1
        // pos = 0.5 + 0.5*cmd
        double pos = SERVO_NEUTRAL + (cmd * 0.5);
        pos = clamp(pos, 0.0, 1.0);
        robot.turret_servo.setPosition(pos);
    }

    private static double ramp(double current, double target, double step) {
        double diff = target - current;
        if (diff > step) return current + step;
        if (diff < -step) return current - step;
        return target;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
