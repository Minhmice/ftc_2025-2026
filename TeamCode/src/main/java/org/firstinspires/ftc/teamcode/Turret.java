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

    // ===== TUNING =====
    // Auto aiming: proportional correction based on tag pixel offset (higher = turret turns faster)
    private static final double AUTO_GAIN = 1.0;

    // Manual stick deadzone
    private static final double MANUAL_DEADZONE = 0.10;

    // Extra deadband near 0 to avoid servo "buzzing"
    private static final double CMD_DEADBAND = 0.03;

    // Limit max speed so turret doesn't slam endstops too hard
    private static final double MAX_CMD = 0.85;

    // Rate limit: larger step = turret catches up to tag faster at ~30 FPS
    private static final double CMD_RAMP_STEP = 0.18;

    // Servo neutral stop position for continuous rotation
    private static final double SERVO_NEUTRAL = 0.5;

    // If turret direction is reversed, set to -1
    private static final double DIR = 1.0;

    // Tag center normalization (your camera width assumption)
    private static final double TAG_HALF_WIDTH_PX = 320.0; // 480px width => half = 240

    // State
    private double lastCmd = 0.0;

    public Turret(RobotHardware robot,
                  SensorManager sensorManager,
                  GamepadController gamepadController,
                  Main main) {
        this.robot = robot;
        this.sensorManager = sensorManager;
        this.gamepadController = gamepadController;
        this.main = main;
    }

    public void update() {
        handleTurretRotation();
    }

    private void handleTurretRotation() {
        // 1) AUTO COMMAND (Apriltag)
        double autoCmd = 0.0;
        if (main != null && main.tagProcessing != null && main.tagProcessing.isDetected()) {
            // distanceToCenter expected roughly [-TAG_HALF_WIDTH_PX .. +TAG_HALF_WIDTH_PX]
            double px = main.tagProcessing.getDistanceToCenter();
            double norm = clamp(px / TAG_HALF_WIDTH_PX, -1.0, 1.0);
            autoCmd = norm * AUTO_GAIN;
        }

        // 2) MANUAL OVERRIDE
        double manual = gamepadController.shooter_turret_rotate; // expected [-1..1]
        double cmd = autoCmd;

        if (Math.abs(manual) > MANUAL_DEADZONE) {
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
}
