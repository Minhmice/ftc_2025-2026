package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter
 *  - Flywheel speed-hold using encoder velocity control (preferred).
 *  - Automatic angle servo setpoint based on AprilTag range.
 *  - Non-blocking kicker timing.
 */
public class Shooter {
    private final RobotHardware robot;
    private final Main main;
    private final DcMotorEx flywheel;

    // ---------------- KICKER ----------------
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private static final double KICK_POS = 90.0 / 180.0;
    private static final double REST_POS = 0.0;
    private static final long KICK_HOLD_MS = 500;
    private boolean kicking = false;

    // ---------------- FLYWHEEL SPEED HOLD ----------------
    // Best practice: hold RPM via encoder (setVelocity) instead of setPower,
    // because setPower(0.9) will sag when battery voltage drops.
    private boolean useVelocityHold = true;

    // TODO: set these 2 constants to match your shooter motor.
    // Common examples:
    //  - goBILDA motor encoder: 28 ticks/rev at the motor
    //  - some output encoders can be 537.7 ticks/rev (depends gear ratio)
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_FREE_RPM = 6000.0;

    private static final double TARGET_SPEED_FRACTION = 0.90; // user request
    private double targetRpm = SHOOTER_FREE_RPM * TARGET_SPEED_FRACTION;

    // Voltage-comp fallback (only used if velocity hold is disabled)
    private static final double NOMINAL_VOLTAGE = 12.0;

    // ---------------- AUTO ANGLE (RANGE -> SERVO POS) ----------------
    private boolean autoAngleEnabled = true;
    private double angleTrim = 0.0;          // operator fine-tune
    private double anglePos = 0.50;          // filtered output

    // Smoothing to prevent jitter
    private static final double ANGLE_ALPHA = 0.25;

    // Range from ftcPose.range is in inches (FTC SDK).
    // LUT + interpolation is more reliable than projectile physics in FTC.
    // TODO: tune these points by testing. Format: {rangeIn, servoPos}
    private static final double[][] RANGE_TO_ANGLE_LUT = new double[][]{
            {12, 0.22},
            {18, 0.27},
            {24, 0.32},
            {30, 0.38},
            {36, 0.44},
            {48, 0.52},
            {60, 0.60},
    };

    // If camera is not aligned with shooter pivot, offset range a bit (inches).
    private static final double RANGE_OFFSET_IN = 0.0;

    public Shooter(RobotHardware robot, Main main) {
        this.robot = robot;
        this.main = main;

        robot.kicking_servo.setPosition(REST_POS);
        main.kick = false;

        this.flywheel = robot.motor_shooter; // DcMotorEx in RobotHardware
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.angle_servo.setPosition(anglePos);
    }

    // ---------------- PUBLIC CONTROLS ----------------
    public void setAutoAngleEnabled(boolean enabled) { this.autoAngleEnabled = enabled; }
    public boolean isAutoAngleEnabled() { return autoAngleEnabled; }

    /** Fine-tune the LUT output (small trims during match). */
    public void nudgeAngleTrim(double delta) {
        angleTrim = clamp(angleTrim + delta, -0.15, 0.15);
    }
    public double getAngleTrim() { return angleTrim; }

    /** Prefer velocity hold; can disable for quick debugging. */
    public void setUseVelocityHold(boolean enabled) { this.useVelocityHold = enabled; }

    public void setTargetRpm(double rpm) { this.targetRpm = Math.max(0.0, rpm); }
    public double getTargetRpm() { return targetRpm; }

    /** Current measured RPM from encoder velocity (approx). */
    public double getCurrentRpm() {
        double ticksPerSec = flywheel.getVelocity();
        return (ticksPerSec / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    /** Call every loop if you want auto angle to track tag range. */
    public void updateAutoAngle(TagProcessing tagProcessing) {
        if (!autoAngleEnabled) return;

        if (tagProcessing != null && tagProcessing.isDetected()) {
            double rangeIn = tagProcessing.getRangeInches();
            if (rangeIn > 0.0) {
                double target = angleFromRangeIn(rangeIn + RANGE_OFFSET_IN);
                target = clamp(target + angleTrim, 0.0, 1.0);
                anglePos = anglePos + ANGLE_ALPHA * (target - anglePos);
                robot.angle_servo.setPosition(anglePos);
            }
        }
    }

    /** Spin flywheel at constant 90% speed (encoder hold preferred). */
    public void run_flywheel_motor() {
        if (useVelocityHold) {
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setVelocity(rpmToTicksPerSec(targetRpm));
        } else {
            // fallback: voltage-compensated power
            double v = robot.getBatteryVoltage();
            double power = TARGET_SPEED_FRACTION * (NOMINAL_VOLTAGE / Math.max(1.0, v));
            flywheel.setPower(clamp(power, 0.0, 1.0));
        }
    }

    public void stop_flywheel_motor() {
        flywheel.setPower(0.0);
    }

    // ---------------- KICKER ----------------
    /** Request a kick (edge-triggered, non-blocking). */
    public void request_kick() {
        if (kicking) return;

        kicking = true;
        timer.reset();
        robot.kicking_servo.setPosition(KICK_POS);
        main.kick = true;
    }

    /** Call every loop to complete kick timing. */
    public void update() {
        if (!kicking) return;

        if (timer.milliseconds() >= KICK_HOLD_MS) {
            robot.kicking_servo.setPosition(REST_POS);
            main.kick = false;
            kicking = false;
        }
    }

    public boolean isKicking() { return kicking; }

    // ---------------- INTERNAL HELPERS ----------------
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

    private static double angleFromRangeIn(double rangeIn) {
        double minR = RANGE_TO_ANGLE_LUT[0][0];
        double maxR = RANGE_TO_ANGLE_LUT[RANGE_TO_ANGLE_LUT.length - 1][0];
        double r = clamp(rangeIn, minR, maxR);

        for (int i = 0; i < RANGE_TO_ANGLE_LUT.length - 1; i++) {
            double r0 = RANGE_TO_ANGLE_LUT[i][0];
            double a0 = RANGE_TO_ANGLE_LUT[i][1];
            double r1 = RANGE_TO_ANGLE_LUT[i + 1][0];
            double a1 = RANGE_TO_ANGLE_LUT[i + 1][1];

            if (r >= r0 && r <= r1) {
                double t = (r - r0) / (r1 - r0);
                return a0 + t * (a1 - a0);
            }
        }
        return RANGE_TO_ANGLE_LUT[RANGE_TO_ANGLE_LUT.length - 1][1];
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
