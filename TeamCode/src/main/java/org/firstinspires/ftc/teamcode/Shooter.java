package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter – flywheel speed-hold (encoder velocity), non-blocking kicker timing.
 */
public class Shooter {
    private final RobotHardware robot;
    private final KickStateSetter kickStateSetter;
    private final DcMotorEx flywheel;

    // ---------------- KICKER ----------------
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private static final double KICK_POS = 90.0 / 180.0;
    private static final double REST_POS = 0.0;
    private static final long KICK_HOLD_MS = 350;
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

    public Shooter(RobotHardware robot, KickStateSetter kickStateSetter) {
        this.robot = robot;
        this.kickStateSetter = kickStateSetter;

        robot.kicking_servo.setPosition(REST_POS);
        if (kickStateSetter != null) kickStateSetter.setKick(false);

        this.flywheel = robot.motor_shooter; // DcMotorEx in RobotHardware
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ---------------- PUBLIC CONTROLS ----------------
    /** Prefer velocity hold; can disable for quick debugging. */
    public void setUseVelocityHold(boolean enabled) { this.useVelocityHold = enabled; }

    public void setTargetRpm(double rpm) { this.targetRpm = Math.max(0.0, rpm); }
    public double getTargetRpm() { return targetRpm; }

    /** Current measured RPM from encoder velocity (approx). */
    public double getCurrentRpm() {
        double ticksPerSec = flywheel.getVelocity();
        return (ticksPerSec / SHOOTER_TICKS_PER_REV) * 60.0;
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
        if (kickStateSetter != null) kickStateSetter.setKick(true);
    }

    /** Call every loop to complete kick timing. */
    public void update() {
        if (!kicking) return;

        if (timer.milliseconds() >= KICK_HOLD_MS) {
            robot.kicking_servo.setPosition(REST_POS);
            if (kickStateSetter != null) kickStateSetter.setKick(false);
            kicking = false;
        }
    }

    public boolean isKicking() { return kicking; }

    // ---------------- INTERNAL HELPERS ----------------
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
