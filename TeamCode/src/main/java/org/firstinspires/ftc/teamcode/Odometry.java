package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Odometry {

    private final RobotHardware robot;

    // --- Pod specs ---
    public static final double WHEEL_DIAMETER_CM = 3.2;   // 32mm
    public static final double ENCODER_PPR = 2000.0;      // 2000 ppr
    public static final double TICKS_PER_CM = ENCODER_PPR / (Math.PI * WHEEL_DIAMETER_CM); // ~198.94

    // --- SIGN CONFIG ---
    public static int X_ENCODER_SIGN = +1;  // odometry_x
    public static int Y_ENCODER_SIGN = -1;  // motor_4
    public static int IMU_YAW_SIGN   = +1;

    // --- OFFSETS from robot CENTER (cm) ---
    // PARALLEL: lateral offset of Y encoder (+left, -right)
    // PERP: forward offset of X encoder (+forward, -back)
    public static double PARALLEL_WHEEL_OFFSET_CM = +10.0; // your current best guess
    public static double PERP_WHEEL_OFFSET_CM     = +10.0;

    // --- SCRUB compensation (cm/rad) ---
    // Learned automatically in rotate-only mode (you can also set manually).
    public static double SCRUB_X_CM_PER_RAD = 0.0; // applies to X channel
    public static double SCRUB_Y_CM_PER_RAD = 0.0; // applies to Y channel

    // Auto-learn settings
    public static boolean AUTO_LEARN_SCRUB = true;
    public static double SCRUB_LEARN_ALPHA = 0.10; // 0..1 (EMA speed)
    public static double SCRUB_CLAMP_CM_PER_RAD = 50.0; // safety clamp

    // --- Rotate-only detection thresholds (based on your drive command) ---
    public static double ROT_ONLY_TURN_CMD_THR = 0.20;
    public static double ROT_ONLY_TRANS_CMD_THR = 0.06;

    // Minimum heading change to use for learning (avoid noise)
    public static double MIN_DTHETA_FOR_LEARN_RAD = Math.toRadians(1.0);

    // Pose (field) cm, theta continuous rad
    private double xCm, yCm, thetaRad;

    // last sensors
    private int lastXTicks, lastYTicks;
    private double lastThetaRad;

    // drive command (set by TeleOp)
    private double cmdForward = 0.0; // +forward
    private double cmdStrafe  = 0.0; // +left
    private double cmdTurn    = 0.0; // +CCW

    public Odometry(RobotHardware robot, double startXcm, double startYcm) {
        this.robot = robot;
        resetPose(startXcm, startYcm);
    }

    /**
     * IMPORTANT: call this every loop BEFORE update()
     * Use the same sign convention as your drive:
     * forward: +forward, strafe: +left, turn: +CCW
     */
    public void setDriveCommand(double forward, double strafe, double turn) {
        this.cmdForward = forward;
        this.cmdStrafe = strafe;
        this.cmdTurn = turn;
    }

    public void resetPose(double startXcm, double startYcm) {
        this.xCm = startXcm;
        this.yCm = startYcm;

        double imu = getImuYawRadSigned();
        this.thetaRad = imu;
        this.lastThetaRad = imu;

        this.lastXTicks = getXTicksSigned();
        this.lastYTicks = getYTicksSigned();
    }

    public void update() {
        int curX = getXTicksSigned();
        int curY = getYTicksSigned();
        double curTheta = getImuYawRadSigned();

        int dXticks = curX - lastXTicks;
        int dYticks = curY - lastYTicks;

        double dx_raw = dXticks / TICKS_PER_CM; // +left
        double dy_raw = dYticks / TICKS_PER_CM; // +forward

        double dTheta = angleDelta(curTheta, lastThetaRad);
        double midTheta = lastThetaRad + 0.5 * dTheta;

        // 1) rotate-only (giữ nguyên logic của bạn để LEARN scrub)
        boolean rotateOnly =
                Math.abs(cmdTurn) > ROT_ONLY_TURN_CMD_THR &&
                        Math.abs(cmdForward) < ROT_ONLY_TRANS_CMD_THR &&
                        Math.abs(cmdStrafe) < ROT_ONLY_TRANS_CMD_THR;

        // 2) turning lock: chỉ cần robot "đang xoay" là không cộng dịch chuyển
        // - OR theo lệnh turn
        // - OR theo IMU dTheta để phòng trường hợp bạn quên setDriveCommand()
        final double TURN_LOCK_CMD_THR = 0.05;                 // chỉnh theo tay ga của bạn
        final double TURN_LOCK_DTHETA_THR = Math.toRadians(0.5); // deadband chống nhiễu
        boolean turningLock =
                Math.abs(cmdTurn) > TURN_LOCK_CMD_THR ||
                        Math.abs(dTheta) > TURN_LOCK_DTHETA_THR;

        // -------- Offset compensation (center translation) --------
        double dy_center = dy_raw - (dTheta * PARALLEL_WHEEL_OFFSET_CM);
        double dx_center = dx_raw - (dTheta * PERP_WHEEL_OFFSET_CM);

        // -------- Scrub compensation (empirical) --------
        dx_center -= (dTheta * SCRUB_X_CM_PER_RAD);
        dy_center -= (dTheta * SCRUB_Y_CM_PER_RAD);

        // -------- Rotate-only: auto-learn scrub + hard lock --------
        if (rotateOnly) {
            if (AUTO_LEARN_SCRUB && Math.abs(dTheta) > MIN_DTHETA_FOR_LEARN_RAD) {
                double kx_total = dx_raw / dTheta;
                double ky_total = dy_raw / dTheta;

                double targetScrubX = kx_total - PERP_WHEEL_OFFSET_CM;
                double targetScrubY = ky_total - PARALLEL_WHEEL_OFFSET_CM;

                targetScrubX = clamp(targetScrubX, -SCRUB_CLAMP_CM_PER_RAD, SCRUB_CLAMP_CM_PER_RAD);
                targetScrubY = clamp(targetScrubY, -SCRUB_CLAMP_CM_PER_RAD, SCRUB_CLAMP_CM_PER_RAD);

                SCRUB_X_CM_PER_RAD = lerp(SCRUB_X_CM_PER_RAD, targetScrubX, SCRUB_LEARN_ALPHA);
                SCRUB_Y_CM_PER_RAD = lerp(SCRUB_Y_CM_PER_RAD, targetScrubY, SCRUB_LEARN_ALPHA);
            }

            dx_center = 0.0;
            dy_center = 0.0;
        }

        // -------- NEW: đang xoay là KHÔNG cộng dịch chuyển --------
        // (kể cả không phải rotate-only; ví dụ bạn vừa xoay vừa rê nhẹ, thì vẫn bị khóa x,y)
        if (turningLock) {
            dx_center = 0.0;
            dy_center = 0.0;
        }

        // Rotate robot-frame -> field-frame
        double cos = Math.cos(midTheta);
        double sin = Math.sin(midTheta);

        double dx_field = dx_center * cos - dy_center * sin;
        double dy_field = dx_center * sin + dy_center * cos;

        xCm += dx_field;
        yCm += dy_field;

        thetaRad = lastThetaRad + dTheta;

        lastXTicks = curX;
        lastYTicks = curY;
        lastThetaRad = thetaRad;
    }


    // ---------------- Sensors ----------------
    private int getXTicksSigned() {
        return X_ENCODER_SIGN * robot.odometry_x.getCurrentPosition();
    }

    private int getYTicksSigned() {
        return Y_ENCODER_SIGN * robot.motor_4.getCurrentPosition();
    }

    private double getImuYawRadSigned() {
        return IMU_YAW_SIGN * robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // ---------------- Utils ----------------
    private static double angleDelta(double target, double current) {
        double d = target - current;
        while (d <= -Math.PI) d += 2.0 * Math.PI;
        while (d >  Math.PI) d -= 2.0 * Math.PI;
        return d;
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ---------------- API ----------------
    public double getX() { return xCm; }
    public double getY() { return yCm; }
    public double getTheta(AngleUnit unit) { return unit.fromRadians(thetaRad); }
    public double getThetaRad() { return thetaRad; }

    // For telemetry
    public double getScrubX() { return SCRUB_X_CM_PER_RAD; }
    public double getScrubY() { return SCRUB_Y_CM_PER_RAD; }
}
