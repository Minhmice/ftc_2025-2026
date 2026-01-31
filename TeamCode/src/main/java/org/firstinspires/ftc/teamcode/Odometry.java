package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Odometry - ước lượng vị trí (x, y, theta) từ encoder. Theta cố định 0 (không dùng IMU).
 *
 * Convention: dx_raw +left, dy_raw +forward, dTheta +CCW.
 * Field frame: xCm, yCm (cm), thetaRad (rad, continuous).
 */
public class Odometry {

    private final RobotHardware robot;

    // ===================== POD / ENCODER SPECS =====================
    public static final double WHEEL_DIAMETER_CM = 3.2;     // 32mm
    public static final double ENCODER_PPR = 2000.0;        // ticks per revolution
    public static final double TICKS_PER_CM =
            ENCODER_PPR / (Math.PI * WHEEL_DIAMETER_CM);

    // ===================== SIGN CONFIG =====================
    public static int X_ENCODER_SIGN = +1;  // odometry_x
    public static int Y_ENCODER_SIGN = -1;  // motor_4
    public static int IMU_YAW_SIGN   = +1;

    // ===================== GEOMETRY OFFSETS (cm) - theo em Vinh =====================
    // PARALLEL: lateral offset of Y encoder (+left, -right)
    // PERP: forward offset of X encoder (+forward, -back)
    public static double PARALLEL_WHEEL_OFFSET_CM = -4.0;
    public static double PERP_WHEEL_OFFSET_CM     = -15.0;

    // ===================== SCRUB COMPENSATION (cm/rad) =====================
    public static double SCRUB_X_CM_PER_RAD = 0.0;
    public static double SCRUB_Y_CM_PER_RAD = 0.0;

    public static boolean AUTO_LEARN_SCRUB = true;
    public static double SCRUB_LEARN_ALPHA = 0.10;
    public static double SCRUB_CLAMP_CM_PER_RAD = 50.0;

    public static double ROT_ONLY_TURN_CMD_THR  = 0.20;
    public static double ROT_ONLY_TRANS_CMD_THR = 0.06;
    public static double MIN_DTHETA_FOR_LEARN_RAD = Math.toRadians(1.0);

    // Chặn tick nhảy khi robot đứng yên (em Vinh)
    public static int MIN_TICKS_NOISE_X = 2;
    public static int MIN_TICKS_NOISE_Y = 2;

    private double xCm, yCm, thetaRad;
    private int lastXTicks, lastYTicks;
    private double lastThetaRad;
    private double cmdForward = 0.0, cmdStrafe = 0.0, cmdTurn = 0.0;

    // Debug
    private int lastDXticks, lastDYticks;
    private double lastDxRaw, lastDyRaw, lastDTheta;

    public Odometry(RobotHardware robot, double startXcm, double startYcm) {
        this.robot = robot;
        resetPose(startXcm, startYcm);
    }

    /** Gọi mỗi loop TRƯỚC update(). */
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
        this.lastDXticks = 0;
        this.lastDYticks = 0;
    }

    public void update() {
        int curX = getXTicksSigned();
        int curY = getYTicksSigned();
        double curTheta = getImuYawRadSigned();

        int dXticks = curX - lastXTicks;
        int dYticks = curY - lastYTicks;

        if (Math.abs(dXticks) <= MIN_TICKS_NOISE_X) dXticks = 0;
        if (Math.abs(dYticks) <= MIN_TICKS_NOISE_Y) dYticks = 0;

        double dx_raw = dXticks / TICKS_PER_CM;
        double dy_raw = dYticks / TICKS_PER_CM;

        double dTheta = angleDelta(curTheta, lastThetaRad);
        double midTheta = lastThetaRad + 0.5 * dTheta;

        lastDXticks = dXticks;
        lastDYticks = dYticks;
        lastDxRaw = dx_raw;
        lastDyRaw = dy_raw;
        lastDTheta = dTheta;

        boolean rotateOnly =
                Math.abs(cmdTurn) > ROT_ONLY_TURN_CMD_THR &&
                Math.abs(cmdForward) < ROT_ONLY_TRANS_CMD_THR &&
                Math.abs(cmdStrafe) < ROT_ONLY_TRANS_CMD_THR;

        double dy_center = dy_raw - (dTheta * PARALLEL_WHEEL_OFFSET_CM);
        double dx_center = dx_raw - (dTheta * PERP_WHEEL_OFFSET_CM);
        dx_center -= (dTheta * SCRUB_X_CM_PER_RAD);
        dy_center -= (dTheta * SCRUB_Y_CM_PER_RAD);

        if (rotateOnly) {
            if (AUTO_LEARN_SCRUB && Math.abs(dTheta) > MIN_DTHETA_FOR_LEARN_RAD) {
                double invDtheta = 1.0 / dTheta;
                double kx_total = dx_raw * invDtheta;
                double ky_total = dy_raw * invDtheta;
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

    private int getXTicksSigned() {
        return X_ENCODER_SIGN * robot.odometry_x.getCurrentPosition();
    }

    private int getYTicksSigned() {
        return Y_ENCODER_SIGN * robot.motor_4.getCurrentPosition();
    }

    /** Theta không dùng IMU: luôn 0 (robot frame = field frame). */
    private double getImuYawRadSigned() {
        return 0.0;
    }

    private static double angleDelta(double target, double current) {
        double d = target - current;
        while (d <= -Math.PI) d += 2.0 * Math.PI;
        while (d > Math.PI) d -= 2.0 * Math.PI;
        return d;
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public double getX() { return xCm; }
    public double getY() { return yCm; }
    public double getTheta(AngleUnit unit) { return unit.fromRadians(thetaRad); }
    public double getThetaRad() { return thetaRad; }
    public double getScrubX() { return SCRUB_X_CM_PER_RAD; }
    public double getScrubY() { return SCRUB_Y_CM_PER_RAD; }
    public int getLastDXticks() { return lastDXticks; }
    public int getLastDYticks() { return lastDYticks; }
    public double getLastDxRaw() { return lastDxRaw; }
    public double getLastDyRaw() { return lastDyRaw; }
    public double getLastDThetaRad() { return lastDTheta; }
}
