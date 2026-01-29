package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Automove {
    private final RobotHardware robot;
    private final Odometry odometry;
    private final LinearOpMode opMode;

    private final ElapsedTime runtime = new ElapsedTime();

    // ===================== TUNING (NO PID) =====================
    // P-gain đơn giản (closed-loop thường)
    private double kPos = 0.06;        // power per cm (tăng nếu ì, giảm nếu rung)
    private double kTurn = 2.2;        // power per rad

    // giới hạn công suất
    private double maxTransPower = 0.7;
    private double maxTurnPower  = 0.6;

    // tối thiểu để thắng ma sát (nếu robot bị đứng yên khi gần mục tiêu)
    private double minTransPower = 0.12;
    private double minTurnPower  = 0.10;

    // tolerance
    private static final double POS_TOL_CM = 2.0;
    private static final double HEADING_TOL_DEG = 2.0;

    // ramp nhẹ chống giật (có thể bỏ nếu không cần)
    private double transSlewRate = 4.0; // power/sec
    private double turnSlewRate  = 6.0;

    private double lastFwd = 0, lastStrafe = 0, lastTurn = 0;

    public Automove(RobotHardware robot, Odometry odometry, LinearOpMode opMode) {
        this.robot = robot;
        this.odometry = odometry;
        this.opMode = opMode;
    }

    // ===================== CONFIG APIs =====================
    public void setGains(double kPos, double kTurn) {
        this.kPos = kPos;
        this.kTurn = kTurn;
    }

    public void setPowerLimits(double maxTrans, double maxTurn, double minTrans, double minTurn) {
        this.maxTransPower = clip(maxTrans, 0, 1);
        this.maxTurnPower  = clip(maxTurn, 0, 1);
        this.minTransPower = clip(minTrans, 0, 1);
        this.minTurnPower  = clip(minTurn, 0, 1);
    }

    public void setSlewRates(double transRate, double turnRate) {
        this.transSlewRate = Math.max(0, transRate);
        this.turnSlewRate  = Math.max(0, turnRate);
    }

    public void resetFilters() {
        lastFwd = lastStrafe = lastTurn = 0;
    }

    // =========================================================
    // 1) HÀM HỖ TRỢ: DI CHUYỂN MECANUM (ROBOT-CENTRIC)
    // forward: +forward, strafe: +left, turn: +CCW
    // =========================================================
    public void driveMecanumRobotCentric(double forward, double strafe, double turn) {
        // clamp
        forward = clip(forward, -1, 1);
        strafe  = clip(strafe,  -1, 1);
        turn    = clip(turn,    -1, 1);

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        // mapping theo code bạn
        robot.motor_1.setPower(fl);
        robot.motor_2.setPower(bl);
        robot.motor_3.setPower(fr);
        robot.motor_4.setPower(br);
    }

    // =========================================================
    // 2) HÀM HỖ TRỢ: XOAY TẠI CHỖ ĐẾN HEADING
    // =========================================================
    public void turnToHeading(double targetHeadingDeg) {
        resetFilters();
        runtime.reset();

        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            // feed command cho odom (đang xoay)
            odometry.setDriveCommand(0, 0, lastTurn);
            odometry.update();

            double curHeading = odometry.getTheta(AngleUnit.RADIANS);
            double targetRad = Math.toRadians(targetHeadingDeg);
            double err = AngleUnit.normalizeRadians(targetRad - curHeading);

            double dt = runtime.seconds();
            runtime.reset();
            if (dt <= 1e-6) dt = 0.02;

            double errDeg = Math.abs(Math.toDegrees(err));
            if (errDeg <= HEADING_TOL_DEG) break;

            // closed-loop thường (P-only)
            double turn = kTurn * err;

            // clamp & min
            turn = clip(turn, -maxTurnPower, maxTurnPower);
            if (Math.abs(turn) < minTurnPower) turn = Math.signum(turn) * minTurnPower;

            // ramp
            turn = slew(turn, lastTurn, turnSlewRate, dt);
            lastTurn = turn;

            // quay tại chỗ
            driveMecanumRobotCentric(0, 0, turn);

            opMode.telemetry.addData("TurnTo", "target=%.0f cur=%.1f err=%.1f deg",
                    targetHeadingDeg, Math.toDegrees(curHeading), Math.toDegrees(err));
            opMode.telemetry.addData("turnCmd", "%.2f", turn);
            opMode.telemetry.update();

            opMode.sleep(20);
        }

        stopMotors();
        lastTurn = 0;
    }

    // =========================================================
    // DRIVE TO POINT (x,y) bằng mecanum (field-centric), tolerance 2cm
    // Option: giữ heading mong muốn trong khi chạy
    // =========================================================
    public void driveToPoint(double targetXcm, double targetYcm, Double holdHeadingDegOrNull) {
        resetFilters();
        runtime.reset();

        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {

            // feed command loop trước cho odom
            odometry.setDriveCommand(lastFwd, lastStrafe, lastTurn);

            odometry.update();
            double x = odometry.getX();
            double y = odometry.getY();
            double heading = odometry.getTheta(AngleUnit.RADIANS);

            double dt = runtime.seconds();
            runtime.reset();
            if (dt <= 1e-6) dt = 0.02;

            // error field (X=left, Y=forward)
            double errX = targetXcm - x;
            double errY = targetYcm - y;

            double dist = Math.hypot(errX, errY);
            if (dist <= POS_TOL_CM) break;

            // ----- Translation command (field) : P-only -----
            // cmdFieldX (+left), cmdFieldY (+forward)
            double cmdFieldX = kPos * errX;
            double cmdFieldY = kPos * errY;

            // limit vector magnitude
            double[] limited = limitVector(cmdFieldX, cmdFieldY, maxTransPower);
            cmdFieldX = limited[0];
            cmdFieldY = limited[1];

            // add min power để thắng ma sát (nhưng vẫn giữ hướng)
            double mag = Math.hypot(cmdFieldX, cmdFieldY);
            if (mag > 1e-6 && mag < minTransPower) {
                double s = minTransPower / mag;
                cmdFieldX *= s;
                cmdFieldY *= s;
            }

            // ----- Heading hold (optional) -----
            double turnCmd = 0.0;
            if (holdHeadingDegOrNull != null) {
                double targetRad = Math.toRadians(holdHeadingDegOrNull);
                double errH = AngleUnit.normalizeRadians(targetRad - heading);
                if (Math.abs(Math.toDegrees(errH)) > HEADING_TOL_DEG) {
                    turnCmd = kTurn * errH;
                    turnCmd = clip(turnCmd, -maxTurnPower, maxTurnPower);
                    if (Math.abs(turnCmd) < minTurnPower) turnCmd = Math.signum(turnCmd) * minTurnPower;
                }
            }

            // ----- Field -> Robot (rotate by -heading) -----
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            double robotStrafe  = (cmdFieldX * cos) + (cmdFieldY * sin);     // +left
            double robotForward = (-cmdFieldX * sin) + (cmdFieldY * cos);    // +forward

            // ramp
            robotForward = slew(robotForward, lastFwd, transSlewRate, dt);
            robotStrafe  = slew(robotStrafe,  lastStrafe, transSlewRate, dt);
            turnCmd      = slew(turnCmd,      lastTurn, turnSlewRate,  dt);

            lastFwd = robotForward;
            lastStrafe = robotStrafe;
            lastTurn = turnCmd;

            driveMecanumRobotCentric(robotForward, robotStrafe, turnCmd);

            opMode.telemetry.addData("DriveTo", "T(%.1f,%.1f) C(%.1f,%.1f) dist=%.2f",
                    targetXcm, targetYcm, x, y, dist);
            opMode.telemetry.addData("Err", "x=%.1f y=%.1f", errX, errY);
            opMode.telemetry.addData("CmdRobot", "fwd=%.2f str=%.2f turn=%.2f",
                    robotForward, robotStrafe, turnCmd);
            opMode.telemetry.update();

            opMode.sleep(20);
        }

        stopMotors();
        resetFilters();
    }

    // Shortcut: đi đến (x,y) không giữ heading
    public void driveToPoint(double targetXcm, double targetYcm) {
        driveToPoint(targetXcm, targetYcm, null);
    }

    // ===================== UTILS =====================
    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double slew(double target, double current, double slewRate, double dt) {
        if (slewRate <= 0) return target;
        double maxDelta = slewRate * dt;
        double delta = target - current;
        if (delta >  maxDelta) delta =  maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        return current + delta;
    }

    private static double[] limitVector(double x, double y, double maxMag) {
        double mag = Math.hypot(x, y);
        if (mag <= maxMag || mag < 1e-9) return new double[]{x, y};
        double s = maxMag / mag;
        return new double[]{x * s, y * s};
    }

    public void stopMotors() {
        robot.motor_1.setPower(0);
        robot.motor_2.setPower(0);
        robot.motor_3.setPower(0);
        robot.motor_4.setPower(0);
    }
}
