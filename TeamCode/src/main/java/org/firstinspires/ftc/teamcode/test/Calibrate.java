package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ODO Calibrate (Rotate In Place) v2", group = "Calibration")
public class Calibrate extends LinearOpMode {

    // ----- Odometry pod constants -----
    private static final double WHEEL_DIAMETER_CM = 3.2;   // 32mm
    private static final double ENCODER_PPR = 2000.0;      // 2000 PPR
    private static final double TICKS_PER_CM = ENCODER_PPR / (Math.PI * WHEEL_DIAMETER_CM);

    // ----- Use your tuned signs here -----
    private static int IMU_YAW_SIGN = +1;
    private static int X_ENCODER_SIGN = -1;
    private static int Y_ENCODER_SIGN = -1;

    // Drive motors by your map:
    // motor_0: fl, motor_1: bl, motor_2: fr, motor_3: br
    private DcMotorEx fl, bl, fr, br;

    // Encoders + IMU
    private DcMotorEx encX; // odometry_x
    private DcMotorEx encY; // motor_4 (fallback br)
    private IMU imu;

    // Capture state
    private boolean capturing = false;
    private int startXTicks = 0;
    private int startYTicks = 0;
    private double startTheta = 0;

    // Debounce
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {

        // ---- Motors ----
        fl = hardwareMap.get(DcMotorEx.class, "motor_0");
        bl = hardwareMap.get(DcMotorEx.class, "motor_1");
        fr = hardwareMap.get(DcMotorEx.class, "motor_2");
        br = hardwareMap.get(DcMotorEx.class, "motor_3");

        // ---- Encoders ----
        encX = hardwareMap.get(DcMotorEx.class, "odometry_x");

        // Prefer motor_4 as your Y encoder; fallback to br if not present
        try {
            encY = hardwareMap.get(DcMotorEx.class, "motor_4");
        } catch (Exception e) {
            encY = br;
        }

        // ---- IMU ----
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);

        telemetry.addLine("ODO Rotate Calibration (motor_0..3 mapping)");
        telemetry.addLine("Right stick X: rotate");
        telemetry.addLine("A: start capture | B: stop + compute offsets");
        telemetry.addLine("Rotate smoothly ~360–540 deg.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---- Rotate control ----
            double turn = deadband(gamepad1.right_stick_x, 0.05);

            // rotate in place: left +turn, right -turn
            setDrivePowers(+turn, +turn, -turn, -turn);

            // ---- Buttons ----
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA) {
                capturing = true;
                startXTicks = getXTicksSigned();
                startYTicks = getYTicksSigned();
                startTheta = getYawRadSigned();
                telemetry.addLine("CAPTURE STARTED: rotate in place, then press B.");
            }

            if (b && !lastB) {
                if (!capturing) {
                    telemetry.addLine("Not capturing. Press A first.");
                } else {
                    capturing = false;

                    int endX = getXTicksSigned();
                    int endY = getYTicksSigned();
                    double endTheta = getYawRadSigned();

                    int dXticks = endX - startXTicks;
                    int dYticks = endY - startYTicks;
                    double dTheta = angleDelta(endTheta, startTheta);

                    double dxRawCm = dXticks / TICKS_PER_CM; // robot +left
                    double dyRawCm = dYticks / TICKS_PER_CM; // robot +forward

                    telemetry.addLine("=== RESULT ===");
                    telemetry.addData("dTheta (deg)", Math.toDegrees(dTheta));
                    telemetry.addData("dxRaw (cm)", dxRawCm);
                    telemetry.addData("dyRaw (cm)", dyRawCm);

                    if (Math.abs(dTheta) < 1e-6) {
                        telemetry.addLine("INVALID: dTheta too small. Rotate more.");
                    } else {
                        // Pure rotation model:
                        // dy_raw ≈ x_offset * dTheta  (x_offset of Y encoder; +left, -right)
                        // dx_raw ≈ y_offset * dTheta  (y_offset of X encoder; +forward, -back)
                        double estParallelOffset = dyRawCm / dTheta; // paste into PARALLEL_WHEEL_OFFSET_CM
                        double estPerpOffset = dxRawCm / dTheta;     // paste into PERP_WHEEL_OFFSET_CM

                        telemetry.addLine("Estimated offsets to paste into Odometry (cm):");
                        telemetry.addData("PARALLEL_WHEEL_OFFSET_CM (Y encoder)", estParallelOffset);
                        telemetry.addData("PERP_WHEEL_OFFSET_CM (X encoder)", estPerpOffset);

                        telemetry.addLine("Copy lines:");
                        telemetry.addLine(String.format("PARALLEL_WHEEL_OFFSET_CM = %.3f;", estParallelOffset));
                        telemetry.addLine(String.format("PERP_WHEEL_OFFSET_CM     = %.3f;", estPerpOffset));

                        telemetry.addLine("If dTheta sign is opposite of your rotation, flip IMU_YAW_SIGN.");
                    }
                }
            }

            lastA = a;
            lastB = b;

            // ---- Live debug ----
            telemetry.addData("Yaw (deg)", Math.toDegrees(getYawRadSigned()));
            telemetry.addData("X ticks (signed)", getXTicksSigned());
            telemetry.addData("Y ticks (signed)", getYTicksSigned());
            telemetry.addData("Ticks/cm", TICKS_PER_CM);
            telemetry.addLine("A start | B compute | RightStickX rotate");
            telemetry.update();

            sleep(10);
        }

        setDrivePowers(0, 0, 0, 0);
    }

    // ----- signed sensors -----
    private int getXTicksSigned() {
        return X_ENCODER_SIGN * encX.getCurrentPosition();
    }

    private int getYTicksSigned() {
        return Y_ENCODER_SIGN * encY.getCurrentPosition();
    }

    private double getYawRadSigned() {
        return IMU_YAW_SIGN * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // ----- utils -----
    private static double angleDelta(double target, double current) {
        double d = target - current;
        while (d <= -Math.PI) d += 2.0 * Math.PI;
        while (d > Math.PI) d -= 2.0 * Math.PI;
        return d;
    }

    private static double deadband(double v, double db) {
        return Math.abs(v) < db ? 0.0 : v;
    }

    private void setDrivePowers(double flP, double blP, double frP, double brP) {
        fl.setPower(flP);
        bl.setPower(blP);
        fr.setPower(frP);
        br.setPower(brP);
    }
}
