package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TELEOP")
public class Main extends LinearOpMode {

    // ================== SHOOTER STATE ==================
    public boolean kick;
    public boolean shooter_active = false;

    // giữ để telemetry/debug (servo angle thực tế)
    public double shooting_angle = 0;

    // luôn 90% – KHÔNG chỉnh bằng dpad nữa
    public static final double SHOOTER_POWER_FIXED = 0.90;

    // Auto-angle controls (edge-trigger)
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastBack = false;

    // ================== SYSTEM ==================
    public int team_color;
    RobotHardware robot;
    SensorManager sensorManager;
    Movement movement;
    GamepadController gamepadController;
    ArtifactProcessing artifactProcessing;
    Turret turret;
    TagProcessing tagProcessing;
    Odometry odometry;
    Shooter shooter;

    // Shoot sequence flags
    private boolean inshoot = false;
    private boolean lastTriangle = false;
    private boolean kickStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);
        robot.init();

        sensorManager = new SensorManager(robot);
        gamepadController = new GamepadController(gamepad1, gamepad2);
        movement = new Movement(robot, gamepadController);
        artifactProcessing = new ArtifactProcessing(robot, sensorManager);
        tagProcessing = new TagProcessing(robot);
        turret = new Turret(robot, sensorManager, gamepadController, this);
        odometry = new Odometry(robot, 0, 0);
        shooter = new Shooter(robot, this);

        waitForStart();

        movement.setMaxSpeed(0.25);
        movement.setDeadzone(0.1);

        while (opModeIsActive()) {

            // ================== INPUT ==================
            gamepadController.update_gamepad();

            // ================== VISION ==================
            tagProcessing.update(team_color);

            // ================== AUTO SHOOTING ANGLE ==================
            // Y / A: trim nhỏ (edge-trigger)
            boolean y = gamepad2.y;
            boolean a = gamepad2.a;

            if (y && !lastY) shooter.nudgeAngleTrim(+0.01);
            if (a && !lastA) shooter.nudgeAngleTrim(-0.01);

            lastY = y;
            lastA = a;

            // BACK: bật / tắt auto-angle
            boolean back = gamepad2.back;
            if (back && !lastBack) {
                shooter.setAutoAngleEnabled(!shooter.isAutoAngleEnabled());
            }
            lastBack = back;

            // Cập nhật góc bắn theo range AprilTag
            shooter.updateAutoAngle(tagProcessing);

            shooting_angle = robot.angle_servo.getPosition();

            // ================== ARTIFACT ==================
            artifactProcessing.update();

            // ================== DRIVE ==================
            movement.move_robot();

            // ================== ODOMETRY ==================
            odometry.setDriveCommand(
                    movement.getCmdForward(),
                    movement.getCmdStrafe(),
                    movement.getCmdTurn()
            );
            odometry.update();

            // ================== TURRET ==================
            turret.update();

            // ================== INTAKE ==================
            if (gamepad2.circle) {
                artifactProcessing.run_collecter();
            } else if (gamepad2.square) {
                artifactProcessing.stop_collecter();
            }

            // ================== FLYWHEEL ==================
            if (gamepad2.left_bumper) {
                shooter_active = true;
            } else if (gamepad2.right_bumper) {
                shooter_active = false;
            }

            if (shooter_active) {
                // velocity-hold 90% (ổn định dù pin yếu)
                shooter.run_flywheel_motor();
            } else {
                shooter.stop_flywheel_motor();
            }

            // ================== SHOOT SEQUENCE ==================
            boolean triangle = gamepad2.triangle;
            if (triangle && !lastTriangle) {
                inshoot = true;
                kickStarted = false;
                artifactProcessing.start_pre_shoot();
            }
            lastTriangle = triangle;

            if (inshoot) {
                if (artifactProcessing.is_ready_to_kick()) {
                    shooter.request_kick();
                }

                shooter.update();

                if (shooter.isKicking()) {
                    kickStarted = true;
                }

                if (kickStarted && !shooter.isKicking()) {
                    artifactProcessing.request_post_shoot();
                    inshoot = false;
                    kickStarted = false;
                }
            } else {
                shooter.update();
            }

            // ================== TELEMETRY ==================
            telemetry.addData("AutoAngle", shooter.isAutoAngleEnabled());
            telemetry.addData("AnglePos", "%.3f", shooting_angle);
            telemetry.addData("TagDetected", tagProcessing.isDetected());
            telemetry.addData("TagRange(in)", "%.1f", tagProcessing.getRangeInches());

            telemetry.addData("Flywheel", shooter_active ? "ON" : "OFF");

            telemetry.addData("Yaw (deg)",
                    robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("X (cm)", "%.2f", odometry.getX());
            telemetry.addData("Y (cm)", "%.2f", odometry.getY());

            telemetry.addData("Slots [0,1,2]", "%d %d %d",
                    artifactProcessing.artifact_slots[0],
                    artifactProcessing.artifact_slots[1],
                    artifactProcessing.artifact_slots[2]);

            telemetry.addData("Queue [0,1,2]", "%d %d %d",
                    artifactProcessing.artifact_queue[0],
                    artifactProcessing.artifact_queue[1],
                    artifactProcessing.artifact_queue[2]);

            telemetry.addData("ShootState",
                    artifactProcessing.is_ready_to_kick()
                            ? "READY"
                            : (inshoot ? "PREP" : "IDLE"));

            telemetry.update();
        }
    }
}
