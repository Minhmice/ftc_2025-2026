package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Logic auto 30s DECODE 2026: nhặt bóng (timing-only), bắn FIFO. Batch: COLLECT → DRIVE_TO_SHOOT → SHOOT → lặp.
 */
public class AutoRunner implements KickStateSetter {

    private final LinearOpMode opMode;
    private final RobotHardware robot;
    private final int teamColor; // 1=Red, 2=Blue
    private final Odometry odometry;
    private final Automove automove;
    private final SensorManager sensorManager;
    private final ArtifactProcessing artifactProcessing;
    private final TagProcessing tagProcessing;
    private final Turret turret;
    private final Shooter shooter;

    private final double intakeX, intakeY, shootX, shootY, parkX, parkY;
    private static final double AUTO_TIME_SEC = 28.0;
    private static final long COLLECT_TIMEOUT_MS = 8000;
    private static final int BATCH_SHOOT_COUNT = 3;

    public AutoRunner(RobotHardware robot, LinearOpMode opMode, int teamColor,
                      double startX, double startY,
                      double intakeX, double intakeY, double shootX, double shootY, double parkX, double parkY) {
        this.robot = robot;
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.intakeX = intakeX; this.intakeY = intakeY;
        this.shootX = shootX; this.shootY = shootY;
        this.parkX = parkX; this.parkY = parkY;

        odometry = new Odometry(robot, startX, startY);
        automove = new Automove(robot, odometry, opMode);
        sensorManager = new SensorManager(robot);
        artifactProcessing = new ArtifactProcessing(robot, sensorManager);
        tagProcessing = new TagProcessing(robot);
        turret = new Turret(robot, sensorManager, tagProcessing);
        shooter = new Shooter(robot, this);
    }

    @Override
    public void setKick(boolean value) { /* Shooter gọi, không cần lưu */ }

    public void run30Seconds() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        int shotsFired = 0;
        State state = State.DRIVE_TO_INTAKE;

        while (opMode.opModeIsActive() && !opMode.isStopRequested() && timer.seconds() < AUTO_TIME_SEC) {
            if (timer.seconds() >= AUTO_TIME_SEC - 2.0) state = State.PARK;
            tagProcessing.update(teamColor);
            odometry.setDriveCommand(0, 0, 0);
            odometry.update();
            artifactProcessing.update();

            switch (state) {
                case DRIVE_TO_INTAKE:
                    automove.driveToPoint(intakeX, intakeY, null);
                    state = State.COLLECT;
                    break;
                case COLLECT: {
                    artifactProcessing.setCollectorRunning(true);
                    artifactProcessing.run_collector();
                    ElapsedTime collectT = new ElapsedTime();
                    collectT.reset();
                    while (opMode.opModeIsActive() && collectT.milliseconds() < COLLECT_TIMEOUT_MS) {
                        tagProcessing.update(teamColor);
                        odometry.setDriveCommand(0, 0, 0);
                        odometry.update();
                        artifactProcessing.update();
                        if (artifactProcessing.getBallCount() >= 3) break;
                        opMode.sleep(20);
                    }
                    artifactProcessing.setCollectorRunning(false);
                    artifactProcessing.stop_collector();
                    state = State.DRIVE_TO_SHOOT;
                    break;
                }
                case DRIVE_TO_SHOOT:
                    shooter.run_flywheel_motor();
                    automove.driveToPoint(shootX, shootY, null);
                    state = State.SHOOT_ONE;
                    break;
                case SHOOT_ONE: {
                    for (int b = 0; b < BATCH_SHOOT_COUNT && opMode.opModeIsActive(); b++) {
                        if (!artifactProcessing.hasBallToShoot()) break;
                        artifactProcessing.start_pre_shoot();
                        while (opMode.opModeIsActive() && !artifactProcessing.is_ready_to_kick()) {
                            tagProcessing.update(teamColor);
                            turret.update();
                            artifactProcessing.update();
                            shooter.update();
                            opMode.sleep(20);
                        }
                        shooter.request_kick();
                        while (opMode.opModeIsActive() && shooter.isKicking()) {
                            shooter.update();
                            opMode.sleep(20);
                        }
                        artifactProcessing.request_post_shoot();
                        while (opMode.opModeIsActive() && !artifactProcessing.isShootIdle()) {
                            artifactProcessing.update();
                            opMode.sleep(20);
                        }
                        shotsFired++;
                    }
                    if (shotsFired >= 9) {
                        shooter.stop_flywheel_motor();
                        state = State.PARK;
                    } else {
                        state = State.DRIVE_TO_INTAKE;
                    }
                    break;
                }
                case PARK:
                    automove.driveToPoint(parkX, parkY, null);
                    automove.stopMotors();
                    return;
            }
            opMode.sleep(20);
        }
        automove.stopMotors();
        shooter.stop_flywheel_motor();
    }

    private enum State { DRIVE_TO_INTAKE, COLLECT, DRIVE_TO_SHOOT, SHOOT_ONE, PARK }
}
