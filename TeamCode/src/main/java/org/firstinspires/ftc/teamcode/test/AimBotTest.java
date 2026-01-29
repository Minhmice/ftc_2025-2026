package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp(name = "AimBotTest", group = "TEST")
public class AimBotTest extends LinearOpMode {

    private boolean detected;
    private double bearing;
    private double range;
    private double distanceToCenter = -1;
    private DcMotor motor;


    public AprilTagProcessor april_tag;
    public VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        init_april_tag();
        waitForStart();
        while (opModeIsActive()) {
            update(2);
            if(isDetected()) {
                telemetry.addData("Distance", getDistanceToCenter());
                if(getDistanceToCenter() > 50) {
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor.setPower(0.05);
                } else if(getDistanceToCenter() < -50){
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor.setPower(0.05);
                } else {
                    motor.setPower(0);
                }

            } else {
                motor.setPower(0);
            }
            telemetry.update();
        }
    }

    private void init_april_tag() {
        april_tag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "cam1"))
                .addProcessor(april_tag)
                .build();
    }

    public void update(int team_color) {
        this.detected = false;

        List<AprilTagDetection> tags = april_tag.getDetections();
        if (tags != null && !tags.isEmpty()) {
            int targetId = (team_color == 1) ? 24 : 20;

            for (AprilTagDetection tag : tags) {
                if (tag.id == targetId) {
                    this.detected = true;
                    this.bearing = tag.ftcPose.bearing;
                    this.range = tag.ftcPose.range;
                    this.distanceToCenter = tag.center.x;

                    break;
                }
            }
        }
    }

    public boolean isDetected() {
        return detected;
    }

    public double getBearing() {
        return bearing;
    }

    public double getRange() {
        return range;
    }

    public double getDistanceToCenter() {
        return distanceToCenter - 240;
    }
}
