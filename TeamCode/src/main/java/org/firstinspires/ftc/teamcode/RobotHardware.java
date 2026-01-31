package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotHardware {

    // ✅ FIX: voltage sensor must be an object, not double
    public VoltageSensor voltageSensor;
    private final HardwareMap hardwareMap;

    // Motors
    public DcMotor motor_1, motor_2, motor_3, motor_4;
    public DcMotorEx motor_shooter;                 // ✅ FIX: DcMotorEx for setVelocity()
    public DcMotor motor_collector;

    public DcMotor odometry_x;

    // Servos (servo_sort = REV Smart Servo 360° continuous, cấu hình bằng SRS Programmer)
    public ServoImplEx kicking_servo;
    public ServoImplEx turret_servo;
    public ServoImplEx servo_sort;

    // Vision
    public AprilTagProcessor april_tag;
    public VisionPortal visionPortal;

    // Sensors (turret limit switches)
    public DigitalChannel turret_end_stop_left, turret_end_stop_right;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        init_motor();
        init_servo();
        init_april_tag();
        init_end_stops();
        init_odometry();
        init_voltage(); // ✅ ADD
    }

    public void initAprilTag() { init_april_tag(); }

    private void init_motor() {
        motor_1 = hardwareMap.get(DcMotor.class, "motor_1");
        motor_2 = hardwareMap.get(DcMotor.class, "motor_2");
        motor_3 = hardwareMap.get(DcMotor.class, "motor_3");
        motor_4 = hardwareMap.get(DcMotor.class, "motor_4");

        // Drive directions (theo em Vinh: motor_3, motor_4 REVERSE)
        motor_3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_4.setDirection(DcMotorSimple.Direction.REVERSE);

        // If you use motor_4 encoder for odometry Y, reset once at init
        motor_4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ✅ FIX: shooter should be DcMotorEx for velocity control
        motor_shooter = hardwareMap.get(DcMotorEx.class, "motor_shooter");
        motor_shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_collector = hardwareMap.get(DcMotor.class, "motor_collector");
    }

    private void init_odometry() {
        odometry_x = hardwareMap.get(DcMotor.class, "odometry_x");
        odometry_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ✅ FIX: must set a running mode after reset
        odometry_x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void init_servo() {
        kicking_servo = hardwareMap.get(ServoImplEx.class, "kicking_servo");
        kicking_servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 20000));

        turret_servo = hardwareMap.get(ServoImplEx.class, "turret_servo");
        turret_servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 20000));

        servo_sort = hardwareMap.get(ServoImplEx.class, "servo_sort");
        servo_sort.setPwmRange(new PwmControl.PwmRange(500, 2500, 20000));
        servo_sort.setPosition(0.5);

        kicking_servo.setPosition(0);
        turret_servo.setPosition(0.5);
    }

    private void init_april_tag() {
        april_tag = new AprilTagProcessor.Builder().build();
        // Decimation 2–3: trade range for FPS (~22–30 FPS) so turret gets fresh frames often
        april_tag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "cam1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(april_tag)
                .build();
    }

    private void init_end_stops() {
        turret_end_stop_left = hardwareMap.get(DigitalChannel.class, "turret_end_stop_left");
        turret_end_stop_right = hardwareMap.get(DigitalChannel.class, "turret_end_stop_right");
        turret_end_stop_left.setMode(DigitalChannel.Mode.INPUT);
        turret_end_stop_right.setMode(DigitalChannel.Mode.INPUT);
    }

    // ✅ ADD: voltage sensor init
    private void init_voltage() {
        if (hardwareMap.voltageSensor != null && hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
    }

    // ✅ ADD: safe battery voltage getter (use min of all sensors)
    public double getBatteryVoltage() {
        double minV = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double v = s.getVoltage();
            if (v > 0) minV = Math.min(minV, v);
        }
        return (minV == Double.POSITIVE_INFINITY) ? 12.0 : minV;
    }

}
