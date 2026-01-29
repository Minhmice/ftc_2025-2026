package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
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
    public DcMotor motor_collector, motor_sorting;

    public DcMotor odometry_x;

    // Servos
    public ServoImplEx kicking_servo;
    public ServoImplEx turret_servo;
    public ServoImplEx angle_servo;

    // IMU
    public IMU imu;

    // Vision
    public AprilTagProcessor april_tag;
    public VisionPortal visionPortal;
    /** Optional: capture frame for UDP stream. Set before init() if UdpConfig valid. */
    private FrameCaptureProcessor frameCaptureProcessor;

    // Sensors
    public TCS34725_ColorSensor color_sensor0, color_sensor1, color_sensor2;
    public DigitalChannel ir_sensor;
    public DigitalChannel turret_end_stop_left, turret_end_stop_right;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        init_motor();
        init_servo();
        init_imu();
        init_april_tag();
        init_color_sensor();
        init_ir();
        init_end_stops();
        init_odometry();
        init_voltage(); // ✅ ADD
    }

    public void initAprilTag() { init_april_tag(); }

    /** Gọi trước init() nếu muốn gửi camera qua UDP. */
    public void setFrameCaptureProcessor(FrameCaptureProcessor processor) {
        this.frameCaptureProcessor = processor;
    }

    public FrameCaptureProcessor getFrameCaptureProcessor() {
        return frameCaptureProcessor;
    }

    private void init_motor() {
        motor_1 = hardwareMap.get(DcMotor.class, "motor_1");
        motor_2 = hardwareMap.get(DcMotor.class, "motor_2");
        motor_3 = hardwareMap.get(DcMotor.class, "motor_3");
        motor_4 = hardwareMap.get(DcMotor.class, "motor_4");

        // Drive directions
        motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_2.setDirection(DcMotorSimple.Direction.REVERSE);

        // If you use motor_4 encoder for odometry Y, reset once at init
        motor_4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ✅ FIX: shooter should be DcMotorEx for velocity control
        motor_shooter = hardwareMap.get(DcMotorEx.class, "motor_shooter");
        motor_shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_collector = hardwareMap.get(DcMotor.class, "motor_collector");

        motor_sorting = hardwareMap.get(DcMotor.class, "motor_sort");
        motor_sorting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_sorting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        angle_servo = hardwareMap.get(ServoImplEx.class, "angle_servo");
        // ✅ optional but recommended for consistency
        angle_servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 20000));

        kicking_servo.setPosition(0);
        turret_servo.setPosition(0.5);
        angle_servo.setPosition(0.5);
    }

    private void init_imu() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imu_param = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imu_param);
        imu.resetYaw();
    }

    private void init_april_tag() {
        april_tag = new AprilTagProcessor.Builder().build();
        // Decimation 2–3: trade range for FPS (~22–30 FPS) so turret gets fresh frames often
        april_tag.setDecimation(2);
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "cam1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(april_tag);
        if (frameCaptureProcessor != null) {
            builder.addProcessor(frameCaptureProcessor);
        }
        visionPortal = builder.build();
    }

    private void init_color_sensor() {
        color_sensor0 = hardwareMap.get(TCS34725_ColorSensor.class, "color_sensor0");
        color_sensor1 = hardwareMap.get(TCS34725_ColorSensor.class, "color_sensor1");
        color_sensor2 = hardwareMap.get(TCS34725_ColorSensor.class, "color_sensor2");
    }

    private void init_ir() {
        ir_sensor = hardwareMap.get(DigitalChannel.class, "ir_sensor");
        ir_sensor.setMode(DigitalChannel.Mode.INPUT);
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

    public int[] getNormalizedColors(int sensor_code) {
        TCS34725_ColorSensor selectedSensor;
        switch (sensor_code) {
            case 1: selectedSensor = color_sensor1; break;
            case 2: selectedSensor = color_sensor2; break;
            case 0:
            default: selectedSensor = color_sensor0; break;
        }

        if (selectedSensor == null) return new int[]{0, 0, 0};

        int rawRed = selectedSensor.red();
        int rawGreen = selectedSensor.green();
        int rawBlue = selectedSensor.blue();

        return new int[]{rawRed, rawGreen, rawBlue};
    }
}
