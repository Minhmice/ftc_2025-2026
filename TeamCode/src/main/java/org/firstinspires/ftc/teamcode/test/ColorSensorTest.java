package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TCS34725_ColorSensor;

@TeleOp(group = "TEST")
public class ColorSensorTest extends LinearOpMode {

    RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {

        TCS34725_ColorSensor colorSensor = hardwareMap.get(TCS34725_ColorSensor.class, "color_sensor");

        waitForStart();

        while (opModeIsActive()) {
            // Lấy giá trị màu thô từ cảm biến
            int rawRed = colorSensor.red();
            int rawGreen = colorSensor.green();
            int rawBlue = colorSensor.blue();
            int alpha = colorSensor.clear();



            // Hiển thị giá trị RGB đã được chuẩn hóa (0-255)
            telemetry.addData("Red (Normalized)", rawRed);
            telemetry.addData("Green (Normalized)", rawGreen);
            telemetry.addData("Blue (Normalized)", rawBlue);
            telemetry.addLine("---");


            telemetry.update();
        }
    }
}
