package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.motor.MotorController;

@Autonomous(name = "Main")
public class Main extends LinearOpMode {
    public MotorController motor_controller;
    @Override
    public void runOpMode() throws InterruptedException {
        motor_controller = new MotorController(hardwareMap);
    }

}
