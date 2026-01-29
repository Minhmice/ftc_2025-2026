package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(group = "TEST")
public class MotorTest extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode(){
        waitForStart();
        while(opModeIsActive()) {

            motor = hardwareMap.get(DcMotor.class, "motor_4");

            motor.setPower(1);
        }
    }
}
