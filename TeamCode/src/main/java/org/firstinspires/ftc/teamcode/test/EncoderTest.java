package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(group = "TEST")
public class EncoderTest extends LinearOpMode {

    GoBildaPinpointDriver.GoBildaOdometryPods odometry;

    @Override
    public void runOpMode(){
        odometry = hardwareMap.get(GoBildaPinpointDriver.GoBildaOdometryPods.class, "odometry");

    }
}
