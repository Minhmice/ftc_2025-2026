package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;

public class MotorController {

    private HardwareMap map;
    public MotorController(HardwareMap map) {
        this.map = map;
    }
    MotorHardware motors = new MotorHardware(map);

    //thêm hàm điểu khiển cho motor ở đây

}
