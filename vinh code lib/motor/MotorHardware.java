package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;

import java.util.HashMap;
import java.util.Map;


public class MotorHardware {

    private Map<String, DcMotor> motor_map = new HashMap<>();
    public MotorHardware(HardwareMap map) {
        for(int i = 0; i < Config.motor_count; i++) {
            motor_map.put(Config.motors[i], map.get(DcMotor.class, Config.motors[i]));
        }
    }

    //đặt hướng cho motor
    public void set_motor_direction(String motor_name, DcMotor.Direction direction) {
        //kiểm tra xem tồn tại motor không
        if(!motor_map.containsKey(motor_name)) return;
        //đặt hướng cho motor
        motor_map.get(motor_name).setDirection(direction);
    }

    //đặt công suất cho motor
    public void set_motor_throttle(String motor_name, double throttle) {
        //kiểm tra xem tồn tại motor không
        if(!motor_map.containsKey(motor_name)) return;
        //đặt công suất cho motor
        motor_map.get(motor_name).setPower(throttle);
    }
}
