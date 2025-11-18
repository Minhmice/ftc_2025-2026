package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;


public class MotorHardware {
    private final DcMotor motor_list[] = new DcMotor[Config.motor_count];
    public MotorHardware(HardwareMap map) {
        for(int i = 0; i < Config.motor_count; i++) {
            motor_list[i] = map.get(DcMotor.class, Config.motors[i]);
        }
    }

    //đặt hướng cho motor
    public void set_motor_direction(MotorMap motor_name, DcMotor.Direction direction) {
        //kiểm tra xem tồn tại motor không
        if(motor_name.index < 0 || motor_name.index >= Config.motor_count) return;

        //đặt hướng cho motor
        motor_list[motor_name.index].setDirection(direction);
    }

    //đặt công suất cho motor
    public void set_motor_throttle(MotorMap motor_name, double throttle) {
        //kiểm tra xem tồn tại motor không
        if(motor_name.index < 0 || motor_name.index >= Config.motor_count) return;

        //đặt công suất cho motor
        motor_list[motor_name.index].setPower(throttle);
    }
}
