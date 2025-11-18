package org.firstinspires.ftc.teamcode.pid;

public class PID {

    private double kP, kI, kD;
    private double integral = 0;
    private double prev_error = 0;
    private double integral_limit = Double.MAX_VALUE;
    private double output_min = -1;
    private double output_max = 1;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    //tính toán PID
    public double update(double target, double current, double dt) {
        double error = target - current;

        // tích phân
        integral += error * dt;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;

        // đạo hàm
        double derivative = (error - prev_error) / dt;

        // PID output
        double output = kP * error + kI * integral + kD * derivative;

        // chuẩn hóa output
        output = Math.max(output_min, Math.min(output_max, output));

        prev_error = error;
        return output;
    }

    //reset PID
    public void reset() {
        integral = 0;
        prev_error = 0;
    }

    //chuẩn hóa đầu ra
    public void set_output_range(double min, double max) {
        output_min = min;
        output_max = max;
    }

}
