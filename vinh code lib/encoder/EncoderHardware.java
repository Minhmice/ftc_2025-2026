package org.firstinspires.ftc.teamcode.encoder;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;

import java.util.HashMap;
import java.util.Map;

public class EncoderHardware {

    Map<String, DcMotor> encoder_map = new HashMap<>();
    public EncoderHardware(HardwareMap map) {
        for(int i = 0; i < Config.encoder_count; i++) {

            DcMotor encoder = map.get(DcMotor.class, Config.encoders[i]);

            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            encoder_map.put(Config.encoders[i], encoder);

        }
    }

    public int read_encoder(String encoder_name) {
        if(!encoder_map.containsKey(encoder_name)) return -2025;
        return encoder_map.get(encoder_name).getCurrentPosition();
    }

}
