package org.firstinspires.ftc.teamcode.encoder;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Config;

import java.util.HashMap;
import java.util.Map;

public class EncoderProcessing {

    EncoderHardware encoders;

    Map<String, Double> encoder_dist = new HashMap<>();

    Map<String, Double> encoder_delta = new HashMap<>();

    Map<String, Integer> last_pos = new HashMap<>();

    public EncoderProcessing(HardwareMap map) {
        encoders = new EncoderHardware(map);

        for (int i = 0; i < Config.encoder_count; i++) {
            String name = Config.encoders[i];

            encoder_dist.put(name, 0d);
            encoder_delta.put(name, 0d);

            last_pos.put(name, 0);
        }
    }

    public double get_dist(String encoder_name) {
        return encoder_dist.get(encoder_name);
    }

    public double get_delta(String encoder_name) {
        return encoder_delta.get(encoder_name);
    }

    public void update() {

        for (int i = 0; i < Config.encoder_count; i++) {

            String name = Config.encoders[i];

            int new_pos = encoders.read_encoder(name);
            int old_pos = last_pos.get(name);

            int delta_pos = new_pos - old_pos;

            if (delta_pos > 30000) delta_pos -= 65536;
            if (delta_pos < -30000) delta_pos += 65536;

            double delta_rev = (double) delta_pos / Config.TICKS_PER_REVOL;

            double delta_dist = delta_rev * Config.C;

            encoder_delta.put(name, delta_dist);

            encoder_dist.put(name, encoder_dist.get(name) + delta_dist);

            last_pos.put(name, new_pos);
        }
    }
}
