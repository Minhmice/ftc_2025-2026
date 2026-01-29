package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;

/**
 * Đọc file cấu hình UDP (key=value) dùng chung Java + Python.
 * Đường dẫn: /sdcard/FIRST/udp_config.txt hoặc assets.
 */
public class UdpConfig {

    public static final String DEFAULT_PATH = "/sdcard/FIRST/udp_config.txt";
    private static final int MAX_PAYLOAD_BYTES = 65500;

    private String receiverHost;
    private int udpPort;
    private int cameraWidth;
    private int cameraHeight;
    private int jpegQuality;
    private boolean valid;

    public UdpConfig() {
        this(DEFAULT_PATH);
    }

    public UdpConfig(String path) {
        receiverHost = null;
        udpPort = 5000;
        cameraWidth = 320;
        cameraHeight = 240;
        jpegQuality = 70;
        valid = false;
        load(path);
    }

    private void load(String path) {
        File file = new File(path);
        if (!file.exists() || !file.canRead()) return;
        try (BufferedReader r = new BufferedReader(
                new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8))) {
            String line;
            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                int eq = line.indexOf('=');
                if (eq <= 0) continue;
                String key = line.substring(0, eq).trim().toLowerCase();
                String value = line.substring(eq + 1).trim();
                switch (key) {
                    case "receiver_host":
                        receiverHost = value.isEmpty() ? null : value;
                        break;
                    case "udp_port":
                        udpPort = parseInt(value, 5000);
                        break;
                    case "camera_width":
                        cameraWidth = parseInt(value, 320);
                        break;
                    case "camera_height":
                        cameraHeight = parseInt(value, 240);
                        break;
                    case "jpeg_quality":
                        jpegQuality = Math.max(1, Math.min(100, parseInt(value, 70)));
                        break;
                }
            }
            valid = (receiverHost != null && !receiverHost.isEmpty() && udpPort > 0 && udpPort <= 65535);
        } catch (Exception ignored) {
            valid = false;
        }
    }

    private static int parseInt(String s, int def) {
        try {
            return Integer.parseInt(s.trim());
        } catch (NumberFormatException e) {
            return def;
        }
    }

    public boolean isValid() { return valid; }
    public String getReceiverHost() { return receiverHost; }
    public int getUdpPort() { return udpPort; }
    public int getCameraWidth() { return cameraWidth; }
    public int getCameraHeight() { return cameraHeight; }
    public int getJpegQuality() { return jpegQuality; }
    public static int getMaxPayloadBytes() { return MAX_PAYLOAD_BYTES; }
}
