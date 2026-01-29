package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FrameCaptureProcessor;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.UdpConfig;
import org.firstinspires.ftc.teamcode.UdpLogger;

/**
 * OpMode test: chỉ gửi telemetry + video qua UDP (sender IP, receiver, JPEG).
 * Không drive, không shooter. Dùng để kiểm tra UDP và truyền ảnh.
 */
@TeleOp(name = "Video Sender Op", group = "TEST")
public class VideoSenderOp extends LinearOpMode {

    private RobotHardware robot;
    private UdpConfig udpConfig;
    private FrameCaptureProcessor frameCaptureProcessor;
    private UdpLogger udpLogger;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        udpConfig = new UdpConfig(UdpConfig.DEFAULT_PATH);
        if (udpConfig.isValid()) {
            frameCaptureProcessor = new FrameCaptureProcessor(udpConfig);
            robot.setFrameCaptureProcessor(frameCaptureProcessor);
        }
        robot.init();

        waitForStart();

        if (udpConfig != null && udpConfig.isValid()) {
            udpLogger = new UdpLogger(udpConfig);
            udpLogger.start();
        }

        while (opModeIsActive()) {
            String senderIp = (udpLogger != null) ? udpLogger.getSenderIp() : "N/A";
            String receiver = (udpConfig != null && udpConfig.isValid())
                    ? udpConfig.getReceiverHost() + ":" + udpConfig.getUdpPort()
                    : "N/A";
            String logLine = String.format("UDP sender IP: %s\nReceiver: %s\nSending...", senderIp, receiver);
            if (udpLogger != null) {
                udpLogger.pushLog(logLine);
                byte[] jpeg = frameCaptureProcessor != null ? frameCaptureProcessor.getLastJpeg() : null;
                if (jpeg != null) udpLogger.pushJpeg(jpeg);
            }

            telemetry.addData("UDP sender IP", senderIp);
            telemetry.addData("Receiver", receiver);
            telemetry.addData("Status", udpLogger != null ? "OK" : "No config");
            telemetry.update();
        }

        if (udpLogger != null) udpLogger.stop();
    }
}
