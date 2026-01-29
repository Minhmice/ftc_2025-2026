package org.firstinspires.ftc.teamcode;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Gửi log (type 0x00) và frame JPEG (type 0x01) qua UDP tới receiver_host:udp_port.
 * Chạy trong background thread; Main đẩy telemetry string và trigger gửi.
 */
public class UdpLogger {

    private static final byte TYPE_LOG = 0x00;
    private static final byte TYPE_JPEG = 0x01;
    private static final int MAX_PAYLOAD = 65000;

    private final String receiverHost;
    private final int udpPort;
    private final String senderIp;
    private DatagramSocket socket;
    private final AtomicReference<String> pendingLog = new AtomicReference<>(null);
    private final AtomicReference<byte[]> pendingJpeg = new AtomicReference<>(null);
    private volatile boolean running = true;
    private Thread senderThread;

    public UdpLogger(UdpConfig config) {
        this.receiverHost = config.getReceiverHost();
        this.udpPort = config.getUdpPort();
        this.senderIp = getLocalIpAddress();
    }

    /** Bắt đầu thread gửi. Gọi sau khi config hợp lệ. */
    public void start() {
        try {
            socket = new DatagramSocket();
            senderThread = new Thread(this::runSender);
            senderThread.setDaemon(true);
            senderThread.start();
        } catch (Exception ignored) {
            socket = null;
        }
    }

    /** Dừng và đóng socket. */
    public void stop() {
        running = false;
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }

    /** Đẩy log (sẽ prefix [senderIp] khi gửi). Gọi từ Main sau updateTelemetry. */
    public void pushLog(String text) {
        if (text != null && !text.isEmpty()) {
            pendingLog.set(text);
        }
    }

    /** Đẩy JPEG từ FrameCaptureProcessor. Gọi từ Main/thread. */
    public void pushJpeg(byte[] jpeg) {
        if (jpeg != null && jpeg.length > 0 && jpeg.length < MAX_PAYLOAD) {
            pendingJpeg.set(jpeg);
        }
    }

    /** Trả về IP máy gửi (robot) để prefix log. */
    public String getSenderIp() {
        return senderIp != null ? senderIp : "0.0.0.0";
    }

    private void runSender() {
        long lastLogNs = 0;
        long lastJpegNs = 0;
        final long logIntervalNs = 100_000_000L;   // 10 Hz
        final long jpegIntervalNs = 66_000_000L;  // ~15 Hz
        while (running && socket != null && !socket.isClosed()) {
            long now = System.nanoTime();
            String log = pendingLog.getAndSet(null);
            if (log != null && (now - lastLogNs) >= logIntervalNs) {
                sendLog(log);
                lastLogNs = now;
            }
            byte[] jpeg = pendingJpeg.getAndSet(null);
            if (jpeg != null && (now - lastJpegNs) >= jpegIntervalNs) {
                sendFrame(jpeg);
                lastJpegNs = now;
            }
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    private void sendLog(String text) {
        String prefix = "[" + getSenderIp() + "] ";
        String[] lines = text.split("\n");
        StringBuilder sb = new StringBuilder();
        for (String line : lines) {
            if (sb.length() > 0) sb.append("\n");
            sb.append(prefix).append(line.trim());
        }
        byte[] payload = sb.toString().getBytes(StandardCharsets.UTF_8);
        if (payload.length > MAX_PAYLOAD) {
            payload = (prefix + "(truncated)").getBytes(StandardCharsets.UTF_8);
        }
        send(TYPE_LOG, payload);
    }

    private void sendFrame(byte[] jpeg) {
        send(TYPE_JPEG, jpeg);
    }

    private void send(byte type, byte[] payload) {
        if (socket == null || receiverHost == null) return;
        try {
            InetAddress addr = InetAddress.getByName(receiverHost);
            byte[] buf = new byte[1 + payload.length];
            buf[0] = type;
            System.arraycopy(payload, 0, buf, 1, payload.length);
            DatagramPacket p = new DatagramPacket(buf, buf.length, addr, udpPort);
            socket.send(p);
        } catch (Exception ignored) {
        }
    }

    private static String getLocalIpAddress() {
        try {
            java.util.Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces();
            while (en.hasMoreElements()) {
                NetworkInterface intf = en.nextElement();
                if (intf.isLoopback() || !intf.isUp()) continue;
                java.util.Enumeration<InetAddress> addrs = intf.getInetAddresses();
                while (addrs.hasMoreElements()) {
                    InetAddress a = addrs.nextElement();
                    if (a.isLoopbackAddress()) continue;
                    String host = a.getHostAddress();
                    if (host == null) continue;
                    if (host.contains("%")) host = host.substring(0, host.indexOf('%'));
                    if (host.indexOf(':') < 0) return host; // prefer IPv4
                }
            }
        } catch (Exception ignored) {
        }
        return "0.0.0.0";
    }
}
