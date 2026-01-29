package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.ByteArrayOutputStream;
import java.util.concurrent.atomic.AtomicReference;

/**
 * VisionProcessor lấy frame camera, resize và encode JPEG để gửi UDP.
 * Lưu lastJpeg (thread-safe) cho UdpLogger đọc.
 */
public class FrameCaptureProcessor implements VisionProcessor {

    private final int targetWidth;
    private final int targetHeight;
    private final int jpegQuality;

    private final AtomicReference<byte[]> lastJpeg = new AtomicReference<>(null);
    private int frameWidth;
    private int frameHeight;

    public FrameCaptureProcessor(UdpConfig config) {
        this.targetWidth = config.getCameraWidth();
        this.targetHeight = config.getCameraHeight();
        this.jpegQuality = config.getJpegQuality();
    }

    @Override
    public void init(int width, int height, org.firstinspires.ftc.vision.CameraCalibration calibration) {
        frameWidth = width;
        frameHeight = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame == null || frame.empty()) return null;
        try {
            Mat resized = new Mat();
            Imgproc.resize(frame, resized, new Size(targetWidth, targetHeight));
            MatOfByte buf = new MatOfByte();
            Imgcodecs.imencode(".jpg", resized, buf);
            byte[] jpeg = buf.toArray();
            resized.release();
            buf.release();
            if (jpeg != null && jpeg.length < UdpConfig.getMaxPayloadBytes()) {
                lastJpeg.set(jpeg);
            }
        } catch (Exception ignored) {
            // skip frame on error
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // no drawing
    }

    /** Lấy JPEG mới nhất (có thể null). Gọi từ UdpLogger. */
    public byte[] getLastJpeg() {
        return lastJpeg.getAndSet(null);
    }
}
