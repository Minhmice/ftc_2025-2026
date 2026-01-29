package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public class TagProcessing {
    private final RobotHardware robot;
    private boolean detected;
    private double bearing;
    private double range;
    private double distanceToCenter = -1;

    // VisionPortal defaults to 640x480; half-width = 320px
    private static final double IMAGE_HALF_WIDTH_PX = 320.0;

    /** AprilTag ID cho đội đỏ (team_color == 1). */
    private static final int TAG_ID_RED = 24;
    /** AprilTag ID cho đội xanh (team_color == 2). */
    private static final int TAG_ID_BLUE = 20;

    public TagProcessing(RobotHardware robot) {
        this.robot = robot;
    }

    /** team_color: 1 = red, 2 = blue. */
    public void update(int team_color) {
        this.detected = false;

        List<AprilTagDetection> tags = robot.april_tag.getDetections();
        if (tags != null && !tags.isEmpty()) {
            int targetId = (team_color == 1) ? TAG_ID_RED : TAG_ID_BLUE;

            for (AprilTagDetection tag : tags) {
                if (tag.id == targetId) {
                    // Pose can be null if the tag is detected but pose cannot be solved
                    if (tag.ftcPose != null) {
                        this.detected = true;
                        this.bearing = tag.ftcPose.bearing;
                        this.range = tag.ftcPose.range;     // inches (FTC docs)
                        this.distanceToCenter = tag.center.x;
                    }
                    break;
                }
            }
        }
    }

    public boolean isDetected() { return detected; }
    public double getBearing() { return bearing; }

    /** Range from camera to tag center (inches). */
    public double getRangeInches() { return range; }

    /** Pixel offset from image center (px). */
    public double getDistanceToCenter() {
        return distanceToCenter - IMAGE_HALF_WIDTH_PX;
    }
}
