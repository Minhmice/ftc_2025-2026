package org.firstinspires.ftc.teamcode;

/**
 * Điều phối chuỗi bắn: Triangle -> pre_shoot -> ready_to_kick -> request_kick -> post_shoot.
 * Giữ state inshoot, kickStarted, lastTriangle; gọi Shooter.update() mỗi vòng lặp.
 */
public class ShootCoordinator {

    private final ArtifactProcessing artifactProcessing;
    private final Shooter shooter;

    private boolean inshoot = false;
    private boolean lastTriangle = false;
    private boolean kickStarted = false;

    public ShootCoordinator(ArtifactProcessing artifactProcessing, Shooter shooter) {
        this.artifactProcessing = artifactProcessing;
        this.shooter = shooter;
    }

    /**
     * Gọi mỗi vòng lặp TeleOp với trạng thái nút Triangle.
     * Xử lý edge-trigger Triangle, request_kick khi ready, post_shoot khi kick xong.
     */
    public void update(boolean trianglePressed) {
        if (trianglePressed && !lastTriangle) {
            inshoot = true;
            kickStarted = false;
            artifactProcessing.start_pre_shoot();
        }
        lastTriangle = trianglePressed;

        if (inshoot) {
            if (artifactProcessing.is_ready_to_kick()) {
                shooter.request_kick();
            }
            if (shooter.isKicking()) {
                kickStarted = true;
            }
            if (kickStarted && !shooter.isKicking()) {
                artifactProcessing.request_post_shoot();
                inshoot = false;
                kickStarted = false;
            }
        }

        shooter.update();
    }

    /** True khi đang trong chuỗi bắn (từ Triangle đến khi post_shoot xong). */
    public boolean isInShoot() {
        return inshoot;
    }

    /** Trạng thái hiển thị: READY / PREP / IDLE. */
    public String getShootStateLabel(boolean readyToKick) {
        if (readyToKick) return "READY";
        return inshoot ? "PREP" : "IDLE";
    }
}
