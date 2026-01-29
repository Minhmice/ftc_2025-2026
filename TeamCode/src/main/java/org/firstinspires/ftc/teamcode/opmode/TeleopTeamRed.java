package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main;

@TeleOp(name = "TELEOP RED TEAM")
public class TeleopTeamRed extends Main {

    @Override
    public void runOpMode() throws InterruptedException {
        this.team_color = 1;
        super.runOpMode();
    }
}
