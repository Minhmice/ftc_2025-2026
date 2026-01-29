package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main;

@TeleOp(name = "TELEOP BLUE TEAM")
public class TeleopTeamBlue extends Main {

    @Override
    public void runOpMode() throws InterruptedException {
        this.team_color = 2;
        super.runOpMode();
    }
}
