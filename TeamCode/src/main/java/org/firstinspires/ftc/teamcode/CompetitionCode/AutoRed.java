package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name="Red", group ="Competition")
public class AutoRed extends LinearOpMode {
    TeamGamepad teamGamePad;
    Robot robot;

    public void initialize() {

        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");
        teamUtil.telemetry.update();
        teamGamePad = new TeamGamepad(this);
        robot = new Robot(this);
        robot.init(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        teamUtil.telemetry.addLine("Ready to Ultimate");
        teamUtil.telemetry.update();
        waitForStart();
        robot.doAuto();
    }
}
