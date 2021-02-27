package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.RingDetector;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name="Red", group ="Competition")
public class AutoRed extends LinearOpMode {
    TeamGamepad teamGamePad;
    Robot robot;
    RingDetector detector;

    public void initialize() {

        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");
        teamUtil.telemetry.update();
        detector = new RingDetector(telemetry, hardwareMap);
        detector.initDetector();

        teamGamePad = new TeamGamepad(this);
        robot = new Robot(this);
        robot.init(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        detector.activateDetector();
        telemetry.addLine("Starting to detect rings...");

        teamUtil.telemetry.addLine("Ready to Ultimate");
        teamUtil.telemetry.update();
        waitForStart();

        //roll forward to detect rings
        robot.drive.moveInches(180, 6, 6000);

        int rings = detector.detectRings();
        telemetry.addData("rings: ", rings);
        telemetry.update();
        teamUtil.log("rings: "+ rings);
        detector.shutDownDetector();

        robot.doAuto2(1);
    }
}
