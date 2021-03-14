package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
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
        // Start looking at Rings before the start...
        detector.activateDetector();
        telemetry.addLine("Starting to Detect");
        telemetry.update();

        // use the ring detector to detect and vote 4 times a second
        while (!opModeIsActive()) {
            teamUtil.pause(250);
            detector.detectAndVote();
            int[] totals = detector.getTotals();
            //teamUtil.log("1:"+ totals[1]+ " 2:"+ totals[2]+ " 3:"+ totals[3]);
            telemetry.addData("Path: ", detector.getPath());
            telemetry.update();
        }

        waitForStart();
        int path = detector.getPath(); // Result of voting: 1 2 or 3
        teamUtil.log("Path: "+ path);
        detector.shutDownDetector();

        robot.doAutoV3(path);
    }
}
