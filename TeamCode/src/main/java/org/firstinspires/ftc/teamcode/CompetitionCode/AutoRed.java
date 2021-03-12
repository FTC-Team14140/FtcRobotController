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
        telemetry.addData("rings: ", detector.detectRings());
        teamUtil.telemetry.update();
        waitForStart();
        int rings = detector.detectRings();

        //roll forward to detect rings
        long DETECT_TIME_OUT = 3000; // Allow up to 3 seconds for TensorFlow to figure it out
        robot.drive.moveInches(180, 12, 6000);
        rings = detector.detectRings();
        teamUtil.pause(3000);
        long timeOutTime = System.currentTimeMillis() + DETECT_TIME_OUT;
        // Assume NO False positives...

        while (rings < 1 && System.currentTimeMillis() < timeOutTime) {
            teamUtil.pause(100);
            rings = detector.detectRings();
        }
        telemetry.addData("rings: ", rings);
        telemetry.update();
        teamUtil.log("Found "+ rings + " rings in msecs:"+(DETECT_TIME_OUT - (timeOutTime -System.currentTimeMillis())));
        detector.shutDownDetector();

        robot.doAuto2(rings);
    }
}
