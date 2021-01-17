package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.RingDetector;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.runningVoteCount;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "testRingDetector")

public class TestRingDetector extends LinearOpMode {

    RingDetector detector;
    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.init(this);


        teamUtil.log("starting OpMode");


        detector = new RingDetector(telemetry, hardwareMap);
        detector.initDetector();

        telemetry.addLine("initialized");
        telemetry.update();


        // Start looking at Skystones before the start...
        detector.activateDetector();
        telemetry.addLine("Starting to Detect");
        telemetry.update();



        waitForStart();

        while(opModeIsActive()){

            detector.reportRingInformation();
            telemetry.addData("path: ", detector.detectRings());

            telemetry.update();


        }

        detector.shutDownDetector();
    }
}
