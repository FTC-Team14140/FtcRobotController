package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Blocker {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    CRServo driveServo;

    void Blocker() {
        teamUtil.log("Constructing Blocker");

        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init() {
        teamUtil.log("Initializing Blocker");
        driveServo = hardwareMap.crservo.get("driveServo");
    }

    // Starts the blocker moving out.  Will continue until stop is called
    void extend() {

    }

    // Starts the blocker moving in.  Will continue until stop is called
    void retract() {

    }

    // Stop the blocker from moving
    void stop() {

    }

    // Launches a new thread to extend the servo fully
    void extendNoWait () {

    }

    // Launches a new thread to retract the servo fully
    void retractNoWait () {

    }
}
