package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Blocker {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    CRServo driveServo;

    double FOWARD_FULL_POWER = 1;
    double STOP = 0;
    double BACKWARDS_FULL_POWER = -1;
    long EXTEND_TIME = 3000;
    long RETRACT_TIME = 3000;
    boolean moving = false;

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
        driveServo.setPower(FOWARD_FULL_POWER);
    }

    // Starts the blocker moving in.  Will continue until stop is called
    void retract() {
        driveServo.setPower(BACKWARDS_FULL_POWER);
    }

    // Stop the blocker from moving
    void stop() {
        driveServo.setPower(STOP);
    }
    void extendFully () {
        driveServo.setPower(FOWARD_FULL_POWER);
        teamUtil.pause(EXTEND_TIME);
        driveServo.setPower(STOP);
    }
    // Launches a new thread to extend the servo fully
    void extendNoWait () {
        if (moving) {
            teamUtil.log("called extendNotWait while moving blocker mechanism");
            return;
        }
        moving = true;
        Thread thread = new Thread(new Runnable() {
            public void run() {
                extendFully();
                moving = false;
            }
        });
        thread.start();

    }
    void retractFully () {
        driveServo.setPower(BACKWARDS_FULL_POWER);
        teamUtil.pause(RETRACT_TIME);
        driveServo.setPower(STOP);
    }

    // Launches a new thread to retract the servo fully
    void retractNoWait () {
        if (moving) {
            teamUtil.log("called retractNotWait while moving blocker mechanism");
            return;
        }
        Thread thread = new Thread(new Runnable() {
            public void run() {
                retractFully();
                moving = false;
            }
        });
        thread.start();
    }
}
