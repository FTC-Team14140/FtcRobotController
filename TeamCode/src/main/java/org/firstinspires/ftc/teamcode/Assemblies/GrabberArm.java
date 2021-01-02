package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class GrabberArm {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx arm;
    Servo grabber;

    void GrabberArm() {
        teamUtil.log("Constructing GrabberArm");

        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init() {
        teamUtil.log("Initializing GrabberArm");
        arm = hardwareMap.get(DcMotorEx.class, "armMotor");
        grabber = hardwareMap.servo.get("grabberServo");
    }

    // Moves the arm to the start position (straight up)
    // and resets the encoder position to 0
    void reset() {
        teamUtil.log("Resetting GrabberArm");

    }

    // move the servo to the grab position
    // This is also the stowed position
    void grab() {

    }

    // move the servo to the release position
    // This is also the ready position
    void release() {

    }

    // Launches a new thread to move the arm to its ready position with the grabber open and ready
    void moveToReadyNoWait() {

    }

    // Moves the arm to the place/ready position and releases the wobble goal.
    void placeAndRelease() {

    }

    // Launches a new thread to Grab a wobble goal and lift it up for transport
    void grabAndLiftNoWait () {

    }

    // stows the grabber and arm inside the robot
    void stow() {

    }

    // Launches a new thread to stow the grabber and arm inside the robot
    void stowNoWait() {

    }
}
