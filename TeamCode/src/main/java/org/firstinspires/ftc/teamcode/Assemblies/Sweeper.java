package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Sweeper {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx motor;
    Servo sweeper;

    void Sweeper() {
        teamUtil.log("Constructing Sweeper");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init() {
        teamUtil.log("Initializing Sweeper");
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        sweeper = hardwareMap.servo.get("sweeperServo");
    }

    // retracts the arm fully and resets the encoder position to 0
    void reset() {
        teamUtil.log("Resetting Sweeper");

    }

    // Starts the blocker moving out at full speed.  Will continue until stop is called
    void extend() {

    }

    // Starts the blocker moving in and full speed.  Will continue until stop is called
    void retract() {

    }

    // Stop the blocker at its current position
    void stop() {

    }

    // Move the sweeper servo to the stow position
    void moveToStow() {

    }

    // Move the sweeper servo to the ready position (high enough to not block rings, but as close as possible.
    void moveToReady() {

    }

    // Move the sweeper servo to the sweep position.
    void moveToSweep() {

    }

    // Move the sweeper to the specified position.
    // 0 means straight up
    // 1 means straight down
    // This is intended to be hooked up to a gamepad control for manual control over the sweeper position
    void manualControl(float position) {

    }

    // Launches a new thread to retract and stow the sweeper
    void retractAndStowNoWait () {

    }

}
