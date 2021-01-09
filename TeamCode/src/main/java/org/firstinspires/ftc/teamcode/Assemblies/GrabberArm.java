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

    final double STALL_POWER = .098; //idk if this is the right number
    boolean isReset = false;

    enum GRABBER_POS {
        OPEN,
        CLOSE
    }

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
        grab();
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(STALL_POWER);
        teamUtil.pause(500); // let them get going
        do {
            long last = arm.getCurrentPosition();
            teamUtil.sleep(250);
            // if things aren't moving, we have stalled
            if (arm.getCurrentPosition() == last) {
                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(0);
                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                isReset = true;
                teamUtil.log("Calibrating Grabber - Finished");

                return;
            }
            teamUtil.log("Grabber Encoder:" + " " + last);

        } while (true);
    }

    // Starts the blocker moving out at full speed.  Will continue until stop is called
    void extend() {
        arm.setTargetPosition(FULLY_EXTENDED);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(EXTEND_SPEED);
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
