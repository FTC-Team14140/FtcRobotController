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
    final int HORIZONTAL = 1; // TODO: find correct number
    final int STOW_POS = 1; // TODO: find correct number
    final int READY_POS = 1; // TODO: find correct number
    final int TRANSPORT_POS = 1; // TODO: find correct number
    final double ARM_SPEED = .50; // TODO: find correct number
    final float GRABBER_GRAB = .50f; // TODO: find correct number
    final float GRABBER_OPEN = .50f; // TODO: find correct number
    boolean isReset = false;

    enum GRABBER_POS {
        OPEN,
        CLOSE
    }

    GRABBER_POS currentGrabberPosition;

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

    // move the servo to the grab positionu
    // This is also the stowed position
    void grab() {
        grabber.setPosition(GRABBER_GRAB);
        currentGrabberPosition = GRABBER_POS.CLOSE;
    }

    // move the servo to the release position
    // This is also the ready position
    void release() {
        grabber.setPosition(GRABBER_OPEN);
        currentGrabberPosition = GRABBER_POS.OPEN;
    }

    // Launches a new thread to move the arm to its ready position with the grabber open and ready
    void moveToReadyNoWait() {
        placeAndRelease();
    }

    // Moves the arm to the place/ready position and releases the wobble goal.
    void placeAndRelease() {
        arm.setTargetPosition(READY_POS);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_SPEED);
        while (Math.abs(arm.getCurrentPosition()-READY_POS)>50){
        }
        release();
    }

    // Launches a new thread to Grab a wobble goal and lift it up for transport
    void grabAndLiftNoWait () {
        grab();
        teamUtil.pause(1000);
        arm.setTargetPosition(TRANSPORT_POS);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_SPEED);
    }

    // stows the grabber and arm inside the robot
    void stow() {
        grab();
        teamUtil.pause(1000);
        arm.setTargetPosition(STOW_POS);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_SPEED);
    }

    // Launches a new thread to stow the grabber and arm inside the robot
    void stowNoWait() {

    }
}
