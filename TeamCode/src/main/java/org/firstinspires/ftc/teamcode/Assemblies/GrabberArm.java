package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class GrabberArm {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotorEx arm;
    public Servo grabber;

    final double STALL_POWER = -0.098; //idk if this is the right number UP is Negative
    final int HORIZONTAL = 0;
    final int STOW_POS = 3800;
    final int READY_POS = 2575;
    final int TRANSPORT_POS = READY_POS / 2; // Was 600
    final int AUTO_DROP = READY_POS - 250;
    final double ARM_SPEED = 1500;
    final float GRABBER_GRAB = .598f;
    final float GRABBER_OPEN = 1f;
    boolean isReset = false;
    boolean isBusy = false;

    public enum GRABBER_POS {
        OPEN,
        CLOSE
    }

    public GRABBER_POS currentGrabberPosition;




    public GrabberArm() {
        teamUtil.log("Constructing GrabberArm");

        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    public void init() {
        teamUtil.log("Initializing GrabberArm");
        arm = hardwareMap.get(DcMotorEx.class, "armMotor");
        grabber = hardwareMap.servo.get("grabberServo");
        currentGrabberPosition = GRABBER_POS.OPEN;
    }

    public void armTelemetry() {
        teamUtil.telemetry.addLine("Grabber Arm:"+ arm.getCurrentPosition() + " Grabber:"+grabber.getPosition()+ " Enum:"+currentGrabberPosition);
    }

    // Moves the arm to the start position (straight up)
    // and resets the encoder position to 0
    public void reset() {
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
    public void grab() {


        grabber.setPosition(GRABBER_GRAB);
        currentGrabberPosition = GRABBER_POS.CLOSE;
    }

    // move the servo to the release position
    // This is also the ready position
    public void release() {
        //grabber arm was in stow, should not open the grabber
        if(arm.getCurrentPosition() > (READY_POS + STOW_POS)/2 ){
            return;
        }
        grabber.setPosition(GRABBER_OPEN);
        currentGrabberPosition = GRABBER_POS.OPEN;
    }

    // Launches a new thread to move the arm to its ready position with the grabber open and ready
    public void moveToReadyNoWait() {
        if (isBusy) {
            teamUtil.log("called moveToReadyNoWait while grabber busy");
            return;
        }
        isBusy = true;
        Thread thread = new Thread(new Runnable() {
            public void run() {
                placeAndRelease();
                isBusy = false;
            }
        });
        thread.start();
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
    void grabAndLift () {
        grab();
        teamUtil.pause(1000);
        arm.setTargetPosition(TRANSPORT_POS);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_SPEED);
        while (Math.abs(arm.getCurrentPosition()-TRANSPORT_POS)>50){
        }
    }
    void lift () {
        teamUtil.pause(1000);
        arm.setTargetPosition(TRANSPORT_POS);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_SPEED);
        while (Math.abs(arm.getCurrentPosition()-TRANSPORT_POS)>50){
        }
    }

    void liftToAutoDrop () {
        teamUtil.pause(1000);
        arm.setTargetPosition(AUTO_DROP);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_SPEED);
        while (Math.abs(arm.getCurrentPosition()-AUTO_DROP)>50){
        }
    }

    // Launches a new thread to Grab a wobble goal and lift it up for transport
    public void grabAndLiftNoWait () {
        if (isBusy) {
            teamUtil.log("called grabAndLiftNoWait while grabber busy");
            return;
        }
        isBusy = true;
        Thread thread = new Thread(new Runnable() {
            public void run() {
                grabAndLift();
                isBusy = false;
            }
        });
        thread.start();
    }

    public void liftNoWait () {
        if (isBusy) {
            teamUtil.log("called LiftNoWait while grabber busy");
            return;
        }
        isBusy = true;
        Thread thread = new Thread(new Runnable() {
            public void run() {
                lift();
                isBusy = false;
            }
        });
        thread.start();
    }


    public void liftToAutoDropNoWait () {
        if (isBusy) {
            teamUtil.log("called LiftNoWait while grabber busy");
            return;
        }
        isBusy = true;
        Thread thread = new Thread(new Runnable() {
            public void run() {
                liftToAutoDrop();
                isBusy = false;
            }
        });
        thread.start();
    }
    // stows the grabber and arm inside the robot
    void stow() {
        grab();
        teamUtil.pause(1000);
        arm.setTargetPosition(STOW_POS);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_SPEED);
        while (Math.abs(arm.getCurrentPosition()-STOW_POS)>50){
        }
    }

    // Launches a new thread to stow the grabber and arm inside the robot
    public void stowNoWait() {
        if (isBusy) {
            teamUtil.log("called stowNoWait while grabber busy");
            return;
        }
        isBusy = true;
        Thread thread = new Thread(new Runnable() {
            public void run() {
                stow();
                isBusy = false;
            }
        });
        thread.start();
    }
}
