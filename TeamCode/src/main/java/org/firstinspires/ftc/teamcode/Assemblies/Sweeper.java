package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Sweeper {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx motor;
    public Servo sweeper;
    Boolean sweeperCalibrated = false;
    Double TENSION_POWER = 0.098;
    int FULLY_EXTENDED = 1000; //TODO: find right number for extended encoder position
    int EXTENDED_THESHOLD = (int) (FULLY_EXTENDED * .95);
    Double EXTEND_SPEED = 1000.0; //TODO: find right number
    int FULLY_RETRACTED = 1000; //TODO: find right number for retracted encoder position
    int RETRACTED_DOWN_THESHOLD = 1000;
    int RETRACTED_UP_THESHOLD = FULLY_RETRACTED + 5;

    Double RETRACT_SPEED = 1000.0; //TODO: find right number
    final float SWEEP = 0.1f; //TODO: find right number
    final float STOWED = 0.1f; //TODO: find right number
    final float READY = 0.1f; //TODO: find right number



    public Sweeper() {
        teamUtil.log("Constructing Sweeper");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init() {
        teamUtil.log("Initializing Sweeper");
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        sweeper = hardwareMap.servo.get("sweeperServo");
    }

    public void sweeperTelemetry() {
        teamUtil.telemetry.addLine("Sweeper Arm:"+ motor.getCurrentPosition() + " Grabber:"+sweeper.getPosition());
    }

    // retracts the arm fully and resets the encoder position to 0
    void reset() {
        teamUtil.log("Resetting Sweeper");
        moveToStow();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(TENSION_POWER);
        teamUtil.pause(500); // let them get going
        do {
            long last = motor.getCurrentPosition();
            teamUtil.sleep(250);
            // if things aren't moving, we have stalled
            if (motor.getCurrentPosition() == last) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sweeperCalibrated = true;
                teamUtil.log("Calibrating Sweeper - Finished");

                return;
            }
            teamUtil.log("Sweeper Encoder:" + " " + last);

        } while (true);
    }

    // Extends the sweeper arm at full speed until stop is called or it hits the end
    void extend() {
        if (motor.getCurrentPosition() > EXTENDED_THESHOLD) {
            stop();
        } else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(EXTEND_SPEED);
        }
    }

    // retract the sweeper arm at full speed until stop is called or it cannot go further
    void retract() {
        if (sweeper.getPosition() <= READY && motor.getCurrentPosition() < RETRACTED_UP_THESHOLD) {
            stop();
        } else if (sweeper.getPosition() > READY && motor.getCurrentPosition() < RETRACTED_DOWN_THESHOLD) {
            stop();
        } else  {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(RETRACT_SPEED);
        }
    }

    // Extends the sweeper arm to full extension
    void extendFully() {
        motor.setTargetPosition(FULLY_EXTENDED);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(EXTEND_SPEED);
    }

    // Starts the blocker moving in and full speed.  Will continue until stop is called
    void retractFully() {
        motor.setTargetPosition(FULLY_RETRACTED);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(RETRACT_SPEED);
    }

    // Stop the blocker at its current position
    void stop() {
        motor.setVelocity(0);
    }

    // Move the sweeper servo to the stow position
    void moveToStow() {
        sweeper.setPosition(STOWED);
    }

    // Move the sweeper servo to the ready position (run to position) (high enough to not block rings, but as close as possible.
    void moveToReady() {
        sweeper.setPosition(READY);
    }

    // Move the sweeper servo to the sweep position. (run to position)
    void moveToSweep() {
        sweeper.setPosition(SWEEP);
    }

    // Move the sweeper to the specified position.
    // 0 means READY Position
    // 1 means SWEEP position
    // This is intended to be hooked up to a gamepad control for manual control over the sweeper position
    void manualControl(float position) {
        float controlledPosition = (SWEEP-READY)*position+READY;
        sweeper.setPosition(controlledPosition);
    }

    // Launches a new thread to retract and stow the sweeper
    void retractAndStowNoWait () {
        moveToStow();
        retractFully();
    }
}
