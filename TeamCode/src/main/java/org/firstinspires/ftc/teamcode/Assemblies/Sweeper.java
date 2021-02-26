package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    Double TENSION_POWER = -0.3;
    int FULLY_EXTENDED = 1800;
    int EXTENDED_THESHOLD = (int) (FULLY_EXTENDED * .95);
    Double EXTEND_SPEED = 2000.0; //TODO: find right number
    Double RETRACT_SPEED = -2000.0; //TODO: find right number
    int FULLY_RETRACTED = 0;
    int RETRACTED_DOWN_THESHOLD = FULLY_RETRACTED + 200;
    int RETRACTED_UP_THESHOLD = FULLY_RETRACTED + 40;

    public final float SWEEP = 0.2f;
    public final float STOWED = 0.86f;
    public final float READY = (STOWED-SWEEP)/2+SWEEP; // half way between the two
    public boolean motorRunning = false;


    public Sweeper() {
        teamUtil.log("Constructing Sweeper");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init() {
        teamUtil.log("Initializing Sweeper");
        motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        sweeper = hardwareMap.servo.get("sweeperServo");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void sweeperTelemetry() {
        String s = "UNKNOWN";
        if (sweeper.getPosition() == STOWED)
            s = "STOWED";
        else if (sweeper.getPosition() == READY)
            s = "READY";
        else if (sweeper.getPosition() == SWEEP)
            s = "SWEEP";
        teamUtil.telemetry.addLine("Sweeper Arm:"+ motor.getCurrentPosition() + " Grabber:"+s+ " Grabber:"+sweeper.getPosition());
    }

    // retracts the arm fully and resets the encoder position to 0
    public void reset() {
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
                // reset stalled position as Zero
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setPower(0);

                // Move the motor out a bit so it won't interfere with the intake
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setVelocity(EXTEND_SPEED);
                motor.setTargetPosition(RETRACTED_UP_THESHOLD);
                while (motor.isBusy()) {}

                // Get state ready to run
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setPower(0);
                sweeperCalibrated = true;
                teamUtil.log("Calibrating Sweeper - Finished");
                return;
            }
            teamUtil.log("Sweeper Encoder:" + " " + last);

        } while (true);
    }

    public int getRetractedTarget() {
        if (sweeper.getPosition() >= STOWED) {
            return RETRACTED_UP_THESHOLD;
        } else {
            return RETRACTED_DOWN_THESHOLD;
        }
    }

    // Use RunToPosition and setVelocity to extend/retract the sweeper
    public void extend2(float speed) {
        if (motor.getCurrentPosition() >= EXTENDED_THESHOLD) {
            stop();
            motorRunning = false;
        } else {
            motorRunning = true;
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(EXTENDED_THESHOLD);
            motor.setVelocity(EXTEND_SPEED * speed);
        }
    }

    public void retract2(float speed) {
        int threshold = getRetractedTarget();
        if (motor.getCurrentPosition() <= threshold) {
            stop();
            motorRunning = false;
        } else {
            motorRunning = true;
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(threshold);
            motor.setVelocity(EXTEND_SPEED * speed);
        }
    }

        // Extends the sweeper arm at full speed until stop is called or it hits the end
    public void extend() {
        if (motor.getCurrentPosition() > EXTENDED_THESHOLD) {
            stop();
            motorRunning = false;
        } else {
            motorRunning = true;
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(EXTEND_SPEED);
        }
    }


    // retract the sweeper arm at full speed until stop is called or it cannot go further
    public void retract() {
        if (sweeper.getPosition() >= STOWED) {
            if (motor.getCurrentPosition() < RETRACTED_UP_THESHOLD+30) {
                stop();
            } else {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocity(RETRACT_SPEED);
            }
        } else {
            if (motor.getCurrentPosition() < RETRACTED_DOWN_THESHOLD+30) {
                stop();
            } else  {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocity(RETRACT_SPEED);
            }
        }
    }

    // Extends the sweeper arm to full extension
    // TODO: Test
    public void extendFully() {
        teamUtil.log("sweeper is extending fully to pos: " + FULLY_EXTENDED);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(EXTEND_SPEED);
        motor.setTargetPosition(FULLY_EXTENDED);
    }

    // Starts the blocker moving in and full speed.  Will continue until stop is called
    // TODO: Test
    public void retractFully() {
        // set target based on servo position
        int target = sweeper.getPosition() >= STOWED ? RETRACTED_UP_THESHOLD : RETRACTED_DOWN_THESHOLD;
        teamUtil.log("sweeper is retracting fully to pos: " + target);

        // start the motors moving to the target
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(RETRACT_SPEED);
        motor.setTargetPosition(target);
    }

    // Stop the blocker at its current position
    public void stop() {
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
    // 1 means SWEEP Position
    // -1 means STOWED position
    // This is intended to be hooked up to a gamepad joystick for manual control over the sweeper position
//    public void manualControl(float position) {
//        if (position > .1) {// allow for a little dead zone
//            double slope = (SWEEP - READY) / .9;
//            sweeper.setPosition(READY + (position * slope));
//        } else if (position < -.1) {
//            double slope = (READY - STOWED) / .9;
//            sweeper.setPosition(READY + (position * slope));
//        }
//    }
    public void manualControl(float triggerPosition) {
            double slope = (SWEEP - READY) ;
            sweeper.setPosition(READY + (triggerPosition * slope));
    }

    // Launches a new thread to retract and stow the sweeper
    void retractAndStowNoWait () {
        moveToStow();
        retractFully();
    }
}
