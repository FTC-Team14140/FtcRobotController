package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Shooter {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx flywheel;
    Servo pusher;
    Servo tilter;
    double currentTargetVelocity = 0;
    double POWERSHOT_VELOCITY = 1.0; //TODO: find right number
    double HIGH_GOAL_VELOCITY = 2.0; //TODO: find right number
    double POWERSHOT_POSITION = 0; //TODO: find right number
    double HIGH_GOAL_POSITION = 1; //TODO: find right number
    double LAUNCH_POSITION = 0; //TODO: find right number
    double RELOAD_POSITION = 0; //TODO: find right number
    public boolean motorRunning = false;

    //defining new type with specific values
    public enum ShooterTarget {
        POWERSHOT,
        HIGH_GOAL
    }
    public enum FlyWheel {
        ON,
        OFF
    }
    ShooterTarget currentTarget;
    FlyWheel flywheelRunning;

    public Shooter() {
        teamUtil.log("Constructing Shooter");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init() {
        teamUtil.log("Initializing Shooter");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        pusher = hardwareMap.servo.get("pusherServo");
        tilter = hardwareMap.servo.get("tilterServo");
        currentTarget = ShooterTarget.POWERSHOT;
    }

    // Tilt the shooter to aim at the powershots
    // If the flywheel is running, this will change the speed as needed
    public void aimAt(ShooterTarget newTarget) {
        if (newTarget == currentTarget) {
            return;
        }else {
            currentTarget = newTarget;
            changeSpeed();
            changeAngle();
        }

    }

    public void changeSpeed() {
        if (currentTarget == ShooterTarget.POWERSHOT) {
            currentTargetVelocity = POWERSHOT_VELOCITY;
        }else{
            currentTargetVelocity = HIGH_GOAL_VELOCITY;
        }
        if (motorRunning){
            flywheel.setVelocity(currentTargetVelocity);
        }
    }

    void changeAngle() {
        if (currentTarget == ShooterTarget.POWERSHOT) {
            tilter.setPosition(POWERSHOT_POSITION);

        }else {
            tilter.setPosition(HIGH_GOAL_POSITION);
        }
    }

    // StartFlyWheel at the speed needed for the current aim
    public void flywheelStart() {
        if (motorRunning = false){
            flywheel.setVelocity(currentTargetVelocity);
            motorRunning = true;
        }
    }

    // Returns true if the flywheel is spinning fast enough for the current aim
    public boolean flywheelReady() {
        if (Math.abs(flywheel.getVelocity() - currentTargetVelocity)<currentTargetVelocity*0.05){
            return true;
        }else{
            return false;
        }
    }

    // cut power to the flywheel and let it coast to a stop
    public void stopFlywheel() {
        flywheel.setVelocity(0);
        motorRunning = false;
    }

    // Launch a ring
    public void launch() {
        pusher.setPosition(LAUNCH_POSITION);
        teamUtil.pause(500);
        pusher.setPosition(RELOAD_POSITION);
    }
}
