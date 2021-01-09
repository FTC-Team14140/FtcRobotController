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

    //defining new type with specific values
    public enum ShooterTarget {
        POWERSHOT,
        HIGH_GOAL
    }

    ShooterTarget currentTarget;


    void Shooter() {
        teamUtil.log("Constructing Shooter");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init() {
        teamUtil.log("Initializing Shooter");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        pusher = hardwareMap.servo.get("grabberServo");
        tilter = hardwareMap.servo.get("tilterServo");
        currentTarget = ShooterTarget.POWERSHOT;
    }

    // Tilt the shooter to aim at the powershots
    // If the flywheel is running, this will change the speed as needed
    void aimAt(ShooterTarget newTarget) {
        if (newTarget == currentTarget) {
            return;
        }else {
            currentTarget = newTarget;
            changeSpeed();
            changeAngle();
        }

    }

    void changeSpeed() {

    }

    void changeAngle() {

    }

    // StartFlyWheel at the speed needed for the current aim
    void startFlyWheel() {

    }

    // Returns true if the flywheel is spinning fast enough for the current aim
    boolean flywheelReady() {
        return true;
    }

    // cut power to the flywheel and let it coast to a stop
    void stopFlywheel() {

    }

    // Launch a ring
    void launch() {

    }
}
