package org.firstinspires.ftc.teamcode.Assemblies;

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
    }

    // Tilt the shooter to aim at the powershots
    // If the flywheel is running, this will change the speed as needed
    void aimAtPowerShot () {

    }

    // Tilt the shooter to aim at the highgoal
    // If the flywheel is running, this will change the speed as needed
    void aimAtHighGoal () {

    }

    // StartFlyWheel at the speed needed for the current aim
    void startFlyWheel () {

    }

    // Returns true if the flywheel is spinning fast enough for the current aim
    boolean flywheelReady () {
        return true;
    }

    // cut power to the flywheel and let it coast to a stop
    void stopFlywheel() {

    }

    // Launch a ring
    void launch() {

    }
}
