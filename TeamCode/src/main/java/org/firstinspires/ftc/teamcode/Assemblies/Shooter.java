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
    public Servo pusher;
    public Servo tilter;
    double currentTargetVelocity = 0;
    double FLYWHEEL_MAX_VELOCITY = 2620;
    double POWERSHOT_VELOCITY = FLYWHEEL_MAX_VELOCITY*0.9;
    double HIGH_GOAL_VELOCITY = FLYWHEEL_MAX_VELOCITY*0.95;
    double POWERSHOT_POSITION = 0.463888; //0.46; before more ring compression
    double HIGH_GOAL_POSITION = 0.479;
    public double FAR_SHOT_VELOCITY = FLYWHEEL_MAX_VELOCITY*.95;
    public double AUTO_POS_1 = 0.4725;
    public double AUTO_POS_2 = 0.4730;
    public double AUTO_POS_3 = HIGH_GOAL_POSITION;
    double PUSHER_ALL_OUT = .85;
    double LAUNCH_POSITION = 0.7;
    double RELOAD_POSITION = 0.50;
    public boolean motorRunning = false;
    public boolean launching = false;

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
        motorRunning = false;
        pusher.setPosition(RELOAD_POSITION);
        aimAt(ShooterTarget.POWERSHOT);
        aimAt(ShooterTarget.HIGH_GOAL); // force reset to known position
    }

    public void shooterTelemetry() {
        teamUtil.telemetry.addLine("Shooter Tilt:"+ tilter.getPosition() + "Shooter Pusher" + pusher.getPosition() + " Flywheel:"+flywheel.getVelocity());
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

    public void manualTilt(double tilt) {
        tilter.setPosition(tilt);
    }

    public void manualFlyWheelSpeed(double velocity) {
        currentTargetVelocity = velocity;
        flywheel.setVelocity(currentTargetVelocity);
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


    //NOT USED
//    public void togglePusherAllOut(){
//        if(pusher.getPosition() != PUSHER_ALL_OUT){
//            pusher.setPosition(PUSHER_ALL_OUT);
//        } else {
//            pusher.setPosition(RELOAD_POSITION);
//        }
//    }

    // StartFlyWheel at the speed needed for the current aim
    public void flywheelStart() {
        teamUtil.log("fly wheel start");
        if (motorRunning == false){
            teamUtil.log("fly wheel starting: " + currentTargetVelocity);
            flywheel.setVelocity(currentTargetVelocity);
            motorRunning = true;
        }
    }

    // Returns true if the flywheel is spinning fast enough for the current aim
    public boolean flywheelReady() {
        if (Math.abs(flywheel.getVelocity() - currentTargetVelocity)<currentTargetVelocity*0.05){
            teamUtil.log("Flywheel is ready, thing: " + Math.abs(flywheel.getVelocity() - currentTargetVelocity) );
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
        teamUtil.log("LAUNCH Flywheel:" + flywheel.getVelocity() + "Shooter Tilt:"+ tilter.getPosition());

        pusher.setPosition(LAUNCH_POSITION);
        teamUtil.pause(350);
        pusher.setPosition(RELOAD_POSITION);
        teamUtil.pause(350);
    }
    public void launchAndClear() {
        teamUtil.log("LAUNCH Flywheel:" + flywheel.getVelocity() + "Shooter Tilt:"+ tilter.getPosition());
        pusher.setPosition(PUSHER_ALL_OUT);
        teamUtil.pause(500);
        pusher.setPosition(RELOAD_POSITION);
        teamUtil.pause(500);
    }
    public void pusherManualControl(float triggerPosition) {
        double slope = (PUSHER_ALL_OUT - RELOAD_POSITION) ;
        pusher.setPosition(RELOAD_POSITION + (triggerPosition * slope));
    }

    public void launchNoWait() {
        if (launching) {
            return;
        }
        launching = true;
        Thread thread = new Thread(new Runnable() {
            public void run() {
                launch();
                launching = false;
            }
        });
        thread.start();
    }

}
