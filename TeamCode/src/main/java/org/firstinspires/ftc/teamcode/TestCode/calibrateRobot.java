package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.Shooter;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "Calibrate Robot")

public class calibrateRobot extends LinearOpMode {
    Robot robot;
    TeamGamepad teamGamePad;
    DcMotor intakeMotor;



    public void initialize() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        robot = new Robot(this);
        teamGamePad = new TeamGamepad(this);
        robot.init(true);
        teamUtil.initPerf();
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void findServoPosition (Servo servo) {
        double MAJOR_INCREMENT = 0.05;
        double MINOR_INCREMENT = 0.01;

        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP) && (servo.getPosition()<1)) {
            servo.setPosition(servo.getPosition() + MAJOR_INCREMENT);
        }
        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN) && (servo.getPosition()>0)) {
            servo.setPosition(servo.getPosition() - MAJOR_INCREMENT);
        }
        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT) && (servo.getPosition()<1)) {
            servo.setPosition(servo.getPosition() + MINOR_INCREMENT);
        }
        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT) && (servo.getPosition()>0)) {
            servo.setPosition(servo.getPosition() - MINOR_INCREMENT);
        }
    }

    @Override
    public void runOpMode() {
        float iPower = 0.1f;
        initialize();
        waitForStart();
        robot.drive.resetHeading();
        //robot.grabber.grabber.setPosition(0.75);
        //robot.shooter.tilter.setPosition(0.5);
        //robot.shooter.pusher.setPosition(0.5);
        //robot.sweeper.sweeper.setPosition(0.5);

        while (opModeIsActive()) {
            teamGamePad.gamepadLoop();

            if (gamepad2.a) {
                findServoPosition(robot.grabber.grabber); // check wired
            } else if (gamepad2.b) {
                findServoPosition(robot.shooter.tilter);  // check wired
            } else if (gamepad2.x) {
                findServoPosition(robot.shooter.pusher);// check wired
            } else if (gamepad2.y) {
                findServoPosition(robot.sweeper.sweeper); // check wired
            }
            if (gamepad2.left_bumper) {
                robot.leftIntake.start();
                intakeMotor.setPower(iPower);
            } else {
                robot.leftIntake.stop();
                intakeMotor.setPower(0);
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
                iPower = iPower+.1f;
            } else  if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
                iPower = iPower-.1f;
            }
            if (gamepad2.right_bumper) {
//                robot.rightIntake.start();
                robot.shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
                robot.shooter.flywheelStart();
            } else {
                robot.rightIntake.stop();
            }
            if (gamepad2.left_trigger > .5) {
                robot.blocker.extend(); // check wired
            } else if (gamepad2.right_trigger > .5) {
                robot.blocker.retract(); // check wired
            } else {
                robot.blocker.stop();
            }

            if(gamepad2.left_stick_button){
                robot.grabber.reset();
                robot.shooter.launch();
            } else if(gamepad2.right_stick_button){
                robot.sweeper.reset();
                robot.shooter.launch();
            }
            robot.grabber.armTelemetry();
            robot.shooter.shooterTelemetry();
            robot.blocker.blockerTelemetry();
            robot.sweeper.sweeperTelemetry();
//            telemetry.addLine("sweeper servo: " + )
            telemetry.addLine("Intake Power: "+ iPower);
            telemetry.update();
        }
    }
}

