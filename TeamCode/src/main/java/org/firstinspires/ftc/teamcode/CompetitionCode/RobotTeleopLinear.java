package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.GrabberArm;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
import org.firstinspires.ftc.teamcode.Assemblies.Blocker;
import org.firstinspires.ftc.teamcode.Assemblies.Shooter;
import org.firstinspires.ftc.teamcode.Assemblies.Intake;

@TeleOp(name = "RobotTeleopLinear", group = "z")
public class RobotTeleopLinear extends LinearOpMode {
    TeamGamepad teamGamePad;
    Robot robot;
    double storedHeading;
    boolean blockerExtended = false;


    public void initialize() {

        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");
        teamUtil.telemetry.update();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT);

        teamGamePad = new TeamGamepad(this);
        robot = new Robot(this);
        robot.init(false);
        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);
        //teamUtil.initPerf();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        teamUtil.telemetry.addLine("Let's Drive!");
        teamUtil.telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            teamUtil.telemetry.addData("Heading:", robot.drive.getHeading());
            teamGamePad.gamepadLoop();


            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //Gamepad 1 code
            //this code is for the drive
            if (gamepad1.left_trigger > 0.5) {
                robot.drive.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x, true,
                        robot.drive.getHeading(), storedHeading);

            } else {
                robot.drive.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x, false,
                        robot.drive.getHeading(), storedHeading);

            }

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                robot.drive.resetHeading();
            }


            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1X)) {
                if (!robot.rightIntake.intakeRunning && !robot.leftIntake.intakeRunning) {
                    robot.leftIntake.start();
                    robot.rightIntake.start();
                } else {
                    robot.leftIntake.stop();
                    robot.rightIntake.stop();

                }
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1RB)) {
                if (!robot.blocker.blockerExtended) {
                    robot.blocker.extendNoWait();
                } else {
                    robot.blocker.retractNoWait();

                }
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //Gamepad 2 code

            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
                robot.grabber.liftNoWait();

            }
            if  (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
                robot.grabber.moveToReadyNoWait();
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2RIGHTTRIGGER)) {
                if (robot.grabber.currentGrabberPosition == GrabberArm.GRABBER_POS.OPEN) {
                    robot.grabber.grab();

                } else {
                    robot.grabber.release();

                }
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2RIGHTBUMPPER)) {
                if (!robot.shooter.motorRunning) {
                    robot.shooter.flywheelStart();

                } else {
                    robot.shooter.stopFlywheel();

                }
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2RIGHTTRIGGER)) {
                if (robot.shooter.flywheelReady()) {
                    robot.shooter.launch();
                }

            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2Y)) {
                robot.shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);

            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2X)) {
                robot.shooter.aimAt(Shooter.ShooterTarget.HIGH_GOAL);

            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT)) {
                if (!robot.sweeper.motorRunning) {
                    robot.sweeper.retract();
                }else {
                    robot.sweeper.stop();
                }
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
                if (!robot.sweeper.motorRunning) {
                    robot.sweeper.extend();
                }else {
                    robot.sweeper.stop();
                }
            }
            //sweeper is the servo
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2B)) {
                robot.sweeper.manualControl(1);
                //SWEEP position
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2A)) {
                robot.sweeper.manualControl(0);
                //READY Position
            }
            //this code is the telemetry
            teamUtil.telemetry.update();

            // teamUtil.trackPerf();
        }
    }
}
