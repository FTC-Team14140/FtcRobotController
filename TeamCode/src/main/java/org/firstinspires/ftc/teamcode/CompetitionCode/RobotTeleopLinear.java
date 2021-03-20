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
        teamGamePad = new TeamGamepad(this);
        robot = new Robot(this);
        robot.init(false);

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

            if (true  ) { // Current controls

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   DRIVE Control - GamePad 1
                if (gamepad1.left_trigger > 0.5) {
                    robot.drive.universalJoystick(gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x, true,
                            robot.drive.getHeading(), storedHeading);

                } else {
                    robot.drive.universalJoystick(gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x, true,
                            robot.drive.getHeading(), storedHeading);

                }

                if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                    robot.drive.resetHeading();
                }


                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   INTAKE control - GamePad 1
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1X)) {
                    if (!robot.rightIntake.intakeRunning && !robot.leftIntake.intakeRunning) {
                        robot.leftIntake.start();
                        robot.rightIntake.start();
                    } else {
                        robot.leftIntake.stop();
                        robot.rightIntake.stop();

                    }
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   BLOCKER control - GamePad 1
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1RB)) {
                    if (!robot.blocker.blockerExtended) {
                        robot.blocker.extendNoWait();
                    } else {
                        robot.blocker.retractNoWait();

                    }
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   WOBBLE GOAL GRABBER control - GamePad 1 & 2
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1B)) {
                    robot.grabber.stowNoWait();
                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
                    robot.grabber.liftNoWait();
                }
                if  (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
                    robot.grabber.moveToReadyNoWait();
                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1LEFTTRIGGER)) {
                    //teamUtil.log("grabber control button triggered");
                    if (robot.grabber.currentGrabberPosition == GrabberArm.GRABBER_POS.OPEN) {
                        //teamUtil.log("grabber position is open ");
                        //teamUtil.log("grabber closed on button press");
                        robot.grabber.grab();
                    } else {
                        robot.grabber.release();
                    }
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   SHOOTER control
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2RB)) { // Flywheel on/off
                    if (!robot.shooter.motorRunning) {
                        robot.shooter.flywheelStart();
                    } else {
                        robot.shooter.stopFlywheel();
                    }
                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2Y)) { // Aim at powershot
                    robot.shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);

                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2X)) { // Aim at high goal
                    robot.shooter.aimAt(Shooter.ShooterTarget.HIGH_GOAL);

                }
//                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1A)) {  // launch  TODO Maybe trigger this code if the button is held down?
//                    if (robot.shooter.flywheelReady()) {
//                        robot.shooter.launchNoWait();
//                    }
//                }
                robot.shooter.pusherManualControl(gamepad1.right_trigger); // Pusher for launch and clearing jams

                // manual aiming (not sure about these controls for this...
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT)) {
                    robot.shooter.tilter.setPosition(robot.shooter.tilter.getPosition()-.0025);
                }
                if  (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
                    robot.shooter.tilter.setPosition(robot.shooter.tilter.getPosition()+.0025);
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   SWEEPER control
                if(gamepad2.left_stick_y > 0.1){
                    //robot.sweeper.retract();
                    robot.sweeper.retract2(gamepad2.left_stick_y);
                } else if(gamepad2.left_stick_y < -0.1){
                    //robot.sweeper.extend();
                    robot.sweeper.extend2(-gamepad2.left_stick_y);
                }  else {
                    robot.sweeper.stop();
                }
                robot.sweeper.manualControl(gamepad2.right_trigger);

            /*  WARNING These Methods are not tested and maybe don't work...
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT)) {
                if (!robot.sweeper.motorRunning) {
                    robot.sweeper.retractFully();
                }
            } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
                if (!robot.sweeper.motorRunning) {
                    robot.sweeper.extendFully();
                }
            }
            */

            } else { // New Controls

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   DRIVE Control - GamePad 1
                robot.drive.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x, !gamepad1.right_bumper,
                        robot.drive.getHeading(), storedHeading);

                if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                    robot.drive.resetHeading();
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   SHOOTER control
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1B)) { // Flywheel on/off
                    if (!robot.shooter.motorRunning) {
                        robot.shooter.flywheelStart();
                    } else {
                        robot.shooter.stopFlywheel();
                    }
                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1A)) { // Aim at powershot
                    robot.shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);

                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1Y)) { // Aim at high goal
                    robot.shooter.aimAt(Shooter.ShooterTarget.HIGH_GOAL);

                }
                robot.shooter.pusherManualControl(gamepad1.right_trigger); // Pusher for launch and clearing jams

                // manual aiming
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADUP)) {
                    robot.shooter.tilter.setPosition(robot.shooter.tilter.getPosition()+.005);
                }
                if  (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADDOWN)) {
                    robot.shooter.tilter.setPosition(robot.shooter.tilter.getPosition()-.005);
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   INTAKE control - GamePad 1
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADRIGHT)) {
                    if (!robot.leftIntake.intakeRunning) {
                        robot.leftIntake.start();
                    } else {
                        robot.leftIntake.stop();
                    }
                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADLEFT)) {
                    if (!robot.rightIntake.intakeRunning) {
                        robot.rightIntake.start();
                    } else {
                        robot.rightIntake.stop();
                    }
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   WOBBLE GOAL GRABBER control - GamePad 1 for grabber & 2 for arm
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
                    robot.grabber.stowNoWait();
                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
                    robot.grabber.liftNoWait();
                }
                if  (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT) || teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
                    robot.grabber.moveToReadyNoWait();
                }
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1LEFTTRIGGER)) {
                    //teamUtil.log("grabber control button triggered");
                    if (robot.grabber.currentGrabberPosition == GrabberArm.GRABBER_POS.OPEN) {
                        //teamUtil.log("grabber position is open ");
                        //teamUtil.log("grabber closed on button press");
                        robot.grabber.grab();
                    } else {
                        robot.grabber.release();
                    }
                }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   SWEEPER control GamePad 2
                if(gamepad2.left_stick_y > 0.1){
                    robot.sweeper.retract2(gamepad2.left_stick_y);
                } else if(gamepad2.left_stick_y < -0.1){
                    robot.sweeper.extend2(-gamepad2.left_stick_y);
                }  else {
                    robot.sweeper.stop();
                }
                robot.sweeper.manualControl(gamepad2.right_trigger);

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //   BLOCKER control - GamePad 2
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2LB)) {
                    if (!robot.blocker.blockerExtended) {
                        robot.blocker.extendNoWait();
                    } else {
                        robot.blocker.retractNoWait();

                    }
                }
            }


            //this code is the telemetry
            robot.shooter.shooterTelemetry();
            //robot.sweeper.sweeperTelemetry();
            teamUtil.telemetry.update();

            // teamUtil.trackPerf();
        }
    }
}
