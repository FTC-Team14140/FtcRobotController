package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "CalibrateDriveSystem")
public class CalibrateDriveSystem extends LinearOpMode {

    Robot robot;
    TeamGamepad teamGamePad;

    public void initialize() {
        teamUtil.init(this);
        robot = new Robot(this);
        teamGamePad = new TeamGamepad(this);
        robot.init(true);
        teamUtil.initPerf();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        robot.drive.resetHeading();

        while (opModeIsActive()) {
            teamGamePad.gamepadLoop();

            // Code to test autonomous rotation
            if (gamepad1.dpad_up) {
                robot.drive.rotateTo(RobotDrive.RobotRotation.GOAL);
            }
            if (gamepad1.dpad_down) {
                robot.drive.rotateTo(RobotDrive.RobotRotation.START);
            }
            if (gamepad1.dpad_left) {
                robot.drive.rotateTo(RobotDrive.RobotRotation.THEIR_SIDE);
            }
            if (gamepad1.dpad_right) {
                robot.drive.rotateTo(RobotDrive.RobotRotation.OUR_SIDE);
            }

            // Code to get max velocities
            if (gamepad2.left_bumper) {
                robot.drive.findMaxForwardSpeed();
            } else if (gamepad2.right_bumper) {
                robot.drive.findMaxLeftSpeed();
            }

            // HOLD the GP2 left TRIGGER to adjust the movement parameters
            if (gamepad2.left_trigger > 0.5) {
                if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
                    robot.drive.START_SPEED = robot.drive.START_SPEED + 10;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
                    robot.drive.START_SPEED = robot.drive.START_SPEED - 10;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT)) {
                    robot.drive.END_SPEED = robot.drive.END_SPEED - 10;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
                    robot.drive.END_SPEED = robot.drive.END_SPEED + 10;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2Y)) {
                    robot.drive.MAX_ACCEL_PER_INCH = robot.drive.MAX_ACCEL_PER_INCH + 10;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2A)) {
                    robot.drive.MAX_ACCEL_PER_INCH = robot.drive.MAX_ACCEL_PER_INCH - 10;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2X)) {
                    robot.drive.MAX_DECEL_PER_INCH = robot.drive.MAX_DECEL_PER_INCH - 10;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2B)) {
                    robot.drive.MAX_DECEL_PER_INCH = robot.drive.MAX_DECEL_PER_INCH + 10;
                }
                // HOLD the GP2 Right TRIGGER to trigger movements
            } else if(gamepad2.right_trigger > 0.5){
                if (gamepad2.dpad_up) {
                    robot.drive.moveInches(0, 36, 7000);
                } else if (gamepad2.dpad_down) {
                    robot.drive.moveInches(180, 36, 7000);
                } else if (gamepad2.dpad_left) {
                    robot.drive.moveInches(90, 36, 7000);
                } else if (gamepad2.dpad_right) {
                    robot.drive.moveInches(270, 36, 7000);
                } else if (gamepad2.y) {
                    robot.drive.moveInches(45, 36, 7000);
                } else if (gamepad2.a) {
                    robot.drive.moveInches(225, 36, 7000);
                } else if (gamepad2.x) {
                    robot.drive.moveInches(90, 36, 7000);
                } else if (gamepad2.b) {
                    robot.drive.moveInches(315, 36, 7000);
                }
            }


            robot.drive.telemetryDriveEncoders();
            teamUtil.telemetry.addData("heading:", robot.drive.getHeading());
            telemetry.addLine("Start:"+ robot.drive.START_SPEED+" End:"+robot.drive.END_SPEED+" Acc:"+robot.drive.MAX_ACCEL_PER_INCH+" Dec:"+robot.drive.MAX_DECEL_PER_INCH);
            teamUtil.telemetry.update();

        }
    }
}


