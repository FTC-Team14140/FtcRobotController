package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "CalibrateDriveSystem")
public class CalibrateDriveSystem extends LinearOpMode {

    Robot robot;
    TeamGamepad teamGamePad;
    double testVelocity = 500;
    double HEADING = 0;
    long TIME = 3;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        robot = new Robot(this);
        teamGamePad = new TeamGamepad(this);
        robot.init(true);
        teamUtil.initPerf();
    }

    public void testDriveMotorWiring() {
        robot.drive.setDriveVelocities(robot.drive.MIN_START_SPEED,0,0,0);
        teamUtil.pause(1000);
        robot.drive.setDriveVelocities(0,robot.drive.MIN_START_SPEED,0,0);
        teamUtil.pause(1000);
        robot.drive.setDriveVelocities(0,0,robot.drive.MIN_START_SPEED,0);
        teamUtil.pause(1000);
        robot.drive.setDriveVelocities(0,0,0,robot.drive.MIN_START_SPEED);
        teamUtil.pause(1000);
        robot.drive.stopDrive();
    }

    public void moveNoAccelerateNoHeadingControl () {
        robot.drive.driveMotorsHeadings(HEADING, robot.drive.getHeading(), robot.drive.MIN_START_SPEED);
        teamUtil.pause(TIME*1000);
        robot.drive.stopDrive();
    }

    public void moveNoAccelerateWithHeadingControl () {
        long doneTime = System.currentTimeMillis() + (int)(TIME*1000);
        double heldHeading = robot.drive.getHeading();
        while (System.currentTimeMillis() < doneTime) {
            robot.drive.driveMotorsHeadings(HEADING, heldHeading, robot.drive.MIN_START_SPEED);
        }
        robot.drive.stopDrive();
    }

    public void goForADrive() {
        robot.drive.moveInches(0, 36, 7000);
        robot.drive.moveInches(90, 36, 7000);
        robot.drive.moveInches(180, 36, 7000);
        robot.drive.moveInches(270, 36, 7000);
        robot.drive.moveInches(45, 44, 7000);
        robot.drive.moveInches(180, 32, 7000);
        robot.drive.moveInches(315, 44, 7000);
        robot.drive.moveInches(180, 32, 7000);
    }

    public void adjustSpinThresholds () {
        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
            robot.drive.SPIN_DECEL_THRESHOLD = robot.drive.SPIN_DECEL_THRESHOLD + 1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
            robot.drive.SPIN_DECEL_THRESHOLD = robot.drive.SPIN_DECEL_THRESHOLD - 1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT)) {
            robot.drive.SPIN_SLOW_THRESHOLD = robot.drive.SPIN_SLOW_THRESHOLD + 1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
            robot.drive.SPIN_SLOW_THRESHOLD = robot.drive.SPIN_SLOW_THRESHOLD - 1;
        }
    }

    public void adjustBasicMovementParams() {
        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
            robot.drive.MIN_START_SPEED = robot.drive.MIN_START_SPEED + 10;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
            robot.drive.MIN_START_SPEED = robot.drive.MIN_START_SPEED - 10;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT)) {
            robot.drive.MIN_END_SPEED = robot.drive.MIN_END_SPEED - 10;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
            robot.drive.MIN_END_SPEED = robot.drive.MIN_END_SPEED + 10;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2Y)) {
            robot.drive.MAX_ACCEL_PER_INCH = robot.drive.MAX_ACCEL_PER_INCH + 10;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2A)) {
            robot.drive.MAX_ACCEL_PER_INCH = robot.drive.MAX_ACCEL_PER_INCH - 10;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2X)) {
            robot.drive.MAX_DECEL_PER_INCH = robot.drive.MAX_DECEL_PER_INCH - 10;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2B)) {
            robot.drive.MAX_DECEL_PER_INCH = robot.drive.MAX_DECEL_PER_INCH + 10;
        }
    }

    public void testSpins() {
        if (gamepad2.dpad_up) {
            robot.drive.rotateTo(RobotDrive.RobotRotation.GOAL);
        }
        if (gamepad2.dpad_down) {
            robot.drive.rotateTo(RobotDrive.RobotRotation.START);
        }
        if (gamepad2.dpad_left) {
            robot.drive.rotateTo(RobotDrive.RobotRotation.THEIR_SIDE);
        }
        if (gamepad2.dpad_right) {
            robot.drive.rotateTo(RobotDrive.RobotRotation.OUR_SIDE);
        }

    }

    public void testMoveInches() {
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
            robot.drive.moveInches(135, 36, 7000);
        } else if (gamepad2.b) {
            robot.drive.moveInches(315, 36, 7000);
        }
    }

    public void adjustDistanceSensorMovementConstants () {
        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
            robot.drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD = robot.drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD + .1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
            robot.drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD = robot.drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD - .1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADLEFT)) {
            robot.drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE = robot.drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE - .1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADRIGHT)) {
            robot.drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE = robot.drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE + .1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2Y)) {
            robot.drive.MOVE_TO_DISTANCE_SLOW_DISTANCE = robot.drive.MOVE_TO_DISTANCE_SLOW_DISTANCE + 1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2A)) {
            robot.drive.MOVE_TO_DISTANCE_SLOW_DISTANCE = robot.drive.MOVE_TO_DISTANCE_SLOW_DISTANCE - 1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2X)) {
            robot.drive.MOVE_TO_DISTANCE_DECEL_DISTANCE = robot.drive.MOVE_TO_DISTANCE_DECEL_DISTANCE - 1;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2B)) {
            robot.drive.MOVE_TO_DISTANCE_DECEL_DISTANCE = robot.drive.MOVE_TO_DISTANCE_DECEL_DISTANCE + 1;
        }
    }

    public void testMoveToDistance() {
        if (gamepad2.right_bumper) {
            // Test starting from a stop
            robot.drive.moveToDistance(robot.drive.rightDistance, 270, 5, 7000);
        } else if (gamepad2.left_bumper) {
            // Test starting from a full speed drive
            robot.drive.moveInches(0, 48, 7000, robot.drive.DRIVE_MAX_MOVE_TO_DISTANCE_VELOCITY);
            robot.drive.moveToDistance(robot.drive.frontDistance, 0, 5, 7000);
        }
    }

    public void adjustMoveToLineSpeed () {
        if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
            testVelocity = testVelocity + 100;
        } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
            testVelocity = testVelocity - 100;
        }
    }

    public void testMoveToLine() {
        if (gamepad2.right_bumper) {
            // Test starting from a stop
            robot.drive.moveToLine(robot.drive.frontLeftColor, teamColorSensor.TapeColor.WHITE, 0, 7000);
            robot.drive.stopDrive();
        } else if (gamepad2.left_bumper) {
            // Test starting from drive at various velocities
            robot.drive.moveInches(0, 36, 7000, robot.drive.FIND_LINE_SPEED); // go fast then slow down to find tape
            robot.drive.moveToLine(robot.drive.frontLeftColor, teamColorSensor.TapeColor.RED, 0, 7000);
            robot.drive.moveInches(0, 5, 7000); // move a certain distance after tape and stop
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        robot.drive.resetHeading();

        while (opModeIsActive()) {
            teamGamePad.gamepadLoop();

            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                robot.drive.resetHeading();
            } else if (gamepad2.right_stick_button) {
                adjustDistanceSensorMovementConstants();
                testMoveToDistance();
            } else if (gamepad2.left_stick_button) {
                adjustMoveToLineSpeed();
                testMoveToLine();
            }

            // HOLD the GP2 left TRIGGER to adjust the movement parameters
            if (gamepad2.left_trigger > 0.5) {
                adjustBasicMovementParams();
            }

            teamUtil.telemetry.addData("heading:", robot.drive.getHeading());
            robot.drive.distanceTelemetry();
            robot.drive.colorTelemetry();
            robot.drive.rawColorTelemetry();
            telemetry.addLine("DistNoMv:"+ robot.drive.MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD +" Drift:"+robot.drive.MOVE_TO_DISTANCE_DRIFT_DISTANCE +" Slow:"+robot.drive.MOVE_TO_DISTANCE_SLOW_DISTANCE+" Dec:"+robot.drive.MOVE_TO_DISTANCE_DECEL_DISTANCE);
            telemetry.addLine("Velocity:"+ testVelocity);

            //robot.drive.telemetryDriveEncoders();
            telemetry.addLine("Start:"+ robot.drive.MIN_START_SPEED +" End:"+robot.drive.MIN_END_SPEED +" Acc:"+robot.drive.MAX_ACCEL_PER_INCH+" Dec:"+robot.drive.MAX_DECEL_PER_INCH);
            //telemetry.addLine("SpinSLOW:"+ robot.drive.DRIVE_SLOW_SPIN_VELOCITY+" SpinMAX:"+robot.drive.DRIVE_MAX_SPIN_VELOCITY+" SlowThreshold:"+robot.drive.SPIN_SLOW_THRESHOLD+" Dec:"+robot.drive.SPIN_DECEL_THRESHOLD);
            //telemetry.addLine("Heading:"+ HEADING+" TIME:"+TIME);
            teamUtil.telemetry.update();

        }
    }
}


