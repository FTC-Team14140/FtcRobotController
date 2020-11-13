package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;


@TeleOp(name = "TestDriveSystem")
public class TestDriveSystem extends LinearOpMode {

    // lift system code - should be in its own assembly class...

    public static double MAX_POWER = 1;
    public double DRIVE_POWER = 0.5;
    Robot robot;
    boolean wasTurning = false;
    double storedHeading;

    public void initialize() {
        teamUtil.init(this);
        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);
        robot = new Robot(this);

//        teamGamePad = new TeamGamepad(this);

        robot.init(true);
        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);
        teamUtil.initPerf();
        //robot.latch.latchUp(); // move latches up at start of teleop

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        storedHeading = robot.drive.getHeading();
        waitForStart();

        while (opModeIsActive()) {

            // Code to test the joystick control of the drive
            if(Math.abs(gamepad1.right_stick_x) < 0.1 && wasTurning){
                storedHeading = robot.drive.getHeading();
            }
            robot.drive.universalJoystick(gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x, true,
                    robot.drive.getHeading(), storedHeading);

            // Reset heading
            if (gamepad1.a) {
                robot.drive.resetHeading();
            }

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

            // Code to get operating parameters
            if (gamepad2.left_bumper) {
                robot.drive.findMaxForwardSpeed();
            } else if (gamepad2.right_bumper) {
                robot.drive.findMaxLeftSpeed();
            }

            if (gamepad2.dpad_up) {
                robot.drive.accelerateInchesForward(RobotDrive.MAX_MOTOR_VELOCITY, 36, robot.drive.getHeading(), 7000);
            } else if (gamepad2.dpad_down) {
                robot.drive.accelerateInchesBackward(RobotDrive.MAX_MOTOR_VELOCITY, 36, robot.drive.getHeading(), 7000);
            } else if (gamepad2.dpad_left) {
                //robot.drive.accelerateInchesLeft(RobotDrive.MAX_MOTOR_VELOCITY, 36, robot.drive.getHeading(), 7000);
            } else if (gamepad2.dpad_right) {
                //robot.drive.accelerateInchesRight(RobotDrive.MAX_MOTOR_VELOCITY, 36, robot.drive.getHeading(), 7000);
            } else if (gamepad2.a) {
                robot.drive.stopMotors();
            }

            // TODO Need some code here to adjust max acceleration and deceleration parameters for testing


//            robot.driveTelemetry();
            teamUtil.telemetry.addData("heading:", robot.drive.getHeading());
            //teamUtil.telemetry.addData("ColorSensor: ", robot.drive.bottomColor.getReading());
            //teamUtil.telemetry.addData("ColorSensor BlueTape?: ", robot.drive.bottomColor.onBlue());
            //teamUtil.telemetry.addData("ColorSensor RedTape?: ", robot.drive.bottomColor.onRed());


            //robot.drive.distanceTelemetry();
            robot.drive.telemetryDriveEncoders();
            teamUtil.telemetry.update();

        }
    }
}


