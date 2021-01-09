package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "RobotTeleopLinear", group = "z")
public class RobotTeleopLinear extends LinearOpMode {
    TeamGamepad teamGamePad;
    Robot robot;
    double storedHeading;

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

            //this code is the telemetry
            teamUtil.telemetry.update();

            // teamUtil.trackPerf();
        }
    }
}
