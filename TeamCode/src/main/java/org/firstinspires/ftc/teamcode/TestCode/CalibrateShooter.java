package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "CalibrateShooter")
@Disabled
public class CalibrateShooter extends LinearOpMode {

    //Robot robot;
    TeamGamepad teamGamePad;
    double MAX_VELOCITY = 2500;
    double CURRENT_SPEED = 100;
    DcMotorEx shooter;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        //robot = new Robot(this);
        teamGamePad = new TeamGamepad(this);
        //robot.init(true);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        teamUtil.initPerf();
    }

    public void findMaxVelocity() {
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double max = 0, v;
        shooter.setVelocity(3000);
        while (gamepad2.left_trigger>.5) {
            max = (v = shooter.getVelocity()) > max ? v : max;
        }
        shooter.setVelocity(0);
        teamUtil.log("Shooter Max Velocity:"+max);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //robot.drive.resetHeading();

        while (opModeIsActive()) {
            teamGamePad.gamepadLoop();

            // Run shooter motor at current speed
            if (gamepad2.left_bumper) {
                shooter.setVelocity(CURRENT_SPEED);
            } else {
                shooter.setVelocity(0);
            }

            if (gamepad2.left_trigger>.5) {
                findMaxVelocity();
            }

            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADUP)) {
                CURRENT_SPEED = CURRENT_SPEED + 100;
                } else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD2DPADDOWN)) {
                CURRENT_SPEED = CURRENT_SPEED - 100;
            }

            teamUtil.telemetry.addData("CURRENT:", CURRENT_SPEED);
            telemetry.addLine("Velocity:"+ shooter.getVelocity());
            teamUtil.telemetry.update();
        }
    }
}






