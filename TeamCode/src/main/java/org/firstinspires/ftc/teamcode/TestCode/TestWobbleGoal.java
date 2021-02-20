package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestWobbleGoal")
@Disabled
public class TestWobbleGoal extends LinearOpMode {

    final double forwardSpeed = 100;
    final double backwardSpeed = -100;
    final double open = 0.5;
    final double close = 0.98;

    Servo grabServo;
    DcMotorEx wobbleMotor;
    TeamGamepad teamGamePad;

    public void runOpMode() throws InterruptedException {

        teamGamePad = new TeamGamepad(this);

        grabServo = hardwareMap.servo.get("grabServo");
        grabServo.setPosition(open);

        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            teamGamePad.gamepadLoop();
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1A)) {
                wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                wobbleMotor.setVelocity(backwardSpeed);
                teamUtil.pause(1000);
                wobbleMotor.setVelocity(0);
                wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADRIGHT)){
                wobbleMotor.setVelocity(0);
                wobbleMotor.setTargetPosition(0);
                wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleMotor.setVelocity(backwardSpeed);
            }else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADLEFT)){
                wobbleMotor.setVelocity(0);
                wobbleMotor.setTargetPosition(370);
                wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleMotor.setVelocity(forwardSpeed);
            }else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1X)){
                grabServo.setPosition(open);
            }else if (teamGamePad.wasBounced(TeamGamepad.buttons.GAMEPAD1B)){
                grabServo.setPosition(close);
            }
            telemetry.addData("EncoderPos:", wobbleMotor.getCurrentPosition());
            telemetry.update();
        }

    }

}
