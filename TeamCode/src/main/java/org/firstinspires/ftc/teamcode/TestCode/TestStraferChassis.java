package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;

@TeleOp(name="Test Strafer Chassis")
public class TestStraferChassis extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor rearRight;
    DcMotor rearLeft;
    TeamGamepad gamepad;

    double power = 0.6;
    double time = 1200;

    public void cutPower(){

        frontRight.setPower(0);
        frontLeft.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);
    }


    public void runWithTime(double time){
        long endTime = System.currentTimeMillis() + (long)time;

        while(System.currentTimeMillis() < endTime){
            telemetry.addData("running motors", 0);
            frontRight.setPower(power);
            frontLeft.setPower(-power);
            rearRight.setPower(power);
            rearLeft.setPower(-power);
        }
        cutPower();
        telemetry.update();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        gamepad = new TeamGamepad(this);

        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        rearLeft = hardwareMap.dcMotor.get("RL");
        rearRight = hardwareMap.dcMotor.get("RR");

        waitForStart();

        while(opModeIsActive() && !this.isStopRequested()){

            if(gamepad1.x){
                runWithTime(time);
            } else if(gamepad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADUP)){
                power = power + 0.01;
            } else if(gamepad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADDOWN)){
                power = power - 0.01;
            } else if(gamepad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADLEFT)){
                time = time - 5;
            } else if(gamepad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADRIGHT)){
                time = time + 5;
            }

            telemetry.addData("power: ", power);
            telemetry.addData("time: ", time);
            telemetry.update();


            gamepad.gamepadLoop();







        }


    }


}
