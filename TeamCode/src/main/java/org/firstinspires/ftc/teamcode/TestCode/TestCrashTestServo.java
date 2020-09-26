package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.basicLibs.CrashTestServo;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;

@TeleOp( name ="TestCrashTestServo")
public class TestCrashTestServo extends LinearOpMode {

    CrashTestServo testServo;
    TeamGamepad teamGamepad;
    double desiredPosition = 1.0;
    double degPerSec = 20;

    double SERVO_LOWER_LIM = 0.0;
    double SERVO_HIGHER_LIM = 1.0;

    double DEG_PER_SEC_LOWER_LIM = 0;
    double DEG_PER_SEC_HIGHER_LIM = 1000;



    @Override
    public void runOpMode() throws InterruptedException {

        testServo = new CrashTestServo("testServo", 0.16);
        teamGamepad = new TeamGamepad(this);
        testServo.setPosition(0);
        waitForStart();


        while(opModeIsActive() && !this.isStopRequested()){
            teamGamepad.gamepadLoop();

            if(gamepad1.dpad_up) {
                desiredPosition = Range.clip( desiredPosition + 0.05, SERVO_LOWER_LIM, SERVO_HIGHER_LIM);
            } else if(gamepad1.dpad_down){
                desiredPosition = Range.clip( desiredPosition - 0.05, SERVO_LOWER_LIM, SERVO_HIGHER_LIM);
            }

            if(gamepad1.dpad_right) {
                degPerSec = Range.clip(degPerSec + 10, DEG_PER_SEC_LOWER_LIM, DEG_PER_SEC_HIGHER_LIM);
            } else if(gamepad1.dpad_left){
                degPerSec = Range.clip(degPerSec - 10, DEG_PER_SEC_LOWER_LIM, DEG_PER_SEC_HIGHER_LIM);
            }

            if(gamepad1.a) {
                testServo.setPosition(desiredPosition);
            } else if(gamepad1.y){
                testServo.runToPosition(degPerSec, desiredPosition);
            }


            telemetry.addData("desiredPos", desiredPosition);
            telemetry.addData("degress per sec", degPerSec);
            telemetry.update();


        }



    }
}
