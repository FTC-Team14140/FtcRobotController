package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="testContinuousServos")
public class testContinuousServos extends LinearOpMode {

    CRServo servo1, servo2;
    double FULL_POWER = 1;
    double HALF_POWER = .5;
    double SLOW = .1;
    double STOP = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.crservo.get("s1");
        //servo2 = hardwareMap.crservo.get("s2");

        waitForStart();

        while(opModeIsActive() && !this.isStopRequested()){

            if (gamepad1.y) {
                servo1.setPower(FULL_POWER);
            } else if (gamepad1.b) {
                servo1.setPower(HALF_POWER);
            } else if (gamepad1.a) {
                servo1.setPower(SLOW);
            }
            if (gamepad1.x) {
                servo1.setPower(STOP);
            }
        }
    }
}