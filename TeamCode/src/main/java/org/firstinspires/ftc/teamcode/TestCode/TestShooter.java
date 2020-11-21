package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestShooter")
public class TestShooter extends LinearOpMode {
    DcMotor m1, m2;
    double FULL_POWER = 1;
    double HALF_POWER = .8;
    double SLOW = .2;


    @Override
    public void runOpMode() throws InterruptedException {

        m1 = hardwareMap.get(DcMotor.class, "m1");
//        m2 = hardwareMap.get(DcMotor.class, "m2");
        waitForStart();

        while(opModeIsActive() && !this.isStopRequested()){


            // Code for the 2 wheel shooter
            /*
            if (gamepad1.y) {
                m1.setPower(FULL_POWER);
                m2.setPower(-HALF_POWER);
            } else if (gamepad1.b) {
                m1.setPower(HALF_POWER);
                m2.setPower(-HALF_POWER);
            } else if (gamepad1.a) {
                m1.setPower(SLOW);
                m2.setPower(-SLOW);
            }
            if (gamepad1.x) {
                m1.setPower(0);
                m2.setPower(0);
            }
*/
            


            //Code for the 1 wheel shooter
/*
            if (gamepad1.y) {
                m1.setPower(1);
            } else if (gamepad1.b) {
                m1.setPower(0.9);
            } else if (gamepad1.a) {
                m1.setPower(0.8);
            } else if (gamepad1.dpad_up) {
                m1.setPower(0.7);
            } else if (gamepad1.dpad_left) {
                m1.setPower(0.6);
            }   else if (gamepad1.dpad_down) {
                m1.setPower(0.5);
            }
            if (gamepad1.x) {
                m1.setPower(0);
            }
*/

            //Code for the 2 stage shooter
            if (gamepad1.y) {
                m1.setPower(-1);
                //m2.setPower(1);
            } else if (gamepad1.b) {
                m1.setPower(-0.9);
               // m2.setPower(1);
            } else if (gamepad1.a) {
                m1.setPower(-0.8);
               // m2.setPower(1);
            } else if (gamepad1.dpad_up) {
                m1.setPower(-0.7);
               //// m2.setPower(1);
            } else if (gamepad1.dpad_left) {
                m1.setPower(-0.6);
             //   m2.setPower(1);
            }   else if (gamepad1.dpad_down) {
                m1.setPower(-0.5);
              //  m2.setPower(1);
            }


            if (gamepad1.x) {
                m1.setPower(0);
//                m2.setPower(0);
            }
//            if (gamepad1.dpad_right) {
//                m2.setPower(0);
//            }
        }
    }
}