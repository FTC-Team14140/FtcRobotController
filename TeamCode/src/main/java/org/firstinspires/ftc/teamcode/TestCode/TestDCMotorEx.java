package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="TestDCMotorEx")
public class TestDCMotorEx extends LinearOpMode{

    DcMotorEx motor1, motor2, motor3, motor4;
    double INTAKE_POWER = -1.0;


    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive() && !this.isStopRequested()){
            if (gamepad1.a) {
                motor1.setVelocity(500);
                motor2.setVelocity(500);
            } else {
                motor1.setVelocity(0);
                motor2.setVelocity(0);
            }
        }

    }
}