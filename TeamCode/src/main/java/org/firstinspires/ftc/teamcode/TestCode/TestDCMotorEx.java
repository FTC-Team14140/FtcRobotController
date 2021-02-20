package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;


@TeleOp(name="TestDCMotorEx")
@Disabled
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
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorConfigurationType m1Config = motor1.getMotorType();
        MotorConfigurationType m2Config = motor1.getMotorType();

        teamUtil.log("ticks" + motor1.getMotorType().getTicksPerRev());
        waitForStart();

        while(opModeIsActive() && !this.isStopRequested()){
            if (gamepad1.a) {
                motor1.setVelocity(360, AngleUnit.DEGREES);
                motor2.setVelocity(360, AngleUnit.DEGREES);
            } else {
                motor1.setVelocity(0);
                motor2.setVelocity(0);
            }
            teamUtil.log("encoder:"+motor1.getCurrentPosition());
        }

    }
}