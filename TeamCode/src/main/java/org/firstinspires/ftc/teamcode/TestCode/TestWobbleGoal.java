package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TestWobbleGoal extends LinearOpMode {

    DcMotorEx wobbleMotor;

    public void runOpMode() throws InterruptedException {

        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");

        waitForStart();

    }

}
