package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestTubingIntake")
public class TestTubingIntake extends LinearOpMode {

    DcMotor theMotor;
    double FULL_POWER = 1.0;
    double HALF_POWER = .5;
    double SLOW = .2;


    @Override
    public void runOpMode() throws InterruptedException {

        theMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        waitForStart();

        while(opModeIsActive() && !this.isStopRequested()){

            if (gamepad1.y) {
                theMotor.setPower(FULL_POWER);
            } else if (gamepad1.b) {
                theMotor.setPower(HALF_POWER);
            } else if (gamepad1.a) {
                theMotor.setPower(SLOW);
            }
            if (gamepad1.x) {
                theMotor.setPower(0);
            }
        }
    }
}