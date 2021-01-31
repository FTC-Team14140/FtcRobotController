package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Intake {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    CRServo conveyorServo, rollerServo;
    double FULL_POWER = 1;
    double STOP = 0;
    public boolean intakeRunning = false;



    public Intake() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init(String conveyorServoName, String rollerServoName) {
        teamUtil.log("Initializing Intake");
        conveyorServo = hardwareMap.crservo.get(conveyorServoName);
        rollerServo = hardwareMap.crservo.get(rollerServoName);

    }

    // Starts the mechanisms at full speed
    public void start() {
        conveyorServo.setPower(FULL_POWER);
        rollerServo.setPower(FULL_POWER);
        intakeRunning = true;

    }

    // Stops the mechanisms
    public void stop() {
        conveyorServo.setPower(STOP);
        rollerServo.setPower(STOP);
        intakeRunning = false;
    }

}
