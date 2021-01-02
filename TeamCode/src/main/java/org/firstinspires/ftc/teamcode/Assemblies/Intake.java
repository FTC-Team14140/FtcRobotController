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

    void Intake() {
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
    void start() {

    }

    // Stops the mechanisms
    void stop() {

    }

}
