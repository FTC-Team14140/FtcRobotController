package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Intake {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    CRServo conveyorServo, rollerServo;
    DcMotor intakeMotor;
    boolean usesServo;
    double SERVO_POWER = 1;
    float INTAKE_MOTOR_POWER = 0.7f;
    float STOP = 0f;
    public boolean intakeRunning = false;



    public Intake() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }

    void init(String conveyorServoName, boolean reverseConveyer, String rollerServoName, boolean reverseRoller, boolean usesServo) {
        this.usesServo = usesServo;
        teamUtil.log("Initializing Intake");
        if(usesServo){
            conveyorServo = hardwareMap.crservo.get(conveyorServoName);
            if (reverseConveyer) {
                conveyorServo.setDirection(DcMotorSimple.Direction.REVERSE);
            }

        } else {
            intakeMotor = hardwareMap.get(DcMotor.class, conveyorServoName);
            if (reverseConveyer) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }

        rollerServo = hardwareMap.crservo.get(rollerServoName);
        if (reverseRoller) {
            rollerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }


    }

    // Starts the mechanisms at full speed
    public void start() {
        if(usesServo){
            conveyorServo.setPower(SERVO_POWER);
        } else {
            intakeMotor.setPower(INTAKE_MOTOR_POWER);
        }

        rollerServo.setPower(SERVO_POWER);

        intakeRunning = true;

    }

    // Starts the mechanisms at full speed AT REVERSE
    public void reverse() {
        if(usesServo){
            conveyorServo.setPower(-SERVO_POWER);
        } else {
            intakeMotor.setPower(-INTAKE_MOTOR_POWER);
        }

        rollerServo.setPower(-SERVO_POWER);

        intakeRunning = true;

    }


    // Stops the mechanisms
    public void stop() {
        if(usesServo){
            conveyorServo.setPower(STOP);
        } else {
            intakeMotor.setPower(STOP);
        }
        rollerServo.setPower(STOP);
        intakeRunning = false;
    }

}
