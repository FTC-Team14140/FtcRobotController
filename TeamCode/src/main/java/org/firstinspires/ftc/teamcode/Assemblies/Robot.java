package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.OLD.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.OLD.LiftSystem;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

// A class to encapsulate the entire 
// This class is designed to be used ONLY in a linearOpMode (for Auto OR Teleop)
public class Robot {
    int path;
    public boolean hasBeenInitialized = false;
    private static boolean justRanAuto;

    static {
        justRanAuto = false;
    }

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Blocker blocker;
    public Sweeper sweeper;
    public GrabberArm grabber;
    public Intake leftIntake, rightIntake;
    public Shooter shooter;
    public RobotDrive drive;

    //SkystoneDetector detector;
    boolean timedOut = false;


    public Robot(LinearOpMode opMode) {
        teamUtil.log("Constructing Robot");

        // stash some context for later
        teamUtil.theOpMode = opMode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        teamUtil.log("Constructing Assemblies");
        drive = new RobotDrive(hardwareMap, telemetry);
        blocker = new Blocker();
        sweeper = new Sweeper();
        grabber = new GrabberArm();
        leftIntake = new Intake();
        rightIntake = new Intake();
        shooter = new Shooter();

//latch = new Latch(hardwareMap, telemetry);
        teamUtil.log("Constructing Assemblies - Finished");
        teamUtil.log("Constructed Robot - Finished");
    }

    // Call this before first use!
    public void init(boolean usingDistanceSensors) {
        teamUtil.log("Initializing Robot");
        drive.initDriveMotors();

        //leftIntake.init("conveyorServoLeft", "rollerServoLeft");
        //leftIntake.init("conveyorServoRight", "rollerServoRight");
        //blocker.init();
        //sweeper.init();
        //grabber.init();
        //shooter.init();

        // reset mechanisms if we did not just run auto
        if (!justRanAuto) {
            teamUtil.log("Resetting Robot");

            drive.initImu();
            drive.initSensors();
            //sweeper.reset();
            //grabber.reset();
        }
        drive.resetHeading();
        teamUtil.log("Initializing Robot - Finished");
    }

}


