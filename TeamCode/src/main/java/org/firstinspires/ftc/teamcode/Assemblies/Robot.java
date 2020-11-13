package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.runningVoteCount;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

// A class to encapsulate the entire 
// This class is designed to be used ONLY in a linearOpMode (for Auto OR Teleop)
public class Robot {
    public static final double DISTANCE_TO_BLOCK = 1.125;
    public static final int MIN_DISTANCE_FOR_AUTO_DROPOFF = 6;
    public final double DISTANCE_TO_FOUNDATION = 2.5;
    public static final double AUTOINTAKE_POWER = 0.33;
    public static final double AUTOINTAKE_SIDEWAYS_POWER = 0.33;
    int path;
    public boolean hasBeenInitialized = false;
    private static boolean justRanAuto;

    static {
        justRanAuto = false;
    }

    public LiftSystem liftSystem;
    public RobotDrive drive;
    public Latch latch;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    SkystoneDetector detector;
    boolean timedOut = false;

    public final int MIN_DISTANCE_FOR_AUTO_PICKUP = 0;
    public final int MAX_DISTANCE_FOR_AUTO_DROPOFF = 6;

    public Robot(LinearOpMode opMode) {
        teamUtil.log("Constructing Robot");
        // stash some context for later
        teamUtil.theOpMode = opMode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        teamUtil.log("Constructing Assemblies");
        drive = new RobotDrive(hardwareMap, telemetry);
        //liftSystem = new LiftSystem(hardwareMap, telemetry);
        //latch = new Latch(hardwareMap, telemetry);
        teamUtil.log("Constructing Assemblies - Finished");
        teamUtil.log("Constructed Robot - Finished");
    }

    // Call this before first use!
    public void init(boolean usingDistanceSensors) {
        teamUtil.log("Initializing Robot");
        //liftSystem.initLiftSystem(!justRanAuto);
        justRanAuto = false;
        drive.initImu();
        drive.initDriveMotors();
        if (usingDistanceSensors) {
            drive.initSensors(false);
        } else {
            drive.initSensors(true);
        }
        drive.resetHeading();
        //latch.initLatch();
        teamUtil.log("Initializing Robot - Finished");
    }

}


