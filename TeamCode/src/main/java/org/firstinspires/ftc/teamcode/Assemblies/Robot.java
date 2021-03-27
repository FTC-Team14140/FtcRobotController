package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.RingDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

// A class to encapsulate the entire 
// This class is designed to be used ONLY in a linearOpMode (for Auto OR Teleop)
public class Robot {
    Robot robot;
    int path;
    public boolean hasBeenInitialized = false;
    public static int justRanAuto;

    static {
        justRanAuto = 0;
    }

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Blocker blocker;
    RingDetector detector;
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
        drive = new RobotDrive();
        leftIntake = new Intake();
        rightIntake = new Intake();
        blocker = new Blocker();
        sweeper = new Sweeper();
        grabber = new GrabberArm();
        shooter = new Shooter();
        detector = new RingDetector(telemetry, hardwareMap);
        teamUtil.log("Constructing Assemblies - Finished");

        teamUtil.log("Constructed Robot - Finished");
    }

    // Call this before first use!
    public void init(boolean usingDistanceSensors) {
        teamUtil.log("Initializing Robot");
        drive.initDriveMotors();
        drive.initImu();

        leftIntake.init("leftPulleyServo", true,"leftRollerServo", false);
        rightIntake.init("rightPulleyServo", false,"rightRollerServo",true);
        blocker.init();
        sweeper.init();
        grabber.init();
        shooter.init();

        // reset mechanisms and initialize sensors if we did NOT just run auto
        if (justRanAuto==0) {
            teamUtil.log("Resetting Robot");


            sweeper.reset();
            grabber.reset();

        }else if(justRanAuto == 1){
            blocker.blockerExtended = true;

        }  else if(justRanAuto == 2){
            grabber.grab();
            grabber.liftToTeleopUp();



        } else if(justRanAuto == 3){
//            grabber.stow();
//            blocker.blockerExtended = true;

        }
        drive.initSensors(usingDistanceSensors, true);
        drive.calibrateColorSensors(); // Color sensors should be over non taped mat when this is called

        drive.resetHeading();
        teamUtil.log("Initializing Robot - Finished");
    }

    public void doAutoV3(int path){

        //start flywheel and drive forward a bit
        shooter.flywheelStart();


        if(path == 1){
            shooter.aimAt(Shooter.ShooterTarget.HIGH_GOAL);
            drive.moveInches(290, 70, 5000);

            // Launch 3 rings
            shooter.launchAndClear();
            teamUtil.pause(500); // let flywheel get back up to speed
            shooter.launchAndClear();
            teamUtil.pause(500); // let flywheel get back up to speed
            shooter.launchAndClear();
            shooter.stopFlywheel();

            // prepare to drop wobble goal
            grabber.liftToAutoDropNoWait();

            // position for drop
            drive.rotateTo(110);
            drive.moveInches(90, 10, 2000);
            grabber.release();

            // Move clear of dropped wobble goal
            teamUtil.pause(800);
            drive.moveInches(270, 10, 2000);

            // adjust heading and arm to get 2nd wobble goal
            drive.rotateTo(270);
            grabber.moveToReadyNoWait();

            // use sensors to position on 2nd goal
            drive.moveInches(197, 33, 6500, drive.DRIVE_MAX_MOVE_TO_DISTANCE_VELOCITY);
            grabber.moveToReadyNoWait();
            drive.moveToDistance(drive.backDistance, 205, 14, 4000);
            //drive.moveToLine(drive.backRightColor, teamColorSensor.TapeColor.RED, 90, 3000);
            drive.moveToLine(drive.backRightColor, teamColorSensor.TapeColor.RED, 90, drive.FIND_LINE_SPEED, 270, 3000);
            drive.moveInches(90, 1, 2000);

            // grab it
            grabber.grab();
            teamUtil.pause(750);
            grabber.liftToAutoDropNoWait();

            // move back to target and drop
            drive.moveInches(345, 64, 6000);
            drive.rotateTo(92);
            grabber.release();
            teamUtil.pause(750);

            // park and get ready to block rings
            drive.moveInches(270, 5, 3000);;
            drive.rotateTo(0);
            blocker.extendFully();


        } else if(path == 2){ //Assuming there is ONE ring on the field
            // get shooter ready to take a shot from position 1
            shooter.manualFlyWheelSpeed(shooter.FAR_SHOT_VELOCITY);
            shooter.manualTilt(shooter.AUTO_POS_1);

            // Move to position 1 and shoot
            drive.moveInches(270, 44, 3000);
            drive.rotateTo(7.5);
            shooter.launch();

            // get shooter ready for position 3 (normal shooting position)
            shooter.manualFlyWheelSpeed(shooter.HIGH_GOAL_VELOCITY);
            shooter.manualTilt(shooter.AUTO_POS_3);

            grabber.liftToAutoDropNoWait(); // get arm out of the way of intake

            drive.rotateTo(0);
            leftIntake.start();
            //move to grab last ring
            drive.moveInches(0, 14, 5000);

            //drive up to the white line and shoot 3 times
            drive.moveInches(270, 22, 3000);
            shooter.launchAndClear();
            teamUtil.pause(500); // let flywheel get back up to speed
            shooter.launchAndClear();
            teamUtil.pause(500); // let flywheel get back up to speed
            shooter.launchAndClear();
            //teamUtil.pause(500); // let flywheel get back up to speed
            //shooter.launchAndClear();
            shooter.stopFlywheel();
            leftIntake.stop();

            //Position to drop first wobble goal and drop
            drive.rotateTo(180);
            drive.moveInches(88, 25, 2000);
            grabber.release();
            drive.moveInches(268, 5, 2000);;

            // Position to grab second wobble goal
            drive.rotateTo(270);
            drive.moveInches(195, 48, 6500, drive.DRIVE_MAX_MOVE_TO_DISTANCE_VELOCITY);
            grabber.moveToReadyNoWait();
            drive.moveToDistance(drive.backDistance, 195, 14, 4000);
            //drive.moveToLine(drive.backRightColor, teamColorSensor.TapeColor.RED, 90, 3000);
            drive.moveToLine(drive.backRightColor, teamColorSensor.TapeColor.RED, 90, drive.FIND_LINE_SPEED, 270, 3000);
            drive.moveInches(90, 1, 2000);

            // grab 2nd wobble goal
            grabber.grab();
            teamUtil.pause(750);
            grabber.liftToAutoDropNoWait();

            drive.moveInches(16, 63, 6000);
            drive.rotateTo(180);
            grabber.release();
            drive.moveInches(270, 5, 5000);

        } else if(path == 3){ //Assuming there are FOUR rings on the field
            // get shooter ready to take a shot from position 1
            shooter.manualFlyWheelSpeed(shooter.FAR_SHOT_VELOCITY);
            shooter.manualTilt(shooter.AUTO_POS_1);

            // move to position 1
            drive.moveInches(270, 44, 3000);

            //rotate a little and shoot twice at high goal
            drive.rotateTo(7.5);
            shooter.launchAndClear();
            teamUtil.pause(500); // let flywheel get back up to speed
            shooter.launchAndClear();

            // get shooter ready to shoot from position 2
            shooter.manualTilt(shooter.AUTO_POS_2);
            drive.rotateTo(359);
            grabber.liftNoWait(); // get grabber out the way of intake
            leftIntake.start();
            //move to suck up more rings and shoot twice
            drive.moveInches(0, 10, 5000);
            drive.moveInches(0, 4, 5000);
//            drive.moveInches(180, 5, 5000); drive backwards commented out
            teamUtil.pause(2000); // let intake finish first ring
//            drive.rotateTo(357);
            shooter.launchAndClear();
            teamUtil.pause(500); // let flywheel get back up to speed
            shooter.launchAndClear();
//            teamUtil.pause(500); // let flywheel get back up to speed
//            shooter.launchAndClear();

            // get shooter ready for position 3 (normal shooting position)
            shooter.manualFlyWheelSpeed(shooter.HIGH_GOAL_VELOCITY);
            shooter.manualTilt(shooter.AUTO_POS_3);
            //move to suck up last two rings
            drive.moveInches(0, 5, 5000); //originally 10 when drive backwards was there
            drive.moveInches(0, 5, 5000);

            // drive to normal shooting position
            drive.moveInches(240, 24, 5000);
            //shoot 4 times(one extra to make sure it's not stuck)
            shooter.launchAndClear();
            shooter.launchAndClear();
            shooter.launchAndClear();
            shooter.launchAndClear();
            shooter.stopFlywheel();
            //move diagonally to drop off wobble goal
            drive.moveInches(250, 55, 7000);
            drive.rotateTo(135);
            grabber.release();
            drive.moveInches(300, 7, 5000);


            //move diagonally back to get to white line(and extend blocker)
//            drive.moveInches(300, 24, 4000, drive.DRIVE_MAX_VELOCITY);
//            blocker.extendNoWait();
//            drive.moveInches(300, 24, 5000);
//            drive.rotateTo(0);





//            drive.moveInches(315, 35, 7000, drive.FIND_LINE_SPEED); // go fast then slow down to find tape
//            drive.moveToLine(drive.frontRightColor, teamColorSensor.TapeColor.RED, 315, 7000);
//
//            drive.moveInches(145 , 22 - drive.frontDistance.getDistanceInches() , 3000);
//            shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
//            shooter.launch();

        }
        justRanAuto = path;
    }

    /*
    public void doAuto2(int rings){

        // Move straight down the right wall
        if(rings == 0 || rings == 1){
            drive.moveInches(180, 61, 9000); //67

        } else if(rings == 4){
            drive.moveInches(180, 109, 9000); //115
        }

        //line up off of right wall by 16 inches
        drive.moveToDistance(drive.leftDistance, 90 , 16, 3000);

        // for targets near wall, we can just drop and move to shooting position
        if(rings == 0 || rings == 4){
            grabber.lift();
            grabber.release();

            if(rings == 0){
                //move diaganoally backwards
                drive.moveInches(315, 18, 4000);

            } else if(rings == 4){
                //move diagaonally backwards
                drive.moveInches(340, 62, 4000);
            }

        } else if(rings == 1){ // spin and position for drop on target B

            drive.rotateTo(115);
            drive.moveInches(90, 10, 2000);
            grabber.lift();
            grabber.release();
            drive.moveInches(235, 21, 4000);

        }

        //ALL CODE PROCEEDS AS SAME FROM HERE
        grabber.stowNoWait();
        shooter.flywheelStart();
        drive.rotateTo(270);

        drive.moveInches(0, 15.5, 4000);

        //start shooting for powershots

        shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
        while (!shooter.flywheelReady()) {

        }
        shooter.launch();
        drive.moveInches(0, 7.5, 7000);
        shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
        while (!shooter.flywheelReady()) {

        }
        shooter.launch();
        drive.moveInches(0, 7.5, 7000);
        shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
        while (!shooter.flywheelReady()) {

        }
        shooter.launch();
        shooter.stopFlywheel();

        // Get ready for Teleop and park on line
        blocker.extendNoWait();
        drive.moveInches(200, 32, 4000);

//         record that Auto has been run so we don't reinitialize certain sensors and calibrate motor encoders
        justRanAuto = true;

    }

    public void doAuto(int rings) {
        // TODO: Do a bunch of stuff and get a bunch of points
        //The robot starts touching the wall on the closest to the right side wall if you are facing the goals or the closest line to where you should stand
        //Move robot off the wall
        drive.moveInches(0, 4, 7000);

        //Move robot right to 4 inches off the right wall
        drive.moveToDistance(drive.rightDistance, 270, 4, 7000);
        drive.stopDrive();

        //Robot drive forward for 10 inches need to find the correct end speed
        drive.moveInches(0 , 15, 7000, 1000);

        //Robot drive forward to the first red tape line then stop
        drive.moveToLine(drive.frontRightColor, teamColorSensor.TapeColor.RED, 0, 7000);
        drive.stopDrive();
        //Turn the robot so the wobble goal grabber is facing forward and code to drop the wobble goal
        drive.rotateTo(270);
        grabber.placeAndRelease();

        //Move robot backwards from the wobble goal and stow the wobble goal grabber
        drive.moveInches(180, 4, 7000);
        grabber.stowNoWait();

        //Rotate robot with shooter facing the front of the field (goal)
        drive.rotateTo(90);

        //Move the correct number of inches to the left of the field to line up with the first power shot
        drive.moveInches(0, 30, 7000);  // TODO: Find the exact distance from dropping off the wobble goal to the power shot

        //Move robot to the white line with the shooter facing forwards
        drive.moveToLine(drive.frontRightColor, teamColorSensor.TapeColor.WHITE, 270, 7000);
        drive.stopDrive();

        //Move robot backwards off of the white line
        drive.moveInches(90, 2, 7000);

        //Aims the shooter starts the flywheel and launch one ring at the power shot then moving to next powershot etc,
        shooter.flywheelStart();
        shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
        while (!shooter.flywheelReady()) {

        }
        shooter.launch();
        drive.moveInches(0, 7.5, 7000);
        shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
        while (!shooter.flywheelReady()) {

        }
        shooter.launch();
         drive.moveInches(0, 7.5, 7000);
        shooter.aimAt(Shooter.ShooterTarget.POWERSHOT);
        while (!shooter.flywheelReady()) {

        }
        shooter.launch();
        shooter.stopFlywheel();

        //Drives the robot to park on the white line
        drive.moveInches(270, 4, 7000);

        //TODO: NEW STUFF HERE 2/26
        drive.moveInches(0, 5, 7000);
        grabber.moveToReadyNoWait();


        // record that Auto has been run so we don't reinitialize certain sensors and calibrate motor encoders
        justRanAuto = true;
    }
     */
}


