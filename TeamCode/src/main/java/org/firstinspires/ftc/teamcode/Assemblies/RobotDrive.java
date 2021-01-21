package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;
import org.firstinspires.ftc.teamcode.basicLibs.teamDistanceSensor;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RobotDrive - a Class to encapsulate the drive system along with the sensors that are used to control it
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

public class RobotDrive {

    // Tuning Constants for drive movements
    private double COUNTS_PER_INCH = 45;  // Calibrated for the Ultimate Goal Robot
    private double COUNTS_PER_INCH_SIDEWAYS = 50.625;  // Calibrated for the Ultimate Goal Robot  TODO: See if we can find a trig formula to make inches more accurate at all headings

    double LastEndSpeed = 0;  //Keep track of the end velocity of the last movement so we can start from there on the next
    public double DRIVE_MAX_VELOCITY = 2300; // tics/sec, for forward and backward
    public double DRIVE_MAX_STRAFE_VELOCITY = 1900; //tics/sec, for left and right

    double ROTATION_ADJUST_FACTOR = 0.1; // TODO: verify this is a reasonable P coefficient for rotational error adjustment

    public double MIN_START_SPEED = 300; // MIN power to get the robot to start moving
    public double MAX_ACCEL_PER_INCH = 250; // max velocity acceleration per inch without skidding
    public double MAX_DECEL_PER_INCH = 55; // max power deceleration per inch without skidding
    public double MIN_END_SPEED = 250; // Power to decelerate to before stopping completely

    // Tuning constants for Spins
    public double DRIVE_MAX_SPIN_VELOCITY = DRIVE_MAX_VELOCITY; // max velocity for spins
    public double DRIVE_SLOW_SPIN_VELOCITY = 250; // slowest velocity for spins
    public double SPIN_DECEL_THRESHOLD = 50; // start deceleration this many degrees from the target
    public double SPIN_SLOW_THRESHOLD = 13; // slow down to a very slow turn this far from the target
    public double SPIN_DRIFT_DEGREES = 1.5; // cut the motors completely when we are within this many degrees of the target to allow for a little drift

    // Tuning constants for MoveToInches
    public double DRIVE_MAX_MOVE_TO_DISTANCE_VELOCITY = DRIVE_MAX_VELOCITY/2;
    public double MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD = 1; // No movement if we are within this distance of target
    public double MOVE_TO_DISTANCE_DRIFT_DISTANCE = .5; // Cut motors at this distance from target to allow for drift
    public double MOVE_TO_DISTANCE_SLOW_DISTANCE = 4; // Drop motors to a slow crawl at this distance from target
    public double MOVE_TO_DISTANCE_DECEL_DISTANCE = 10; // Distance for decleration before reaching SLOW_DISTANCE

    public double FIND_LINE_SPEED = 700; // Speed for looking for lines

    public static double HEADING_OFFSET; // offset between IMU heading and field

    HardwareMap hardwareMap;
    boolean timedOut = false;

    // Hardware
    revHubIMUGyro revImu;

    DcMotorEx fLeftMotor;
    DcMotorEx bLeftMotor;
    DcMotorEx fRightMotor;
    DcMotorEx bRightMotor;

    public boolean distanceSensorsOnline = false; // True if distance sensors initialized and online
    public teamDistanceSensor frontDistance;
    public teamDistanceSensor rightDistance;
    public teamColorSensor frontLeftColor;
    public teamColorSensor frontRightColor;



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Constructors, Initialization
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public RobotDrive() {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initDriveMotors() {
        fLeftMotor = hardwareMap.get(DcMotorEx.class, "fLeftMotor");
        fRightMotor = hardwareMap.get(DcMotorEx.class, "fRightMotor");
        bLeftMotor = hardwareMap.get(DcMotorEx.class, "bLeftMotor");
        bRightMotor = hardwareMap.get(DcMotorEx.class, "bRightMotor");
        fLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //fLeftMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);   // These coeffiecients were found using the technique in this doc: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#
        //fRightMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);  // This was based on the SkyStone Robot.  these are NOT the defaults for these motors
        //bLeftMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);
        //bRightMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);
        setAllMotorsWithEncoder();
        setBrakeAllDriveMotors();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initSensors(boolean usingDistanceSensors) {

        if (usingDistanceSensors) {
            frontDistance = new teamDistanceSensor((Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "frontDistance"),1.4f,20,.5);
            distanceSensorsOnline = true;
        }
        frontLeftColor = new teamColorSensor(hardwareMap.get(ColorSensor.class, "flColor"));
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void calibrateColorSensors() {
        frontLeftColor.calibrate();
    }








    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Telemetry
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void distanceTelemetry() {
        if (distanceSensorsOnline) {
            teamUtil.telemetry.addData("Distance F:", "%.1f %b", frontDistance.getDistanceInches(),frontDistance.lastReadingValid);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void colorTelemetry() {
        teamUtil.telemetry.addData("fl: ", frontLeftColor.isOnTape());
    }

    public void rawColorTelemetry() {
        teamUtil.telemetry.addData("FL ","R:%d B:%d A:%.1f", frontLeftColor.redValue(), frontLeftColor.blueValue(), frontLeftColor.getAlpha());
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void telemetryDriveEncoders() {
        teamUtil.telemetry.addData("front left:", fLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("front right:", fRightMotor.getCurrentPosition());

        teamUtil.telemetry.addData("back left:", bLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("back right:", bRightMotor.getCurrentPosition());
    }







    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Low level motor/encoder helper functions
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setAllMotorsWithoutEncoder() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setAllMotorsWithEncoder() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setAllMotorsRunToPosition() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setBrakeAllDriveMotors() {
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void resetAllDriveEncoders() {
        fLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Update the drive motor velocities
    // TODO: Use the Batch data feature (see examples) to speed this up
    public void setDriveVelocities(double flV, double frV, double blV, double brV){
        fLeftMotor.setVelocity(flV);
        fRightMotor.setVelocity(frV);
        bLeftMotor.setVelocity(blV);
        bRightMotor.setVelocity(brV);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopDrive() {
        setDriveVelocities(0,0,0,0);
        teamUtil.log("STOP Motors");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int inchesToEncoderTics(double inches) { // return the # of encoder tics for the specified inches
        return (int) (inches * COUNTS_PER_INCH);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the velocity of all 4 motors based on a driveHeading relative to robot and provided velocity
    // Corrects for SMALL amounts of rotational drift based on robotHeading
    // The presumption is that the current heading will be close to the robotHeading.  If that is not true, the
    // robot will go on a loopy drive...
    public void driveMotorsHeadings(double driveHeading, double robotHeading, double velocity) {
        double flV, frV, blV, brV;
        double x, y, scale;

        // Determine how much adjustment for rotational drift
        double headingError = Math.max(-10.0, Math.min(getHeadingError(robotHeading), 10.0)); // clip this to 10 degrees in either direction
        double rotationAdjust = headingError * ROTATION_ADJUST_FACTOR * velocity;

        // Covert heading to cartesian on the unit circle and scale so largest value is 1
        // This is essentially creating joystick values from the heading
        x = Math.cos(Math.toRadians(driveHeading + 90)); // + 90 cause forward is 0...
        y = Math.sin(Math.toRadians(driveHeading + 90));
        scale = 1 / Math.max(Math.abs(x), Math.abs(y));
        x = x * scale;
        y = y * scale;

        // Clip to motor power range
        flV = Math.max(-1.0, Math.min(x + y, 1.0))*velocity;
        brV = flV;
        frV = Math.max(-1.0, Math.min(y - x, 1.0))*velocity;
        blV = frV;

        // Adjust for rotational drift
        flV = flV - rotationAdjust;
        brV = brV + rotationAdjust;
        frV = frV + rotationAdjust;
        blV = blV - rotationAdjust;

        // Update the motors
        setDriveVelocities(flV, frV, blV, brV);
    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Low level IMU and Angle related methods
    // TODO: the team IMU class needs to be rewritten to hold all the methods below
    // WARNING!! Our IMUs are currently mounted upside down.  So counter clockwise is positive.  An adjustment for that could be added to
    // the IMU class.  meanwhile, all the code below presumes an upside down rev Hub...

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Get access to the IMU from the hardware map
    public void initImu() {
        revImu = new revHubIMUGyro(hardwareMap, teamUtil.telemetry);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // adjust the given angle to be in the range 0-360.
    public double adjustAngle(double angle) {
        //assuming imu runs from [0, 360] and angle is added/substracted, adjust it to expected reading
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculate the min degrees between two points on a circle
    // Assumes both degree measurements are 0-360
    public double minDegreeDiff(double a, double b) {
        if (Math.abs(a - b) <= 180) {
            return Math.abs(a - b);
        } else {
            if (a < b)
                return (360 - b) + a;
            else
                return (360 - a) + b;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Make the current heading 0.
    public void resetHeading() {
        HEADING_OFFSET = revImu.getHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return our current heading as a 0 to 360 range.
    public double getHeading() {
        return adjustAngle(revImu.getHeading() - HEADING_OFFSET);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return the internal IMU heading without our offsets or fixes
    public double getAbsoluteHeading() {
        return revImu.getAbsoluteHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return the difference between the current heading and a target heading.  Returns -180 to 180
    // Positive number means we need to turn left
    public double getHeadingError(double targetAngle) {

        double robotError;

        // calculate heading error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Universal Move Inches  NEW for Ultimate Goal!
    // These methods implement a universal moveInches that takes a heading in degrees and attempts to move
    // a specified number of inches in that direction.  Distance is based on encoder readings, so acceleration
    // slopes are used at the start and end of movements to try to avoid wheel slippage.
    // A non-zero end velocity can be used in which case the robot will remain moving when the method returns.
    // This is intended to allow for a sequence of moves or a fast transition into a sensor driven movement.

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Return the number of encoder tics needed to ramp from start speed to end speed at the specified acceleration
    public int rampEncoderCount(double startSpeed, double endSpeed, double acceleration) {
        return Math.abs(inchesToEncoderTics((endSpeed - startSpeed) / acceleration));
    }

    class MotorData { // a helper class to allow for faster access to hub data
        int eFL, eFR, eBL, eBR;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Return the encoder positions for all 4 drive motors
    // TODO: Use the Batch data feature (see examples) to speed this up
    public void getDriveMotorData(MotorData data) {
        data.eFL = fLeftMotor.getCurrentPosition();
        data.eFR = fRightMotor.getCurrentPosition();
        data.eBL = bLeftMotor.getCurrentPosition();
        data.eBR = bRightMotor.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculate the distance the robot has moved from the specified initialPosition
    public int getEncoderDistance(MotorData initialPositions) {
        MotorData currentPositions = new MotorData();
        getDriveMotorData(currentPositions);

        // Calculate the vector along the forward/backward axis
        int ForwardVector = (currentPositions.eFL-initialPositions.eFL)
                + (currentPositions.eFR-initialPositions.eFR)
                + (currentPositions.eBL-initialPositions.eBL)
                + (currentPositions.eBR-initialPositions.eBR);
        // Calculate the vector along the left/right axis
        int SideVector = (currentPositions.eFL-initialPositions.eFL)
                + (currentPositions.eBR-initialPositions.eBR)
                - ((currentPositions.eFR-initialPositions.eFR)
                   + (currentPositions.eBL-initialPositions.eBL));
        // Return the hypotenuse of the two vectors
        // divide by 4 to account for the math that adds all 4 motor encoders
        return (int) (Math.sqrt(Math.pow(ForwardVector,2)+Math.pow(SideVector,2)) / 4);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drive the motors towards the specified heading, holding the robotHeading, for the specified encodercount
    // Ramp velocity from startVelocity to endVelocity linearly
    public void driveMotorsForEncoderCount(double driveHeading, double robotHeading, double encoderCount, double startVelocity, double endVelocity, long timeOut) {
        boolean details = false;
        teamUtil.log("driveMotors: heading:" + driveHeading + " encoders:" + encoderCount + " StartSpeed:" + startVelocity + " EndSpeed:" + endVelocity);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        // Figure out the distance we need to ramp in encoder tics
        double speedChange = endVelocity - startVelocity; // + if accelerating or - if decelerating
        double slope = speedChange / encoderCount; // slope for the velocity ramp.  + or -

        // Get the encoder positions at the start of the movement
        MotorData initialPositions = new MotorData();
        getDriveMotorData(initialPositions);
        int encoderTicsTraveled = 0;

        while (encoderTicsTraveled < encoderCount && teamUtil.keepGoing(timeOutTime)) {

            // constantly adjust the motor speeds to ramp velocity and correct rotational drift
            driveMotorsHeadings(driveHeading, robotHeading, startVelocity + slope*encoderTicsTraveled);
            if (details)
                teamUtil.log("Distance Traveled: " + encoderTicsTraveled);
            encoderTicsTraveled = getEncoderDistance(initialPositions);
        }

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("driveMotors - TIMED OUT!");
        }
        teamUtil.log("driveMotors - Finished");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Move in any direction a specified number of inches using acceleration curves at start and end
    // Supports non-zero start and end speeds (start speed is presumed to be the last end speed.)
    public void moveInches(double driveHeading, double inches, long timeOut) {
        moveInches(driveHeading, inches, timeOut, DRIVE_MAX_VELOCITY, 0, getHeading());
    }
    public void moveInches(double driveHeading, double inches, long timeOut, double endSpeed) {
        moveInches(driveHeading, inches, timeOut, DRIVE_MAX_VELOCITY, endSpeed, getHeading());
    }
    public void moveInches(double driveHeading, double inches, long timeOut, double endSpeed , double robotHeading) {
        moveInches(driveHeading, inches, timeOut, DRIVE_MAX_VELOCITY, endSpeed, robotHeading);
    }
    public void moveInches(double driveHeading, double inches, long timeOut, double maxVelocity, double endSpeed, double robotHeading) {
        boolean details = false;
        boolean cruisePhase = true;
        boolean stopAtEnd = false;
        teamUtil.log("moveInches: driveHeading:" + driveHeading + " Inches:" + inches + " Velocity:" + maxVelocity + " endSpeed:" + endSpeed + " robotHeading:" + robotHeading);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        double startSpeed = LastEndSpeed;
        if (startSpeed < MIN_START_SPEED) {
            startSpeed = MIN_START_SPEED;
        }
        LastEndSpeed = endSpeed; // save this for next time this method is called
        if (endSpeed == 0) {  // in case they are trying to stop the robot, fix it so we don't stall
            endSpeed = MIN_END_SPEED;
            stopAtEnd = true;
        }

        // Figure out the distances for each phase in encoder tics.  These are all + numbers
        int totalEncoderCount = inchesToEncoderTics(inches);
        int accelerationEncoderCount = rampEncoderCount(startSpeed, maxVelocity, MAX_ACCEL_PER_INCH);
        int decelerationEncoderCount = rampEncoderCount(maxVelocity, endSpeed, MAX_DECEL_PER_INCH);

        // figure out slopes for acceleration and deceleration phases.
        // These are fixed for now, but could vary based on heading given the mecanum wheels
        double accelerationSlope = MAX_ACCEL_PER_INCH / COUNTS_PER_INCH;  // positive slope
        double decelerationSlope = MAX_DECEL_PER_INCH / COUNTS_PER_INCH * -1; // Negative slope

        int target = totalEncoderCount;
        int cruiseStart, decelerationStart;
        if (accelerationEncoderCount + decelerationEncoderCount < totalEncoderCount) {
            // Enough distance to reach maxVelocity
            cruiseStart = accelerationEncoderCount;
            decelerationStart = target - decelerationEncoderCount;
        } else {
            // we don't have enough space to ramp up to full speed so calculate the actual maximum velocity
            // by finding the y value of the two ramp lines where they intersect given the maximum distance
            maxVelocity = (accelerationSlope * endSpeed -
                    decelerationSlope*startSpeed -
                    accelerationSlope * decelerationSlope * totalEncoderCount) /
                    (accelerationSlope-decelerationSlope);
            cruisePhase = false;
            teamUtil.log("Limited Max Velocity to:" + maxVelocity);

            // recompute shortened ramp phases
            accelerationEncoderCount = rampEncoderCount(startSpeed, maxVelocity, MAX_ACCEL_PER_INCH);
            decelerationEncoderCount = rampEncoderCount(maxVelocity, endSpeed, MAX_DECEL_PER_INCH);
            cruiseStart = accelerationEncoderCount;
            decelerationStart = target - decelerationEncoderCount;
        }
        teamUtil.log("accelerationSlope:" + accelerationSlope + " decelerationSlope:" + decelerationSlope + " totalEncoderCount:" + totalEncoderCount);
        teamUtil.log("accelerationEncoderCount:" + accelerationEncoderCount + " decelerationEncoderCount:" + decelerationEncoderCount + " decelerationStart:" + decelerationStart + " totalEncoderCount:" + totalEncoderCount);

        // ramp up if needed
        if (accelerationEncoderCount > 0) {
            driveMotorsForEncoderCount(driveHeading, robotHeading,accelerationEncoderCount,startSpeed,maxVelocity, timeOut);
        }

        // Cruise at Max Velocity if we have a cruise phase
        if (cruisePhase) {
            driveMotorsForEncoderCount(driveHeading, robotHeading, decelerationStart - cruiseStart, maxVelocity, maxVelocity, timeOutTime - System.currentTimeMillis());
        }

        // ramp down if needed
        if (decelerationEncoderCount > 0) {
            driveMotorsForEncoderCount(driveHeading, robotHeading, totalEncoderCount - decelerationStart, maxVelocity, endSpeed, timeOutTime - System.currentTimeMillis());
        }

        if (stopAtEnd) {  // stop the motors if they intend the robot to hold still
            stopDrive();
        }

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("moveInches - TIMED OUT!");
        }
        teamUtil.log("moveInches - Finished");

    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Movement methods using Distance Sensors
    // Uses a 3 stage deceleration path and the IMU to hold the given heading.
    // robot will move forward or backward on the specified driveHeading to gain the target distance reading on the supplied Distance Sensor
    public void moveToDistance(teamDistanceSensor sensor, double driveHeading, double inches, long timeOut) {
        moveToDistance(sensor, driveHeading, inches, LastEndSpeed, getHeading(), 0, timeOut);
    }
    public void moveToDistance(teamDistanceSensor sensor, double driveHeading, double inches, double startSpeed, double robotHeading, double endSpeed, long timeOut) {
        boolean details = false;
        boolean stopAtEnd = false;

        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        if (startSpeed == 0) { //No start speed specified so use full power
            startSpeed = DRIVE_MAX_MOVE_TO_DISTANCE_VELOCITY;
        } else if (startSpeed < MIN_START_SPEED) { // Make sure we will get the robot moving
            startSpeed = MIN_START_SPEED;
        }
        LastEndSpeed = endSpeed; // save this for next time this method is called
        if (endSpeed == 0) {  // in case they are trying to stop the robot, fix it so we don't stall
            endSpeed = MIN_END_SPEED;
            stopAtEnd = true;
        }

        teamUtil.log("moveToDistance: driveHeading:" + driveHeading + " inches:" + inches + " Speed:" + startSpeed + " endSpeed:" + endSpeed + " robotHeading:" + robotHeading);

        double velocity = startSpeed;
        double currentDistance = sensor.getDistanceInches(); // Distance to object

        teamUtil.log("Initial Wall Distance Reading: " + currentDistance);

        // if we are already close enough, leave the robot where it is
        if (Math.abs(currentDistance - inches) <= MOVE_TO_DISTANCE_NO_MOVEMENT_THRESHOLD) { // was 1
            teamUtil.log("Already There!  Not Moving...");
            return;

        } else if (currentDistance < inches) { // flip heading to move backwards
            driveHeading = adjustAngle(driveHeading + 180);
            teamUtil.log("Moving Backwards");
        } else {
            teamUtil.log("Moving Forwards");
        }

        final double preDriftTarget = MOVE_TO_DISTANCE_DRIFT_DISTANCE; // was .5
        final double slowThreshold = MOVE_TO_DISTANCE_SLOW_DISTANCE; // was 5
        final double decelThreshold = MOVE_TO_DISTANCE_SLOW_DISTANCE + MOVE_TO_DISTANCE_DECEL_DISTANCE; // was 10
        final double slope = (startSpeed - endSpeed) / (decelThreshold - slowThreshold); //  slope for the decel phase
        teamUtil.log("preDriftTarget: " + preDriftTarget + " slowThreshold: " + slowThreshold + " decelThreshold: " + decelThreshold + " slope: " + slope);
            // Cruise at max speed
            while ((currentDistance = Math.abs(sensor.getDistanceInches() -inches)) > decelThreshold && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadings(driveHeading, robotHeading, startSpeed);
                if (details) teamUtil.log("CRUISING: TargetDistance:" + currentDistance + " velocity: " + startSpeed);
            }
            // Decelerate to min speed
            while ((currentDistance = Math.abs(sensor.getDistanceInches() -inches)) > slowThreshold && teamUtil.keepGoing(timeOutTime)) {
                velocity = Math.min((currentDistance - slowThreshold) * slope + endSpeed, startSpeed); // decelerate proportionally down to min - don't exceed initial speed
                driveMotorsHeadings(driveHeading, robotHeading, velocity);
                if (details) teamUtil.log("SLOWING: TargetDistance:" + currentDistance + " velocity: " + velocity);
            }
            // cruise at minSpeed once we are very close to target
            while ((currentDistance = Math.abs(sensor.getDistanceInches() -inches)) > preDriftTarget && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadings(driveHeading, robotHeading, endSpeed);
                if (details) teamUtil.log("CRAWLING: TargetDistance:" + currentDistance + " velocity: " + endSpeed);
            }
        if (stopAtEnd) {  // stop the motors if they intend the robot to hold still
            stopDrive();
        }
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("moveToDistance - TIMED OUT!");
        }
        teamUtil.log("Finished moveToDistance");
    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Movement methods using Color Sensors
    public void moveToLine(teamColorSensor sensor, teamColorSensor.TapeColor color, double driveHeading, long timeout){
        moveToLine(sensor, color, driveHeading, LastEndSpeed, getHeading(), timeout);
    }
    public void moveToLine(teamColorSensor sensor, teamColorSensor.TapeColor color, double driveHeading, double startSpeed, double robotHeading, long timeout) {
        boolean details = false;
        boolean stopAtEnd = false;

        long timeOutTime = System.currentTimeMillis() + timeout;
        timedOut = false;

        if (startSpeed == 0) { //No start speed specified so use line seeking speed
            startSpeed = FIND_LINE_SPEED;
        } else if (startSpeed < MIN_START_SPEED) { // Make sure we will get the robot moving
            startSpeed = MIN_START_SPEED;
        }
        LastEndSpeed = startSpeed; // save this for next time a movement method is called

        teamUtil.log("moveToLine: Color: " + color + "driveHeading:" + driveHeading + " startSpeed:" + startSpeed + " robotHeading:" + robotHeading);

        // find line and stop
        teamColorSensor.TapeColor currentColor;
        while (((currentColor = sensor.getColor()) != color) && teamUtil.keepGoing(timeOutTime)) {
            driveMotorsHeadings(driveHeading, robotHeading, startSpeed);
            if (details) teamUtil.log("SEARCHING: Color:" + currentColor + " velocity: " + startSpeed);
        }

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("moveToDistance - TIMED OUT!");
        }
        if (stopAtEnd || timedOut) {  // stop the motors if they intend the robot to hold still
            stopDrive();
        }
        teamUtil.log("Finished moveToLine");

    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods for rotating the robot

    public enum RobotRotation {GOAL, START, OUR_SIDE, THEIR_SIDE}

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate to the desired field relative heading at the maximum speed
    public void rotateTo(RobotRotation attitude) {
        switch (attitude) {
            case GOAL:
                rotateTo(0.0);
                break;
            case START:
                rotateTo(180.0);
                break;
            case OUR_SIDE:
                if (teamUtil.alliance == teamUtil.Alliance.RED) {
                    rotateTo(270);
                } else {
                    rotateTo(90);
                }
                break;
            case THEIR_SIDE:
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    rotateTo(270);
                } else {
                    rotateTo(90);
                }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate to the desired heading at the maximum speed, slowing for accuracy at the end
    public void rotateTo(double heading) {
        teamUtil.log("Rotate To: " + heading);
        boolean details = true;

        final double decelSlope = (DRIVE_MAX_SPIN_VELOCITY - DRIVE_SLOW_SPIN_VELOCITY) / (SPIN_DECEL_THRESHOLD - SPIN_SLOW_THRESHOLD); // + slope
        double leftRotatePower = 1; // Keep track of which way we are rotating
        double rightRotatePower = 1;
        double rotatePower = DRIVE_MAX_SPIN_VELOCITY; // start at full power

        double currentHeading = getHeading();
        double initialHeading = currentHeading; // Stash this so we can make this a "relative" turn from a heading of 0.

        // Determine how many degrees we need to turn from our current position to get to the target
        double turnDegrees = minDegreeDiff(heading, currentHeading); // always +
        turnDegrees = turnDegrees - SPIN_DRIFT_DEGREES; // stop early to allow for drift

        // Determine which we we are spinning (take the short way around)
        if (currentHeading < heading) {
            if (heading - currentHeading < 180) {
                leftRotatePower = -1;
            } else {
                rightRotatePower = -1;
            }
        } else {
            if (currentHeading - heading < 180) {
                rightRotatePower = -1;
            } else {
                leftRotatePower = -1;
            }
        }
        teamUtil.log("Turn Degrees: " + turnDegrees + " LeftPower: " + leftRotatePower);
        teamUtil.log("Initial Heading: " + initialHeading + " current Heading: " + currentHeading);

        // Number of degrees we have turned (in either direction) since we started
        currentHeading = minDegreeDiff(getHeading(), initialHeading); // always +

        // If there is a Full power phase: Rotate at max power until we get to deceleration phase
        if ((currentHeading = minDegreeDiff(getHeading(), initialHeading)) <  turnDegrees - SPIN_DECEL_THRESHOLD) {
            setDriveVelocities(DRIVE_MAX_SPIN_VELOCITY * leftRotatePower, DRIVE_MAX_SPIN_VELOCITY * rightRotatePower, DRIVE_MAX_SPIN_VELOCITY * leftRotatePower, DRIVE_MAX_SPIN_VELOCITY * rightRotatePower);
            while (currentHeading < turnDegrees - SPIN_DECEL_THRESHOLD) {
                if (details)
                    teamUtil.log("MAX: Relative Heading:" + currentHeading + " DifferenceInAngle: " + (turnDegrees - currentHeading) + " RotatePower: " + DRIVE_MAX_SPIN_VELOCITY);
                currentHeading = minDegreeDiff(getHeading(), initialHeading); // always +
            }
        }

        // If we have a deceleration phase: rotate at decelerating power as we close to target
        while (currentHeading < turnDegrees - SPIN_SLOW_THRESHOLD) {
            rotatePower = (turnDegrees - SPIN_SLOW_THRESHOLD - currentHeading) * decelSlope + DRIVE_SLOW_SPIN_VELOCITY; // decelerate proportionally down to min
            setDriveVelocities(rotatePower * leftRotatePower, rotatePower * rightRotatePower, rotatePower * leftRotatePower, rotatePower * rightRotatePower);
            if (details) teamUtil.log("DECEL: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + rotatePower);
            currentHeading = minDegreeDiff(getHeading(), initialHeading); // always +
        }

        // Always have a 'crawl' phase where we rotate at minSpeed once we are very close to target
        setDriveVelocities(DRIVE_SLOW_SPIN_VELOCITY * leftRotatePower, DRIVE_SLOW_SPIN_VELOCITY * rightRotatePower, DRIVE_SLOW_SPIN_VELOCITY * leftRotatePower, DRIVE_SLOW_SPIN_VELOCITY * rightRotatePower);
        while (currentHeading < turnDegrees) {
            if (details) teamUtil.log("CRAWL: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + DRIVE_SLOW_SPIN_VELOCITY);
            currentHeading = minDegreeDiff(getHeading(), initialHeading); // always +
        }
        setDriveVelocities(0,0,0,0);
        teamUtil.log("Finished Turning.  Heading: " + getHeading());
    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Joystick drive methods
    // currently these methods are NOT using the "heldHeading" parameter to try and hold a heading... TODO: Fix this!
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void universalJoystick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, double robotHeading, double heldHeading) {
        double angleInRadians = robotHeading * Math.PI / 180; // TODO: Isn't this converting TO Radians resulting in an 'angleInRadians'?
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        //rotate to obtain new coordinates
        float rotatedLeftX = (float) (Math.cos(angleInRadians) * leftX - Math.sin(angleInRadians) * leftY);
        float rotatedLeftY = (float) (Math.sin(angleInRadians) * leftX + Math.cos(angleInRadians) * leftY);

        driveJoyStick(rotatedLeftX, rotatedLeftY, rightX, isFast, heldHeading);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveJoyStick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, double heldHeading) {

        float leftX ;
        float leftY ;
        float rotationAdjustment;

        int scaleAmount = isFast ? 2 : 1;

        if ((leftJoyStickX > -0.1 && leftJoyStickX < 0.1)) {
            leftX = 0;
        } else if (leftJoyStickX >= 0.1) {
            leftX = (leftJoyStickX / 3 + 1 / 6) * scaleAmount;
        } else {
            leftX = (leftJoyStickX / 3 - 1 / 6) * scaleAmount;
        }

        if ((leftJoyStickY > -0.1 && leftJoyStickY < 0.1)) {
            leftY = 0;
        } else if (leftJoyStickY >= 0.1) {
            leftY = (leftJoyStickY / 3 + 1 / 6) * scaleAmount;
        } else {
            leftY = (leftJoyStickY / 3 - 1 / 6) * scaleAmount;
        }
        rotationAdjustment = (float) (rightJoyStickX * 0.37 * scaleAmount);

        float frontLeft = -(leftY - leftX - rotationAdjustment);
        float frontRight = (-leftY - leftX - rotationAdjustment);
        float backRight = (-leftY + leftX - rotationAdjustment);
        float backLeft = -(leftY + leftX - rotationAdjustment);

//        teamUtil.telemetry.addData("RIGHTX:", rightX);
//        teamUtil.telemetry.addData("LEFTX:", leftX);
//        teamUtil.telemetry.addData("LEFTY:", leftY);
//
//        teamUtil.telemetry.addData("joystickX:", leftJoyStickX);
//        teamUtil.telemetry.addData("joystickY:", leftJoyStickY);

        fLeftMotor.setPower(frontLeft);
        fRightMotor.setPower(frontRight * 1); // 1 was .9
        bRightMotor.setPower(backRight * 1);  // 1 was .9
        bLeftMotor.setPower(backLeft);

    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Code for finding operating parameters
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void findMaxForwardSpeed() {
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_INCH * 60;
        setDriveVelocities(3000, 3000, 3000, 3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        while (fLeftMotor.getCurrentPosition() < travelTics) {
            flmax = (v = fLeftMotor.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fRightMotor.getVelocity()) > frmax ? v: frmax;
            blmax = (v = bLeftMotor.getVelocity()) > blmax ? v : blmax;
            brmax = (v = bRightMotor.getVelocity()) > brmax ? v : brmax;
        }
        stopDrive();
        teamUtil.log("Forward Max Velocities FL:"+flmax+" FR:"+frmax+" BL:"+blmax+" BR:"+brmax);
    }

    public void findMaxLeftSpeed() {
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_INCH * 60;
        setDriveVelocities(-3000, 3000, 3000, -3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        while (Math.abs(fLeftMotor.getCurrentPosition()) < travelTics) {
            flmax = (v = fLeftMotor.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fRightMotor.getVelocity()) > frmax ? v: frmax;
            blmax = (v = bLeftMotor.getVelocity()) > blmax ? v : blmax;
            brmax = (v = bRightMotor.getVelocity()) > brmax ? v : brmax;
        }
        stopDrive();
        teamUtil.log("Sideways Max Velocities FL:"+flmax+" FR:"+frmax+" BL:"+blmax+" BR:"+brmax);
    }
}

