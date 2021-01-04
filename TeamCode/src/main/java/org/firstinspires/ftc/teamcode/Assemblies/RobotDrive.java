package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.basicLibs.DistanceSensors;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class RobotDrive {

    public static final double FULL_POWER = 1;
    public static final double MAX_MOTOR_VELOCITY = 4400; // tics/sec, for forward and backward
    public static final double MAX_STRAFING_VELOCITY = 3600; //tics/sec, for left and right
    public double MAX_ACCEL_PER_INCH = 50; // max velocity acceleration per inch without skidding TODO: Update for the Ultimate Goal Robot
    public double MAX_DECEL_PER_INCH = 105; // max power deceleration per inch without skidding TODO: Update for the Ultimate Goal Robot
    public double START_SPEED = 150; // MIN power to get the robot to start moving
    public double END_SPEED = 50; // Power to decelerate to before stopping completely

    private double COUNTS_PER_INCH = 50.625;  // TODO: Update for the Ultimate Goal Robot
    private double COUNTS_PER_INCH_SIDEWAYS = 39.1;  // TODO: Update for the Ultimate Goal Robot


    public static double INITIAL_HEADING;


    HardwareMap hardwareMap;
    boolean timedOut = false;

    revHubIMUGyro revImu;

    DcMotorEx fLeftMotor;
    DcMotorEx bLeftMotor;
    DcMotorEx fRightMotor;
    DcMotorEx bRightMotor;

    /* Sensors from SkyStone
    public DistanceSensors frontLeftDistance;
    public DistanceSensors frontRightDistance;
    public DistanceSensors leftDistanceSensor;
    public DistanceSensors rightDistanceSensor;
    public DistanceSensors backDistanceSensor;
    public DistanceSensor frontmiddleDistance;
    public ColorSensor frontmiddleColor;
    public ColorSensor bottomColorSensor;
    public teamColorSensor bottomColor;
     */




    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Constructors, Initialization
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public RobotDrive(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        hardwareMap = theHardwareMap;
        //telemetry = theTelemetry;
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
        //fRightMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);  // these are NOT the defaults for these motors
        //bLeftMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);
        //bRightMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);
        setAllMotorsWithEncoder();
        setBrakeAllDriveMotors();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initSensors() {
    /*  TODO: rewrite when we add sensors to this year's robot
        frontSensorsOnly = frontOnly;
        frontLeftDistance = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "frontLeftDistance"));
        frontRightDistance = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "frontRightDistance"));
        if (!frontSensorsOnly) {
            leftDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance"));
            rightDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance"));
            backDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "backDistance"));
        }
        frontmiddleDistance = hardwareMap.get(DistanceSensor.class, "frontColorSensor");
        frontmiddleColor = hardwareMap.get(ColorSensor.class, "frontColorSensor");
        bottomColorSensor = hardwareMap.get(ColorSensor.class, "bottomColorSensor");
        bottomColor = new teamColorSensor(teamUtil.telemetry, bottomColorSensor);
        bottomColor.calibrate();
        frontLeftDistance.setOffset((float) (0.0));
        frontRightDistance.setOffset((float) (0.0));
        if (!frontSensorsOnly) {
            leftDistanceSensor.setOffset((float) (0.0));
            rightDistanceSensor.setOffset((float) (0.0));
            backDistanceSensor.setOffset((float) (0.0));
        }
    */
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void start() {
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
    /*  TODO: Rewrite when we add sensors to the robot
        teamUtil.telemetry.addData("frontLeftDistance:", "%.1f %b", getDistanceInches(frontLeftDistance),frontLeftDistance.didTimeOut());
        teamUtil.telemetry.addData("frontMiddleDistance:", "%.1f", frontmiddleDistance.getDistance(DistanceUnit.CM));
        teamUtil.telemetry.addData("frontRightDistance:", "%.1f %b", getDistanceInches(frontRightDistance),frontRightDistance.didTimeOut());
        if (!frontSensorsOnly) {
            teamUtil.telemetry.addData("leftDistance:", "%.1f %b", getDistanceInches(leftDistanceSensor),leftDistanceSensor.didTimeOut());
            teamUtil.telemetry.addData("rightDistance:", "%.1f %b", getDistanceInches(rightDistanceSensor),rightDistanceSensor.didTimeOut());
            teamUtil.telemetry.addData("backDistance:", "%.1f %b", getDistanceInches(backDistanceSensor),backDistanceSensor.didTimeOut());
        }
        teamUtil.telemetry.addLine("front color:" + frontmiddleColor.alpha() + ":" + frontmiddleColor.red() + ":" + frontmiddleColor.green() + ":" + frontmiddleColor.blue());
    */
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void telemetryDriveEncoders() {
        teamUtil.telemetry.addData("front left:", fLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("front right:", fRightMotor.getCurrentPosition());

        teamUtil.telemetry.addData("back left:", bLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("back right:", bRightMotor.getCurrentPosition());
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveTelemetry() {
        teamUtil.telemetry.addData("Front Left Motor:", fLeftMotor.getPower());
        teamUtil.telemetry.addData("Front Right Motor:", fRightMotor.getPower());
        teamUtil.telemetry.addData("Back Left Motor:", bLeftMotor.getPower());
        teamUtil.telemetry.addData("Back Right Motor:", bRightMotor.getPower());
        teamUtil.telemetry.addData("Heading:", getAbsoluteHeading());

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getBackLeftMotorPos() {
        return bLeftMotor.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getBackRightMotorPos() {
        return bRightMotor.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getFrontRightMotorPos() {
        return fRightMotor.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getFrontLeftMotorPos() {
        return fLeftMotor.getCurrentPosition();
    }







    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Low level helper functions
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Adjust readings from Distance Sensors to make sure we are not relying on bad data.
    // TODO: This should be added to the teamDistanceSensor class.
    public double getDistanceInches(DistanceSensors distanceSensor) {
        double distance = distanceSensor.getDistance();
        if (distance > 20) { // we don't trust readings above 20 inches with our current sensors
            return 1000;
        } else return distance;
    }

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
    public void setMotorVelocities(double fl, double fr, double bl, double br) {
        fLeftMotor.setVelocity(fl);
        fRightMotor.setVelocity(fr);
        bLeftMotor.setVelocity(bl);
        bRightMotor.setVelocity(br);
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // IMU and Angle related methods
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
        INITIAL_HEADING = revImu.getHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return our current heading as a 0 to 360 range.
    public double getHeading() {
        return adjustAngle(revImu.getHeading() - INITIAL_HEADING);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return the internal IMU heading without our offsets or fixes
    public double getAbsoluteHeading() {
        return revImu.getAbsoluteHeading();
    }








    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Basic Motor Power Methods
    // THESE ALL USE setPower() instead of setVelocity()!!

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopMotors() {
        fRightMotor.setPower(0);
        bRightMotor.setPower(0);
        bLeftMotor.setPower(0);
        fLeftMotor.setPower(0);
        teamUtil.log("STOP Motors");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        fLeftMotor.setPower(fl);
        fRightMotor.setPower(fr);
        bLeftMotor.setPower(bl);
        bRightMotor.setPower(br);
    }

    /* The following methods should all be deprecated...use or enhance the ones with acceleration curves instead
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public double clip(double power) {
        power = Range.clip(power, -FULL_POWER, FULL_POWER);
        return power;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForward(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(power);
        fRightMotor.setPower(power * .875); //   TODO: revisit these constants
        bLeftMotor.setPower(power);
        bRightMotor.setPower(power * .875);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackward(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(-power);
        fRightMotor.setPower(-power * .875); //  TODO: revisit these constants
        bLeftMotor.setPower(-power);
        bRightMotor.setPower(-power * .875);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRight(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(power);
        fRightMotor.setPower(-power * 0.8015384615);  //TODO: revisit these constants
        bLeftMotor.setPower(-power * 0.7361538462);
        bRightMotor.setPower(power * 0.8346153846);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeft(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(-power);
        fRightMotor.setPower(power * 0.9215384615); // TODO: revisit these constants
        bLeftMotor.setPower(power * 0.8061538462);
        bRightMotor.setPower(-power * 0.8846153846);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void rotateLeft(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(-power);
        fRightMotor.setPower(power);
        bLeftMotor.setPower(-power);
        bRightMotor.setPower(power);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void rotateRight(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(power);
        fRightMotor.setPower(-power);
        bLeftMotor.setPower(power);
        bRightMotor.setPower(-power);
    }

     */



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods for holding a heading using the gyroscope while moving

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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Run the robot forward/or backward maintaining the specified heading by turning proportionally as needed (P = .1)
    // TODO: We need a universal drive version of this that takes a drive heading and an attitude heading
    public void followHeading(double Heading, double ticsPerSecond) {
//        double velocityAdjust = getHeadingError(Heading) * .1 * ticsPerSecond;
        double velocityAdjust = 0;
        if (ticsPerSecond > 0) {
            fRightMotor.setVelocity(ticsPerSecond * 1 + velocityAdjust); // 1 was .87
            fLeftMotor.setVelocity(ticsPerSecond - velocityAdjust);
            bRightMotor.setVelocity(ticsPerSecond * 1 + velocityAdjust); // 1 was .87
            bLeftMotor.setVelocity(ticsPerSecond - velocityAdjust);
        } else {
            fRightMotor.setVelocity(ticsPerSecond * 1 - velocityAdjust); // 1 was .87
            fLeftMotor.setVelocity(ticsPerSecond + velocityAdjust);
            bRightMotor.setVelocity(ticsPerSecond * 1 - velocityAdjust); // 1 was .87
            bLeftMotor.setVelocity(ticsPerSecond + velocityAdjust);

        }
        //teamUtil.log("targetHeading: " + Heading + " Heading:" + getHeading() + " Adjust" + velocityAdjust);
    }




    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Movement methods using Distance Sensors
    // Enhanced multi-stage Move to Distance
    // Uses a 3 stage deceleration path and the IMU to hold the given heading.  initialTicsPerSecond should always be positive.
    // robot will move forward or backward as needed to gain the target distance reading on the supplied Distance Sensor
    // Currently this assumes distance sensors are on the front of the robot and postive power moves the robot forward.
    // TODO: Consider an acceleration phase from current velocity (could be 0) to avoid wheel slip at the beginning of the movement

    public void moveToDistance(DistanceSensors sensor, double distance, double initialTicsPerSecond, double heading, boolean moveBackIfNeeded, long timeOut) {
        boolean details = true;
        teamUtil.log("Moving to Distance: " + distance + " Velocity: " + initialTicsPerSecond);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        final double maxPower = initialTicsPerSecond;
        final double minPower = END_SPEED; // slow enough to be accurate, fast enough to actually move the robot

        double velocity = maxPower;
        double currentDistance = sensor.getDistance();

        teamUtil.log("Initial Distance Reading: " + currentDistance);

        // if we are already close enough, leave the robot where it is
        if (Math.abs(currentDistance - distance) <= 1) {
            teamUtil.log("Already There!  Not Moving...");

        } else if (currentDistance > distance) { // moving forward to target distance
            teamUtil.log("Moving Forward");

            final double preDriftTarget = distance + .5;
            final double slowThreshold = distance + 5;
            final double decelThreshold = slowThreshold + 10;
            final double slope = (maxPower - minPower) / (decelThreshold - slowThreshold); // slope for the decel phase
            teamUtil.log("preDriftTarget: " + preDriftTarget + " slowThreshold: " + slowThreshold + " decelThreshold: " + decelThreshold + " slope: " + slope);
            // Cruise at max speed
            while (currentDistance > decelThreshold && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, maxPower);
                if (details)
                    teamUtil.log("CRUISING: Distance:" + currentDistance + " velocity: " + maxPower);
                currentDistance = sensor.getDistance();
            }
            // Decelerate to min speed
            while (currentDistance > slowThreshold && teamUtil.keepGoing(timeOutTime)) {
                velocity = Math.min((currentDistance - slowThreshold) * slope + minPower, initialTicsPerSecond); // decelerate proportionally down to min - don't exceed initial speed
                followHeading(heading, velocity);
                if (details)
                    teamUtil.log("SLOWING: Distance:" + currentDistance + " velocity: " + velocity);
                currentDistance = sensor.getDistance();
            }
            // cruise at minSpeed once we are very close to target
            while (currentDistance > preDriftTarget && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, minPower);
                if (details)
                    teamUtil.log("CRAWLING: Distance:" + currentDistance + " velocity: " + minPower);
                currentDistance = sensor.getDistance();
            }
        } else if (moveBackIfNeeded) { // Moving Backwards to a target distance
            teamUtil.log("Moving Backwards");
            final double preDriftTarget = distance - .5;
            final double slowThreshold = distance - 5;
            final double decelThreshold = slowThreshold - 10;
            final double slope = (minPower - maxPower) / (slowThreshold - decelThreshold); // slope for the decel phase
            // Cruise at max speed
            while (currentDistance < decelThreshold && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, -maxPower);
                if (details)
                    teamUtil.log("CRUISING: Distance:" + currentDistance + " velocity: " + -maxPower);
                currentDistance = sensor.getDistance();
            }
            // Decelerate to min speed
            while (currentDistance < slowThreshold && teamUtil.keepGoing(timeOutTime)) {
                velocity = (currentDistance - decelThreshold) * slope + maxPower; // decelerate proportionally down to min
                followHeading(heading, -velocity);
                if (details)
                    teamUtil.log("SLOWING: Distance:" + currentDistance + " velocity: " + -velocity);
                currentDistance = sensor.getDistance();
            }
            // cruise at minSpeed once we are very close to target
            while (currentDistance < preDriftTarget && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, -minPower);
                if (details)
                    teamUtil.log("CRAWLING: Distance:" + currentDistance + " velocity: " + -minPower);
                currentDistance = sensor.getDistance();
            }
        }
        stopMotors();
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving to Distance - TIMED OUT!");
        }
        teamUtil.log("Finished Move to Distance");
    }







    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Methods for moving specific distances using acceleration/deceleration
    // The LATEST AND GREATEST versions that were used in Autonomous in SkyStone

    public int inchesToEncoderTics(double inches) {
        return (int) (inches * COUNTS_PER_INCH);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Ramp the motors smoothly from start velocity to end velocity use maximum acceleration/deceleration rates
    // This will work if both Velocities are positive or negative but not if one is positive and the other negative
    public void rampMotors(double startVelocity, double endVelocity, double heading, long timeOut) {
        boolean details = false;
        teamUtil.log("New Ramp Motors: Start:" + startVelocity + " End:" + endVelocity + " heading:" + heading);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        final double MAX_ACCEL_PER_INCH = 440; // max velocity acceleration per inch without skidding
        final double MAX_DECEL_PER_INCH = 200; // max power deceleration per inch without skidding

        // Figure out the distance we need to ramp in encoder tics
        int encoderCount; // always +
        double speedChange = Math.abs(endVelocity) - Math.abs(startVelocity); // + if accelerating or - if decelerating
        if (Math.abs(endVelocity) > Math.abs(startVelocity)) {
            encoderCount = inchesToEncoderTics(Math.abs(speedChange) / MAX_ACCEL_PER_INCH);
        } else {
            encoderCount = inchesToEncoderTics(Math.abs(speedChange) / MAX_DECEL_PER_INCH);

        }
        double slope = speedChange / encoderCount; // slope for the velocity ramp.  + or -

        setAllMotorsWithEncoder();
        int initialPosition = fRightMotor.getCurrentPosition();

        // Ramp the motors from one speed to the other
        int distanceTraveled = Math.abs(fRightMotor.getCurrentPosition() - initialPosition); // Always +
        while (distanceTraveled < encoderCount && teamUtil.keepGoing(timeOutTime)) {
            double velocity = slope * ((startVelocity > 0) ? distanceTraveled : -distanceTraveled) + startVelocity;
            followHeading(heading, velocity);
            if (details)
                teamUtil.log("Distance Traveled: " + distanceTraveled + " Velocity: " + velocity);
            distanceTraveled = Math.abs(fRightMotor.getCurrentPosition() - initialPosition); // Always +
        }

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("newRampMotors - TIMED OUT!");
        }
        teamUtil.log("newRampMotors - Finished");

    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Attempt to cover the specified distance at up to the specified speed using smooth acceleration at the start and deceleration at the end
    // this methods assumes the robot is at rest when it starts and will leave the robot at rest. Works for forward or backwards motion
    // This version uses setVelocity instead of setPower and also attempts to hold the specified heading
    public void accelerateInchesBackward(double maxVelocity, double inches, double heading, long timeOut) {
        accelerateInchesForward(-maxVelocity, inches, heading, timeOut);
    }

    public void accelerateInchesForward(double maxVelocity, double inches, double heading, long timeOut) {
        boolean details = false;
        teamUtil.log("newAccelerateInchesForward: velocity:" + maxVelocity + " Inches:" + inches + " heading:" + heading);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        double startSpeed = START_SPEED;
        double endSpeed = END_SPEED;
        boolean forward = true;

        if (maxVelocity < 0) { // moving backwards
            startSpeed = startSpeed * -1;
            endSpeed = endSpeed * -1;
        }

        // Figure out the distances for each phase in encoder tics.  These are all + numbers
        int totalEncoderCount = inchesToEncoderTics(inches);
        int accelerationEncoderCount = Math.abs(inchesToEncoderTics((maxVelocity - startSpeed) / MAX_ACCEL_PER_INCH));
        int decelerationEncoderCount = Math.abs(inchesToEncoderTics((maxVelocity - endSpeed) / MAX_DECEL_PER_INCH));

        // figure out slopes for acceleration and deceleration phases.  Each could be + or -
        double accelerationSlope = (maxVelocity - startSpeed) / accelerationEncoderCount;
        double decelerationSlope = (maxVelocity - endSpeed) / decelerationEncoderCount * -1;

        int initialPosition = fRightMotor.getCurrentPosition();
        int target = initialPosition + ((maxVelocity > 0) ? totalEncoderCount : -totalEncoderCount);
        int cruiseStart, decelerationStart;
        if (accelerationEncoderCount + decelerationEncoderCount < totalEncoderCount) {
            // Enough distance to reach maxVelocity
            cruiseStart = initialPosition + ((maxVelocity > 0) ? accelerationEncoderCount : -accelerationEncoderCount);
            decelerationStart = target - ((maxVelocity > 0) ? decelerationEncoderCount : -decelerationEncoderCount);
        } else {
            // we don't have enough space to ramp up to full speed so calculate the actual maximum velocity
            // by finding the y value of the two ramp lines where they intersect given the maximum distance
            maxVelocity = (-decelerationSlope * totalEncoderCount * accelerationSlope - startSpeed * decelerationSlope) / (accelerationSlope - decelerationSlope);
            teamUtil.log("Adjusted Max Velocity to:" + maxVelocity);

            // recompute shortened ramp phases
            accelerationEncoderCount = Math.abs(inchesToEncoderTics((maxVelocity - startSpeed) / MAX_ACCEL_PER_INCH));
            decelerationEncoderCount = Math.abs(inchesToEncoderTics((maxVelocity - endSpeed) / MAX_DECEL_PER_INCH));
            cruiseStart = initialPosition + ((maxVelocity > 0) ? accelerationEncoderCount : -accelerationEncoderCount);
            decelerationStart = target - ((maxVelocity > 0) ? decelerationEncoderCount : -decelerationEncoderCount);
        }
        teamUtil.log("accelerationSlope:" + accelerationSlope + " decelerationSlope:" + decelerationSlope + " totalEncoderCount:" + totalEncoderCount);
        teamUtil.log("accelerationEncoderCount:" + accelerationEncoderCount + " decelerationEncoderCount:" + decelerationEncoderCount + " decelerationStart:" + decelerationStart + " totalEncoderCount:" + totalEncoderCount);

        // ramp up
        rampMotors(startSpeed, maxVelocity, heading, timeOut);

        // Cruise at Max Velocity
        int currentPosition = fRightMotor.getCurrentPosition();
        if (maxVelocity > 0) { // driving forward
            while (currentPosition < decelerationStart && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, maxVelocity);
                if (details)
                    teamUtil.log("Distance Traveled: " + (currentPosition - initialPosition) + "Velocity: " + maxVelocity);
                currentPosition = fRightMotor.getCurrentPosition();
            }
        } else {
            while (currentPosition > decelerationStart && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, maxVelocity);
                if (details)
                    teamUtil.log("Distance Traveled: " + (currentPosition - initialPosition) + "Velocity: " + maxVelocity);
                currentPosition = fRightMotor.getCurrentPosition();
            }

        }

        // ramp down
        rampMotors(maxVelocity, endSpeed, heading, timeOutTime - System.currentTimeMillis());

        stopMotors();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("newRampMotors - TIMED OUT!");
        }
        teamUtil.log("newRampMotors - Finished");

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

    double LastEndSpeed = 0;  //Keep track of the end velocity of the last movement so we can start from there on the next
    double DRIVE_MAX_VELOCITY = 500; // TODO: find a reasonable maximum velocity for the drive
    double ROTATION_ADJUST_FACTOR = 0.1; // TODO: verify this is a reasonable P coefficient for rotational error adjustment

    // Return the number of encoder tics needed to ramp from start speed to end speed at the specified acceleration
    public int rampEncoderCount(double startSpeed, double endSpeed, double acceleration) {
        return Math.abs(inchesToEncoderTics((endSpeed - startSpeed) / acceleration));
    }

    class MotorData { // a helper class to allow for faster access to hub data
        int eFL, eFR, eBL, eBR;
    }

    // Return the encoder positions for all 4 drive motors
    // TODO: Use the Batch data feature (see examples) to speed this up
    public void getDriveMotorData(MotorData data) {
        data.eFL = fLeftMotor.getCurrentPosition();
        data.eFR = fRightMotor.getCurrentPosition();
        data.eBL = bLeftMotor.getCurrentPosition();
        data.eBR = bRightMotor.getCurrentPosition();
    }

    // Update the drive motor velocities
    // TODO: Use the Batch data feature (see examples) to speed this up
    public void setDriveVelocities(double flV, double frV, double blV, double brV){
        fLeftMotor.setVelocity(flV);
        fRightMotor.setVelocity(frV);
        bLeftMotor.setVelocity(blV);
        bRightMotor.setVelocity(brV);
    }


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

    // Set the velocity of all 4 motors based on a driveHeading and provided velocity
    // Corrects for rotational drift based on robotHeading
    public void driveMotorsHeadings(double driveHeading, double robotHeading, double velocity) {
        double flV, frV, blV, brV;
        double x, y, scale;

        // Determine how much adjustment for rotational drift
        double rotationAdjust = getHeadingError(robotHeading) * ROTATION_ADJUST_FACTOR * velocity;

        // Covert heading to cartesian on the unit circle and scale so largest value is 1
        // This is essentially creating joystick values from the heading
        x = Math.cos(Math.toRadians(driveHeading + 90)); // + 90 cause forward is 0...
        y = Math.sin(Math.toRadians(driveHeading + 90));
        scale = 1 / Math.max(Math.abs(x), Math.abs(y));
        x = x * scale;
        y = y * scale;

        // Clip to motor power range
        flV = Math.max(-1.0, Math.min(x + y, 1.0));
        brV = flV;
        frV = Math.max(-1.0, Math.min(y - x, 1.0));
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
    // Drive the motors towards the specified heading, holding the robotHeading, for the specified encodercount
    // Ramp velocity from startVelocity to endVelocity linearly
    public void driveMotors(double driveHeading, double robotHeading, double encoderCount, double startVelocity, double endVelocity, long timeOut) {
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
        boolean stopAtEnd = false;
        teamUtil.log("moveInches: driveHeading:" + driveHeading + " Inches:" + inches+ "velocity:" + maxVelocity);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        double startSpeed = LastEndSpeed;
        LastEndSpeed = endSpeed; // save this for next time this method is called TODO: Need a way to deal with some other method stopping the robot...  They could all just set this...
        if (endSpeed == 0) {  // in case they are trying to stop the robot, fix it so we don't stall
            endSpeed = END_SPEED;
            stopAtEnd = true;
        }

        // Figure out the distances for each phase in encoder tics.  These are all + numbers
        int totalEncoderCount = inchesToEncoderTics(inches);
        int accelerationEncoderCount = rampEncoderCount(startSpeed, maxVelocity, MAX_ACCEL_PER_INCH);
        int decelerationEncoderCount = rampEncoderCount(maxVelocity, endSpeed, MAX_DECEL_PER_INCH);

        // figure out slopes for acceleration and deceleration phases.
        double accelerationSlope = (maxVelocity - startSpeed) / accelerationEncoderCount; // positive slope
        double decelerationSlope = (maxVelocity - endSpeed) / decelerationEncoderCount * -1; // Negative slope

        int target = totalEncoderCount;
        int cruiseStart, decelerationStart;
        if (accelerationEncoderCount + decelerationEncoderCount < totalEncoderCount) {
            // Enough distance to reach maxVelocity
            cruiseStart = accelerationEncoderCount;
            decelerationStart = target - decelerationEncoderCount;
        } else {
            // we don't have enough space to ramp up to full speed so calculate the actual maximum velocity
            // by finding the y value of the two ramp lines where they intersect given the maximum distance
            maxVelocity = (decelerationSlope * totalEncoderCount * accelerationSlope -
                    endSpeed*accelerationSlope +
                    startSpeed * decelerationSlope) / decelerationSlope;
            teamUtil.log("Limited Max Velocity to:" + maxVelocity);

            // recompute shortened ramp phases
            accelerationEncoderCount = rampEncoderCount(startSpeed, maxVelocity, MAX_ACCEL_PER_INCH);
            decelerationEncoderCount = rampEncoderCount(maxVelocity, endSpeed, MAX_DECEL_PER_INCH);
            cruiseStart = accelerationEncoderCount;
            decelerationStart = target - decelerationEncoderCount;
        }
        teamUtil.log("accelerationSlope:" + accelerationSlope + " decelerationSlope:" + decelerationSlope + " totalEncoderCount:" + totalEncoderCount);
        teamUtil.log("accelerationEncoderCount:" + accelerationEncoderCount + " decelerationEncoderCount:" + decelerationEncoderCount + " decelerationStart:" + decelerationStart + " totalEncoderCount:" + totalEncoderCount);

        // ramp up
        driveMotors(driveHeading, robotHeading,accelerationEncoderCount,startSpeed,maxVelocity, timeOut);

        // Cruise at Max Velocity
        driveMotors(driveHeading, robotHeading,decelerationStart-cruiseStart, maxVelocity,maxVelocity,  timeOutTime - System.currentTimeMillis());

        // ramp down
        driveMotors(driveHeading, robotHeading,totalEncoderCount-decelerationStart,maxVelocity,endSpeed, timeOutTime - System.currentTimeMillis());

        if (stopAtEnd) {  // stop the motors if they intend the robot to hold still
            setDriveVelocities(0,0,0,0);
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
    // Methods for rotating the robot
    //
    // THESE ARE THE LATEST AND GREATEST THAT WERE USED IN SKYSTONE

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate to the desired direction at the maximum speed
    public enum RobotRotation {GOAL, START, OUR_SIDE, THEIR_SIDE}

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
                    rotateTo(90.0);
                } else {
                    rotateTo(270.0);
                }
                break;
            case THEIR_SIDE:
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    rotateTo(90.0);
                } else {
                    rotateTo(270.0);
                }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate to the desired heading at the maximum speed, slowing for accuracy at the end
    // TODO: modify to use setVelocity() instead of setPower()

    public void rotateTo(double heading) {
        teamUtil.log("Starting to rotate to " + heading);
        final double decelThreshold = 60; // start deceleration this many degrees from the target TODO: Re-tune for new robot
        final double slowThreshold = 10; // slow down to a very slow turn this far from the target TODO: Re-tune for new robot
        final double maxPower = 1; // TODO: Re-tune for new robot
        final double minPower = .15; // TODO: Re-tune for new robot
        final double decelSlope = (maxPower - minPower) / (decelThreshold - slowThreshold); // + slope
        final double driftDegrees = 1; // cut the motors completely when we are within this many degrees of the target to allow for a little drift
        double leftRotatePower = 1; // Keep track of which way we are rotating
        double rightRotatePower = 1;
        double rotatePower = maxPower; // start at full power

        double currentHeading = getHeading();
        double initialHeading = currentHeading; // Stash this so we can make this a "relative" turn from a heading of 0.

        // Determine how many degrees we need to turn from our current position to get to the target
        double turnDegrees = minDegreeDiff(heading, currentHeading); // always +
        turnDegrees = turnDegrees - driftDegrees; // stop early to allow for drift

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

        // Rotate at max power until we get to deceleration phase
        while (currentHeading < turnDegrees - decelThreshold) {
            setMotorPowers(maxPower * leftRotatePower, maxPower * rightRotatePower, maxPower * leftRotatePower, maxPower * rightRotatePower);
            //teamUtil.log("MAX: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + maxPower);
            currentHeading = minDegreeDiff(getHeading(), initialHeading); // always +
        }

        // rotate at decelerating power as we close to target
        while (currentHeading < turnDegrees - slowThreshold) {
            rotatePower = (turnDegrees - slowThreshold - currentHeading) * decelSlope + minPower; // decelerate proportionally down to min
            setMotorPowers(rotatePower * leftRotatePower, rotatePower * rightRotatePower, rotatePower * leftRotatePower, rotatePower * rightRotatePower);
            //teamUtil.log("DECEL: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + rotatePower);
            currentHeading = minDegreeDiff(getHeading(), initialHeading); // always +
        }

        // rotate at minSpeed once we are very close to target
        while (currentHeading < turnDegrees) {
            setMotorPowers(minPower * leftRotatePower, minPower * rightRotatePower, minPower * leftRotatePower, minPower * rightRotatePower);
            //teamUtil.log("CRAWL: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + minPower);
            currentHeading = minDegreeDiff(getHeading(), initialHeading); // always +
        }
        stopMotors();
        teamUtil.log("Finished Turning.  Actual Heading: " + getHeading());
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
        setMotorVelocities(10000, 10000, 10000, 10000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        while (fLeftMotor.getCurrentPosition() < travelTics) {
            flmax = (v = fLeftMotor.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fRightMotor.getVelocity()) > frmax ? v: frmax;
            blmax = (v = bLeftMotor.getVelocity()) > blmax ? v : blmax;
            brmax = (v = bRightMotor.getVelocity()) > brmax ? v : brmax;
        }
        stopMotors();
        teamUtil.log("Forward Max Velocities FL:"+flmax+" FR:"+frmax+" BL:"+blmax+" BR:"+brmax);
    }

    public void findMaxLeftSpeed() {
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_INCH * 60;
        setMotorVelocities(-10000, 10000, 10000, -10000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        while (Math.abs(fLeftMotor.getCurrentPosition()) < travelTics) {
            flmax = (v = fLeftMotor.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fRightMotor.getVelocity()) > frmax ? v: frmax;
            blmax = (v = bLeftMotor.getVelocity()) > blmax ? v : blmax;
            brmax = (v = bRightMotor.getVelocity()) > brmax ? v : brmax;
        }
        stopMotors();
        teamUtil.log("Sideways Max Velocities FL:"+flmax+" FR:"+frmax+" BL:"+blmax+" BR:"+brmax);
    }
}

