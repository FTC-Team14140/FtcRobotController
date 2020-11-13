package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.DistanceSensors;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class RobotDrive {

    public static final double FULL_POWER = 1;
    public static final double MAX_MOTOR_VELOCITY = 1000; // tics/sec  TODO: Update for the Ultimate Goal Robot
    public double MAX_ACCEL_PER_INCH = 440; // max velocity acceleration per inch without skidding TODO: Update for the Ultimate Goal Robot
    public double MAX_DECEL_PER_INCH = 175; // max power deceleration per inch without skidding TODO: Update for the Ultimate Goal Robot
    public double START_SPEED = 440; // MIN power to get the robot to start moving TODO: Update for the Ultimate Goal Robot
    public double END_SPEED = 300; // Power to decelerate to before stopping completely TODO: Update for the Ultimate Goal Robot

    private double COUNTS_PER_INCH = 62.24;  // TODO: Update for the Ultimate Goal Robot
    private double COUNTS_PER_INCH_SIDEWAYS = 67.82;  // TODO: Update for the Ultimate Goal Robot


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
        //fLeftMotor = hardwareMap.dcMotor.get("fLeftMotor");
        //fRightMotor = hardwareMap.dcMotor.get("fRightMotor");
        //bLeftMotor = hardwareMap.dcMotor.get("bLeftMotor");
        // bRightMotor = hardwareMap.dcMotor.get("bRightMotor");
        // fLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // bLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // setAllMotorsWithoutEncoder();  // This might not be the best option at this point...

        fLeftMotor = hardwareMap.get(DcMotorEx.class, "fLeftMotor");
        fRightMotor = hardwareMap.get(DcMotorEx.class, "fRightMotor");
        bLeftMotor = hardwareMap.get(DcMotorEx.class, "bLeftMotor");
        bRightMotor = hardwareMap.get(DcMotorEx.class, "bRightMotor");
        fLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fLeftMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);   // These coeffiecients were found using the technique in this doc: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#
        fRightMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);  // these are NOT the defaults for these motors
        bLeftMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);
        bRightMotor.setVelocityPIDFCoefficients(1.5, 0.15, 0, 14.9);
        setAllMotorsWithEncoder();
        setBrakeAllDriveMotors();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initSensors(boolean frontOnly) {
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
    public void encoderTelemetry() {
        teamUtil.telemetry.addData("FL ENCODER POS:", fRightMotor.getCurrentPosition());
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
    // TODO: This one needs a comment...does it really need to call correctHeading AND adjustAngle?
    public double getRelativeHeading(double pseudoHeading) {
        return revImu.correctHeading(adjustAngle(pseudoHeading + getHeading()));
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
        double velocityAdjust = getHeadingError(Heading) * .1 * ticsPerSecond;
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
    // Basic Movement methods for inches or time
    // Use at lower speeds or wheel slippage may compromise accuracy

    /* The following methods should all be deprecated...use or enhance the ones with acceleration curves instead
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move forward for a specified amouunt of time and then stop
    public void driveForward(double power, double timeInMilliseconds) {
        teamUtil.log("Moving Forward Milliseconds: " + timeInMilliseconds);
        ElapsedTime driveTime = new ElapsedTime();
        power = clip(power);
        setBrakeAllDriveMotors();
        do {
            driveForward(power);
        } while (driveTime.milliseconds() < timeInMilliseconds && teamUtil.theOpMode.opModeIsActive());
        stopMotors();
        teamUtil.log("Moving Forward Milliseconds - Finished");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // move a specified number of inches (using encoder on front right wheel to measure) with the
    // provided power/speed values for each of the 4 motors.
    // This will work in both RUN_USING_ENCODERS and RUN_WITHOUT_ENCODERS but may give very different results...
    public void moveInches(double distance, double lfspeed, double rfspeed, double lbspeed, double rbspeed, long timeOut) {
        teamUtil.log("Move Inches: " + distance + " " + fRightMotor.getMode()); // log the distance and motor mode
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        setBrakeAllDriveMotors();
        int offset = fRightMotor.getCurrentPosition();
        int encoderCounts = (int) (COUNTS_PER_INCH * distance);
        fLeftMotor.setPower(lfspeed);
        fRightMotor.setPower(rfspeed);
        bLeftMotor.setPower(lbspeed);
        bRightMotor.setPower(rbspeed);
        while ((Math.abs(fRightMotor.getCurrentPosition() - offset) < encoderCounts) && teamUtil.keepGoing(timeOutTime)) {
            // cruising to our destination  could log encoder positions here if needed
        }
        stopMotors();
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Move Inches - TIMED OUT!");
        }
        teamUtil.log("Move Inches - Finished");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInchesForward(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Forward: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();
        setBrakeAllDriveMotors();

        //sets the number of desired inches on both motors
        int encoderCounts = (int) (COUNTS_PER_INCH * inches);
        speed = clip(speed);

        do {

            driveForward(speed);

            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));


        //turns off both motors
        stopMotors();

        //sets it back to normal (there is nothing normal about running drive motors without encoders...-Coach)
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Forward - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Forward - Finished");

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInchesBackward(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Backward: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        //fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH * inches);
        speed = clip(speed);

        do {
            driveBackward(speed);
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed

        //turns off both motors
        stopMotors();

        //sets it back to normal
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Backward - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Backward - Finished");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInchesLeft(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Left: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        //fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH_SIDEWAYS * inches);
        speed = clip(speed);

        do {
            driveLeft(speed);
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed

        //turns off both motors
        stopMotors();

        //sets it back to normal
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Left - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Left - Finished");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInchesRight(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Right: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        //fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH_SIDEWAYS * inches);
        speed = clip(speed);

        do {
            driveRight(speed);
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        //turns off both motors
        stopMotors();

        //sets it back to normal
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Right - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Right - Finished");
    }

    */



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Methods for moving specific distances using acceleration/deceleration
    // The LATEST AND GREATEST versions that were used in Autonomous in SkyStone
    // TODO: create left and right movement versions of these
    // TODO: Better yet, create a version that works for any direction like this:
    //      moveInches(double maxVelocity, double inches, double driveHeading, double robotHeading, long timeOut)

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



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods to move a specific distance with acceleration/deceleration but with non zero start or end speeds
    // These presume the caller will handle needed movement before or after
    // TODO: If we are going to use these methods, we should modify them to use SetVelocity() instead of setPower() on the motors

/*
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
    public void accelerateToSpeedRight(double startSpeed, double endSpeed) {
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.35, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.35, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("About to accelerate right");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveRight(accelSpeed);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
    public void accelerateToSpeedForwards(double startSpeed, double endSpeed) {
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.3, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.3, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta accelerate forward");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED - (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveForward(accelSpeed);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
    public void accelerateToSpeedBackwards(double startSpeed, double endSpeed) {
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.3, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.3, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta accelerate back");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveBackward(accelSpeed);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
    public void accelerateToSpeedLeft(double startSpeed, double endSpeed) {
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.35, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.35, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta accelerate left");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED - (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveLeft(accelSpeed);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Decelerate from the startSpeed to a stop after traveling the specified distance while trying not to slip the wheels.
    // This assume the robot is moving when it is called
    public void decelerateInchesRight(double startSpeed, double inches) {
        double startingPosition = fRightMotor.getCurrentPosition();
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double targetPositionRightMotor = inches * COUNTS_PER_INCH;

        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
        double endingPoint = startingPosition - inches * COUNTS_PER_INCH;
        double decelerationPoint = endingPoint + decelerationEncoderCount;


        setAllMotorsWithEncoder();
//        resetAllDriveEncoders();

        while (fRightMotor.getCurrentPosition() > decelerationPoint) {
            //CRUISING
            driveRight(startSpeed);
            teamUtil.log("Cruising speed: " + startSpeed);
        }

        teamUtil.log("start decelerating");

        while (fRightMotor.getCurrentPosition() > endingPoint) {
            //DECELERATING
            double decelSpeed = Range.clip((startSpeed - (Math.abs(fRightMotor.getCurrentPosition() - decelerationPoint) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
            teamUtil.log("Deceleration speed: " + decelSpeed);
            teamUtil.log("decelDistanceTraveled: " + (Math.abs(fRightMotor.getCurrentPosition() - decelerationPoint)));
            teamUtil.log("DecelDistance: " + decelerationEncoderCount);

            driveRight(decelSpeed);
        }

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Decelerate from the startSpeed to a stop after traveling the specified distance while trying not to slip the wheels.
    // This assume the robot is moving when it is called
    public void decelerateInchesLeft(double startSpeed, double inches) {
        double startingPosition = fRightMotor.getCurrentPosition();

        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;

        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
        double endingPoint = startingPosition + inches * COUNTS_PER_INCH;
        double decelerationPoint = endingPoint - decelerationEncoderCount;


        setAllMotorsWithEncoder();
//        resetAllDriveEncoders();

        while (fRightMotor.getCurrentPosition() < decelerationPoint) {
            //CRUISING
            driveLeft(startSpeed);
            teamUtil.log("Cruising speed: " + startSpeed);
        }

        teamUtil.log("start decelerating");

        while (fRightMotor.getCurrentPosition() < endingPoint) {
            //DECELERATING
            double decelSpeed = Range.clip((startSpeed - ((fRightMotor.getCurrentPosition() - decelerationPoint) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
            teamUtil.log("Deceleration speed: " + decelSpeed);
            teamUtil.log("decelDistanceTraveled: " + (fRightMotor.getCurrentPosition() - decelerationPoint));
            teamUtil.log("DecelDistance: " + decelerationEncoderCount);

            driveLeft(decelSpeed);
        }

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Not really sure what this does...needs a comment!
    public void decelerateToSpeedLeft(double startSpeed) {
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta decelerate left");
        teamUtil.log("decel Distance: " + decelerationEncoderCount * 1 / COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationEncoderCount) {

            double accelSpeed = (startSpeed + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH);
            teamUtil.log("deceleration speed: " + accelSpeed);
            driveLeft(accelSpeed);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Not really sure what this does...needs a comment!
    public void decelerateToSpeedForwards(double startSpeed) {
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta decelerate forward");
        teamUtil.log("decel Distance: " + decelerationEncoderCount * 1 / COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationEncoderCount) {

            double accelSpeed = (startSpeed + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH);
            teamUtil.log("deceleration speed: " + accelSpeed);
            driveForward(accelSpeed);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Not really sure what this does...needs a comment!
    public void decelerateToSpeedBackwards(double startSpeed) {
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta decelerate back");
        teamUtil.log("decel Distance: " + decelerationEncoderCount * 1 / COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationEncoderCount) {

            double accelSpeed = (startSpeed - (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH);
            teamUtil.log("deceleration speed: " + accelSpeed);
            driveBackward(accelSpeed);
        }
    }


 */



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
        final double decelThreshold = 60; // start deceleration this many degrees from the target
        final double slowThreshold = 10; // slow down to a very slow turn this far from the target
        final double maxPower = 1;
        final double minPower = .15;
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
        fRightMotor.setPower(frontRight * 0.9);
        bRightMotor.setPower(backRight * 0.9);
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
        double travelTics = COUNTS_PER_INCH * 72;
        setMotorVelocities(10000, 10000, 10000, 10000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0;
        while (fLeftMotor.getCurrentPosition() < travelTics) {
            flmax = getFrontLeftMotorPos() > flmax ? getFrontLeftMotorPos() : flmax;
            frmax = getFrontRightMotorPos() > frmax ? getFrontRightMotorPos() : frmax;
            blmax = getBackLeftMotorPos() > blmax ? getBackLeftMotorPos() : blmax;
            brmax = getBackRightMotorPos() > brmax ? getBackRightMotorPos() : brmax;
        }
        stopMotors();
        teamUtil.log("Max Velocities FL:"+flmax+" FR:"+frmax+" BL:"+blmax+" BR:"+brmax);
    }

    public void findMaxLeftSpeed() {
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_INCH * 72;
        setMotorVelocities(-10000, 10000, 10000, -10000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0;
        while (fLeftMotor.getCurrentPosition() < travelTics) {
            flmax = getFrontLeftMotorPos() < flmax ? getFrontLeftMotorPos() : flmax;
            frmax = getFrontRightMotorPos() > frmax ? getFrontRightMotorPos() : frmax;
            blmax = getBackLeftMotorPos() > blmax ? getBackLeftMotorPos() : blmax;
            brmax = getBackRightMotorPos() < brmax ? getBackRightMotorPos() : brmax;
        }
        stopMotors();
        teamUtil.log("Max Velocities FL:"+flmax+" FR:"+frmax+" BL:"+blmax+" BR:"+brmax);
    }
}

