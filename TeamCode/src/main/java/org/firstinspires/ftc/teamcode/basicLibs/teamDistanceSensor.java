package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class teamDistanceSensor {

    Rev2mDistanceSensor sensor;
    double minValidReading, maxValidReading;
    private float offset;
    public boolean lastReadingValid = false; // True if last reading was valid and was not clipped
    sampleStats stats = null;


    public teamDistanceSensor(Rev2mDistanceSensor theSensor, float theOffset, double maxDistance, double minDistance) {
        sensor = theSensor;
        offset = theOffset;
        minValidReading = minDistance;
        maxValidReading = maxDistance;
    }
    public void useRunningAverage(int milliSeconds){
        stats = new sampleStats();
        stats.setTimeWindow(milliSeconds);
    }
    private double adjustDistance(double distance) {
        if (distance < minValidReading) {
            lastReadingValid = false;
            return minValidReading;
        } else if (distance > maxValidReading) {
            lastReadingValid = false;
            return maxValidReading;
        } else {
            lastReadingValid = true;
            return distance;
        }
    }
    // Return the current reading constrained to the min and max settings
    public double getDistanceInches() {
        double d = adjustDistance( sensor.getDistance(DistanceUnit.INCH) + offset);
        if (stats != null) {
            stats.addSample(d);
            return stats.getRunningAverage();
        } else {
            return d;
        }
    }

    // Return the current reading constrainted to the min and max settings
    public double getDistanceCms() {
        double d = adjustDistance( sensor.getDistance(DistanceUnit.CM) + offset);
        if (stats != null) {
            stats.addSample(d);
            return stats.getRunningAverage();
        } else {
            return d;
        }
    }
}
