package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class teamColorSensor {
    private ColorSensor colorSensor;
    private int matValueRed;
    private int matValueBlue;
    double WHITE_THRESHOLD = 4000; // TODO: Tune this

    public enum TapeColor {RED, BLUE, WHITE, NONE}

    public teamColorSensor(ColorSensor theColorSensor) {
        colorSensor = theColorSensor;
    }

    public float getAlpha(){
        return colorSensor.alpha();
    }

    public void calibrate() {//Call this when the light sensor is over empty mat
        matValueBlue=colorSensor.blue();
        matValueRed=colorSensor.red();
    }

    public TapeColor getColor() {
        if (onWhite()) {
            return TapeColor.WHITE;  // Test for white first since white will also read red or blue...
        } else if (onRed()) {
            return TapeColor.RED;
        } else if (onBlue()) {
            return TapeColor.BLUE;
        } else  {
            return TapeColor.NONE;
        }
    }

    public boolean isOnTape(){
        return onBlue() || onRed() || onWhite();

    }
    public boolean onBlue() {
        return colorSensor.blue() > matValueBlue * 1.5;
    }
    public boolean onRed() {
        return colorSensor.red() > matValueBlue * 1.5;
    }
    public boolean onWhite() {
        return colorSensor.alpha() > WHITE_THRESHOLD;
    }

    public int redValue() {
        return colorSensor.red();
    }
    public int blueValue() {
        return colorSensor.blue();
    }

}







