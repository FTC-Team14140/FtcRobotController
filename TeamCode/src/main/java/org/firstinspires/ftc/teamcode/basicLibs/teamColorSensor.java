package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class teamColorSensor {
    private ColorSensor colorSensor;
    private int matValueRed;
    private int matValueBlue;
    double WHITE_THRESHOLD = 1000; // TODO: Tune this

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
        if (colorSensor.red() > matValueBlue * 1.5) {
            return TapeColor.RED;
        } else if (colorSensor.blue() > matValueBlue * 1.5) {
            return TapeColor.BLUE;
        } else if (colorSensor.alpha() > WHITE_THRESHOLD) {
            return TapeColor.WHITE;
        } else {
            return TapeColor.NONE;
        }
    }

    public boolean isOnTape(){
        return onBlue() || onRed();

    }
    public boolean onBlue() {
        return colorSensor.blue() > matValueBlue * 1.5;
    }
    public boolean onRed() {
        return colorSensor.red() > matValueBlue * 1.5;
    }
    public boolean onWhite() {
        return colorSensor.red() > matValueBlue * 1.5;
    }

    public int redValue() {
        return colorSensor.red();
    }
    public int blueValue() {
        return colorSensor.blue();
    }

}







