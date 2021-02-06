package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;

public class testDriverStationDisplay extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriverStationDisplay display;
        TeamGamepad teamGamepad;
        ArrayList<String> testData = new ArrayList<String>();
        testData.add("left");
        testData.add("middle");
        testData.add("right");

        teamGamepad = new TeamGamepad(this);
        display = new DriverStationDisplay(telemetry, "Value", testData, 5);


       while(opModeIsActive()){
           teamGamepad.gamepadLoop();
           if(teamGamepad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADLEFT)){
               display.shiftSelectedDataLeft(0);
           } else if(teamGamepad.wasBounced(TeamGamepad.buttons.GAMEPAD1DPADRIGHT)){
               display.shiftSelectedDataRight(0);
           }


       }
    }
}
