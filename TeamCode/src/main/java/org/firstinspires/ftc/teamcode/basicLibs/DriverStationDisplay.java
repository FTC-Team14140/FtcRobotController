package org.firstinspires.ftc.teamcode.basicLibs;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Locale;

public class DriverStationDisplay {

    Telemetry telemetry;
    String name;
    ArrayList<String> data = new ArrayList<String>();
    String selectedData;
    private Telemetry.Item[] mainDisplay;
    int MAX_LINE_NUMS = 12;


    DriverStationDisplay(Telemetry telemetry, String name, ArrayList<String> data, int lineNums) {
        this.telemetry = telemetry;
        this.name = name;
        this.data = data;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        mainDisplay = new Telemetry.Item[lineNums];
        //maybe instead of working in html I'll stick with the Item array cuz it's easier to handle adding or removing lines
//telemetry.addLine("<ul> <li> hii </li> <li> reee </li> </ul>")

        if(this.data.size() != 0){
            selectedData = data.get(0);
        } else{
            //log that data needs to be larger
        }



        for(int i = 0; i < lineNums; i++){
        mainDisplay[i] = telemetry.addData(String.format(Locale.US, "%d", i ), "");
        }
        displaySettingData(0);
        telemetry.update();

    }


    public void displayText(String msg, int lineNum){
        mainDisplay[lineNum].setValue(msg);
        telemetry.update();
    }



    public void addData(String newData){
        data.add(newData);
    }

    public void removeData(String dataToRemove){
        data.remove(dataToRemove);
    }

    public void displaySettingData(int lineNum){
        mainDisplay[lineNum].setValue(selectedData);
        telemetry.update();
    }



    public void shiftSelectedDataLeft(int lineNum){
        int selectedDataIndex = data.indexOf(selectedData);
        int newDataIndex = selectedDataIndex - 1;

        if(newDataIndex > 0){
            selectedData = data.get(newDataIndex);
        } else {
            //log that this action is impossible
        }
        displaySettingData(lineNum);

    }

    public void shiftSelectedDataRight(int lineNum){
        int selectedDataIndex = data.indexOf(selectedData);
        int newDataIndex = selectedDataIndex + 1;

        if(newDataIndex < data.size()){
            selectedData = data.get(newDataIndex);
        } else {
            //log that this action is impossible
        }
        displaySettingData(lineNum);
    }

}
