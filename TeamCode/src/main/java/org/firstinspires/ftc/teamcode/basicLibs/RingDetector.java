package org.firstinspires.ftc.teamcode.basicLibs;/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class RingDetector {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FOUR_RINGS = "Quad";
    private static final String LABEL_ONE_RING = "Single";
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private runningVoteCount ringElection;

    private static final String VUFORIA_KEY = "AQfNUG7/////AAABmUE+GcnGE0LEkw6V6sBYnPdv0drO1QVLisryY2Kp9RhXImHEPLJJuIQaWyj3TKOYDB9P82rUavLg/jTofMcts0xLv8L5R4YfYDSZA4eJJMyEDPZxz6MSUXIpxhs7pof23wYX49SR5f/mvVq/qNOYb2DkpNSjTrMLTmyj0quYsA2LKS6C4zqbTr9XMQLGgmI9dYHV6Nk7HMcltcyB2ETUXPMew+bsp+UugBpt0VPjc9kW09Vy9ZGo9UncX7B/Gw73Kua6lUqqHvtfXpi3Sn2xJMcqWLHn5bxzr1xOwk9Co2kr8A3rU2gxpVzWMAnWHiWGWw9MY6GcIz6rJk+mu/e5jQeTTF08EK6ZXnzITpZQElx0";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public RingDetector(Telemetry theTelemetry, HardwareMap theHardwareMap) {
        teamUtil.log("Constructing Detector");
        this.telemetry = theTelemetry;
        this.hardwareMap = theHardwareMap;
        ringElection = new runningVoteCount(3000);  // track readings for the last 3 seconds

    }

    public void initDetector() {
        teamUtil.log("Initializing Detector");
        ringElection.clearVote();
        initVuforia();

        if (true /*ClassFactory.getInstance().canCreateTFObjectDetector()*/) { // TODO: Seem like 5.5 changed someething here...
            initTfod();
        } else {
            teamUtil.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        teamUtil.log("Initializing Detector - Finished");
    }

    public void activateDetector() {
        teamUtil.log("Detector -- Activating");
        if (tfod != null) {
            teamUtil.log("Detector -- calling activate on tfod");
            tfod.activate();
        }
    }

    public void shutDownDetector() {
        teamUtil.log("Detector -- Shutting down");
        if (tfod != null) {
            teamUtil.log("Detector -- calling shutdown on tfod");
            tfod.shutdown();
        }
    }


    public int detectRings() {
        if (tfod != null) {
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            //teamUtil.log("Detector -- Getting List");

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //teamUtil.log("Detector -- Change Found "+updatedRecognitions.size()+" Object(s)");
                //telemetry.addData("# Object Detected", updatedRecognitions.size());

                for (Recognition recognition : updatedRecognitions) {
                    /*
                    teamUtil.log("Found: " + recognition.getLabel()
                            + "Conf: " + recognition.getConfidence()
                            + " L: " + recognition.getLeft()
                            + " R: " + recognition.getRight()
                            + " T: " + recognition.getTop()
                            + " B: " + recognition.getBottom());

                     */
                    if (recognition.getLeft()<375 || recognition.getRight() > 900 || recognition.getTop() < 230 || recognition.getBottom() > 510){
                        return 0; // ghost recognition outside zooom box
                    } else if (recognition.getLabel() == LABEL_FOUR_RINGS) {
                        return 4;
                    } else if (recognition.getLabel() == LABEL_ONE_RING) {
                        return 1;
                    } else {
                        teamUtil.log("Found strange object, returning 0");
                        return 0;
                    }
                }
                //teamUtil.log("Empty list, returning 0");
                return 0;

            } else {
                //teamUtil.log("DetectRings -- no rings found");
                return 0;
                }
        }

        teamUtil.log("DetectRings -- tfod inactivated");
        return -1;
    }

    public void castVote(int vote) {
        if (vote==0){
            ringElection.vote(1);
        } else if (vote==1) {
            ringElection.vote(2);
        } else
            ringElection.vote(3);
    }

    public void detectAndVote() {
        castVote(detectRings());
    }

    public int getPath() {
        return ringElection.getWinner(1); //
    }

    public int[] getTotals () { return ringElection.getTotals();}

    public void reportRingInformation(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();
            } else {
                telemetry.addData("no detected rings", 0);
            }
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.setZoom(2.5, 16.0/9.0); // zoom in for better detection from the wall

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FOUR_RINGS, LABEL_ONE_RING);
    }

}

