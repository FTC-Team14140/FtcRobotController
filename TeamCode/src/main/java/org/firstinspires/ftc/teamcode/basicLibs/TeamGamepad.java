package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeamGamepad {
    private OpMode theOpMode;
    public enum buttons{
        GAMEPAD1RB,
        GAMEPAD1LB,
        GAMEPAD1A,
        GAMEPAD1B,
        GAMEPAD1X,
        GAMEPAD1Y,
        GAMEPAD1DPADUP,
        GAMEPAD1DPADDOWN,
        GAMEPAD1DPADLEFT,
        GAMEPAD1DPADRIGHT,
        GAMEPAD1RIGHTTRIGGER,
        GAMEPAD1LEFTTRIGGER,
        GAMEPAD2RB,
        GAMEPAD2LB,
        GAMEPAD2A,
        GAMEPAD2B,
        GAMEPAD2X,
        GAMEPAD2Y,
        GAMEPAD2DPADUP,
        GAMEPAD2DPADDOWN,
        GAMEPAD2DPADLEFT,
        GAMEPAD2DPADRIGHT,
        GAMEPAD2RIGHTTRIGGER,
        GAMEPAD2LEFTTRIGGER
    }
    private boolean[] pressed = new boolean[buttons.values().length];
    private boolean[] bounced = new boolean[buttons.values().length];


    public TeamGamepad(OpMode opmode){
        theOpMode = opmode;
    }

    // Call this to find out if a button was pressed and released
    public boolean wasBounced(buttons button){
        if(bounced[button.ordinal()]){
            bounced[button.ordinal()] = false;
            return true;
        } else {
            return false;
        }
    }

    // Call this each time through your loop
    public void gamepadLoop() {
        if(theOpMode.gamepad1.right_bumper){
            pressed[buttons.GAMEPAD1RB.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1RB.ordinal()] && !theOpMode.gamepad1.right_bumper){
            bounced[buttons.GAMEPAD1RB.ordinal()] = true;
            pressed[buttons.GAMEPAD1RB.ordinal()] = false;
        }

        if(theOpMode.gamepad1.left_bumper){
            pressed[buttons.GAMEPAD1LB.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1LB.ordinal()] && !theOpMode.gamepad1.left_bumper){
            bounced[buttons.GAMEPAD1LB.ordinal()] = true;
            pressed[buttons.GAMEPAD1LB.ordinal()] = false;
        }

        if(theOpMode.gamepad1.a){
            pressed[buttons.GAMEPAD1A.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1A.ordinal()] && !theOpMode.gamepad1.a){
            bounced[buttons.GAMEPAD1A.ordinal()] = true;
            pressed[buttons.GAMEPAD1A.ordinal()] = false;
        }

        if(theOpMode.gamepad1.b){
            pressed[buttons.GAMEPAD1B.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1B.ordinal()] && !theOpMode.gamepad1.b){
            bounced[buttons.GAMEPAD1B.ordinal()] = true;
            pressed[buttons.GAMEPAD1B.ordinal()] = false;
        }

        if(theOpMode.gamepad1.x){
            pressed[buttons.GAMEPAD1X.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1X.ordinal()] && !theOpMode.gamepad1.x){
            bounced[buttons.GAMEPAD1X.ordinal()] = true;
            pressed[buttons.GAMEPAD1X.ordinal()] = false;
        }

        if(theOpMode.gamepad1.y){
            pressed[buttons.GAMEPAD1Y.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1Y.ordinal()] && !theOpMode.gamepad1.y){
            bounced[buttons.GAMEPAD1Y.ordinal()] = true;
            pressed[buttons.GAMEPAD1Y.ordinal()] = false;
        }

        if(theOpMode.gamepad1.dpad_down){
            pressed[buttons.GAMEPAD1DPADDOWN.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1DPADDOWN.ordinal()] && !theOpMode.gamepad1.dpad_down){
            bounced[buttons.GAMEPAD1DPADDOWN.ordinal()] = true;
            pressed[buttons.GAMEPAD1DPADDOWN.ordinal()] = false;
        }

        if(theOpMode.gamepad1.dpad_up){
            pressed[buttons.GAMEPAD1DPADUP.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1DPADUP.ordinal()] && !theOpMode.gamepad1.dpad_up){
            bounced[buttons.GAMEPAD1DPADUP.ordinal()] = true;
            pressed[buttons.GAMEPAD1DPADUP.ordinal()] = false;
        }

        if(theOpMode.gamepad1.dpad_left){
            pressed[buttons.GAMEPAD1DPADLEFT.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1DPADLEFT.ordinal()] && !theOpMode.gamepad1.dpad_left){
            bounced[buttons.GAMEPAD1DPADLEFT.ordinal()] = true;
            pressed[buttons.GAMEPAD1DPADLEFT.ordinal()] = false;
        }

        if(theOpMode.gamepad1.dpad_right){
            pressed[buttons.GAMEPAD1DPADRIGHT.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1DPADRIGHT.ordinal()] && !theOpMode.gamepad1.dpad_right){
            bounced[buttons.GAMEPAD1DPADRIGHT.ordinal()] = true;
            pressed[buttons.GAMEPAD1DPADRIGHT.ordinal()] = false;
        }

        if(theOpMode.gamepad2.right_bumper){
            pressed[buttons.GAMEPAD2RB.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2RB.ordinal()] && !theOpMode.gamepad2.right_bumper){
            bounced[buttons.GAMEPAD2RB.ordinal()] = true;
            pressed[buttons.GAMEPAD2RB.ordinal()] = false;
        }

        if(theOpMode.gamepad2.left_bumper){
            pressed[buttons.GAMEPAD2LB.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2LB.ordinal()] && !theOpMode.gamepad2.left_bumper){
            bounced[buttons.GAMEPAD2LB.ordinal()] = true;
            pressed[buttons.GAMEPAD2LB.ordinal()] = false;
        }


        if(theOpMode.gamepad2.a){
            pressed[buttons.GAMEPAD2A.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2A.ordinal()] && !theOpMode.gamepad2.a){
            bounced[buttons.GAMEPAD2A.ordinal()] = true;
            pressed[buttons.GAMEPAD2A.ordinal()] = false;
        }

        if(theOpMode.gamepad2.b){
            pressed[buttons.GAMEPAD2B.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2B.ordinal()] && !theOpMode.gamepad2.b){
            bounced[buttons.GAMEPAD2B.ordinal()] = true;
            pressed[buttons.GAMEPAD2B.ordinal()] = false;
        }

        if(theOpMode.gamepad2.x){
            pressed[buttons.GAMEPAD2X.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2X.ordinal()] && !theOpMode.gamepad2.x){
            bounced[buttons.GAMEPAD2X.ordinal()] = true;
            pressed[buttons.GAMEPAD2X.ordinal()] = false;
        }

        if(theOpMode.gamepad2.y){
            pressed[buttons.GAMEPAD2Y.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2Y.ordinal()] && !theOpMode.gamepad2.y){
            bounced[buttons.GAMEPAD2Y.ordinal()] = true;
            pressed[buttons.GAMEPAD2Y.ordinal()] = false;
        }

        if(theOpMode.gamepad2.dpad_down){
            pressed[buttons.GAMEPAD2DPADDOWN.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2DPADDOWN.ordinal()] && !theOpMode.gamepad2.dpad_down){
            bounced[buttons.GAMEPAD2DPADDOWN.ordinal()] = true;
            pressed[buttons.GAMEPAD2DPADDOWN.ordinal()] = false;
        }

        if(theOpMode.gamepad2.dpad_up){
            pressed[buttons.GAMEPAD2DPADUP.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2DPADUP.ordinal()] && !theOpMode.gamepad2.dpad_up){
            bounced[buttons.GAMEPAD2DPADUP.ordinal()] = true;
            pressed[buttons.GAMEPAD2DPADUP.ordinal()] = false;
        }

        if(theOpMode.gamepad2.dpad_left){
            pressed[buttons.GAMEPAD2DPADLEFT.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2DPADLEFT.ordinal()] && !theOpMode.gamepad2.dpad_left){
            bounced[buttons.GAMEPAD2DPADLEFT.ordinal()] = true;
            pressed[buttons.GAMEPAD2DPADLEFT.ordinal()] = false;
        }

        if(theOpMode.gamepad2.dpad_right){
            pressed[buttons.GAMEPAD2DPADRIGHT.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2DPADRIGHT.ordinal()] && !theOpMode.gamepad2.dpad_right){
            bounced[buttons.GAMEPAD2DPADRIGHT.ordinal()] = true;
            pressed[buttons.GAMEPAD2DPADRIGHT.ordinal()] = false;

        }

        if(theOpMode.gamepad2.right_trigger > 0.5){
            pressed[buttons.GAMEPAD2RIGHTTRIGGER.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2RIGHTTRIGGER.ordinal()] && theOpMode.gamepad2.right_trigger < 0.5){
            bounced[buttons.GAMEPAD2RIGHTTRIGGER.ordinal()] = true;
            pressed[buttons.GAMEPAD2RIGHTTRIGGER.ordinal()] = false;

        }

        if(theOpMode.gamepad1.right_trigger > 0.5){
            pressed[buttons.GAMEPAD1RIGHTTRIGGER.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1RIGHTTRIGGER.ordinal()] && theOpMode.gamepad1.right_trigger < 0.5){
            bounced[buttons.GAMEPAD1RIGHTTRIGGER.ordinal()] = true;
            pressed[buttons.GAMEPAD1RIGHTTRIGGER.ordinal()] = false;

        }

        if(theOpMode.gamepad2.left_trigger > 0.5){
            pressed[buttons.GAMEPAD2LEFTTRIGGER.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD2LEFTTRIGGER.ordinal()] && theOpMode.gamepad2.left_trigger < 0.5){
            bounced[buttons.GAMEPAD2LEFTTRIGGER.ordinal()] = true;
            pressed[buttons.GAMEPAD2LEFTTRIGGER.ordinal()] = false;

        }

        if(theOpMode.gamepad1.left_trigger > 0.5){
            pressed[buttons.GAMEPAD1LEFTTRIGGER.ordinal()] = true;
        }else if (pressed[buttons.GAMEPAD1LEFTTRIGGER.ordinal()] && theOpMode.gamepad1.left_trigger < 0.5){
            bounced[buttons.GAMEPAD1LEFTTRIGGER.ordinal()] = true;
            pressed[buttons.GAMEPAD1LEFTTRIGGER.ordinal()] = false;

        }

    }


}
