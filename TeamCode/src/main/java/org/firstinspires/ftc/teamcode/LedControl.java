package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

/**
 * Created by Techno Team_PC_III on 4/23/2017.
 */

public class LedControl {

    LED                   led1;
    LED                   led2;
    LED                   led3;
    LED                   led4;
    LED                   led5;
    LED                   led6;
    LED                   led7;

    final boolean X = true;
    final boolean o = false;

    int pattNum = 0;

    double startTime;
    int lastInterval = -1;

    boolean ledsOff[] = {o, o, o, o, o, o, o};

    boolean patterns[][][] = {
            {
                    {o, o, o, o, o, o, o},
                    {X, o, o, o, o, o, X},
                    {X, X, o, o, o, X, X},
                    {X, X, X, o, X, X, X},
                    {X, X, X, X, X, X, X},
                    {X, X, X, o, X, X, X},
                    {X, X, o, o, o, X, X},
                    {X, o, o, o, o, o, X}
            },

            {
                    {o, o, o, o, o, o, o},
                    {X, o, X, o, X, o, X},
                    {o, X, o, X, o, X, o}
            },

            {
                    {o, o, o, o, o, o, o},
                    {X, o, o, o, o, o, o},
                    {X, X, o, o, o, o, o},
                    {X, X, X, o, o, o, o},
                    {X, X, X, X, o, o, o},
                    {X, X, X, X, X, o, o},
                    {X, X, X, X, X, X, o},
                    {X, X, X, X, X, X, X},
                    {X, X, X, X, X, X, o},
                    {X, X, X, X, X, o, o},
                    {X, X, X, X, o, o, o},
                    {X, X, X, o, o, o, o},
                    {X, X, o, o, o, o, o},
                    {X, o, o, o, o, o, o},
            },

            {
                    {o, o, o, o, o, o, o},
                    {X, o, o, o, o, o, o},
                    {o, X, o, o, o, o, o},
                    {o, o, X, o, o, o, o},
                    {o, o, o, X, o, o, o},
                    {o, o, o, o, X, o, o},
                    {o, o, o, o, o, X, o},
                    {o, o, o, o, o, o, X},
            },

    };

    double intervals[] = {.1, .1, .05, .075};


    public void init(HardwareMap hardwareMap) {
        led1 = hardwareMap.get(LED.class, "led 1");
        led2 = hardwareMap.get(LED.class, "led 2");
        led3 = hardwareMap.get(LED.class, "led 3");
        led4 = hardwareMap.get(LED.class, "led 4");
        led5 = hardwareMap.get(LED.class, "led 5");
        led6 = hardwareMap.get(LED.class, "led 6");
        led7 = hardwareMap.get(LED.class, "led 7");

        displayPattern(ledsOff);
    }

    void displayPattern(boolean[] b) {
        led1.enable(b[0]);
        led2.enable(b[1]);
        led3.enable(b[2]);
        led4.enable(b[3]);
        led5.enable(b[4]);
        led6.enable(b[5]);
        led7.enable(b[6]);
    }

    void update(double currentTime){
        int currentInterval = (int)((currentTime - startTime)/intervals[pattNum]);
        if(currentInterval != lastInterval) {
            int step = currentInterval % patterns[pattNum].length; //8;
            lastInterval = currentInterval;
            displayPattern(patterns[pattNum][step]);
        }
    }

    }
