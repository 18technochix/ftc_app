package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 1/13/2018.
 */

public class AutoRelicEvent {
    double startTime;
    double endTime;
    Hardware.Servos servo;
    double value;
    boolean done;

    public AutoRelicEvent(double _startTime, double _endTime, Hardware.Servos _servo, double _value) {
        startTime = _startTime;
        endTime = _endTime;
        servo = _servo;
        value = _value;
        done = false;
    }


}
