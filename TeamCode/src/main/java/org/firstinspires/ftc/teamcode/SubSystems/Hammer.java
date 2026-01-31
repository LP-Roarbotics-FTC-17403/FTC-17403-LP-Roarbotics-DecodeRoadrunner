package org.firstinspires.ftc.teamcode.SubSystems;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hammer {
    private Servo hammer = null;
    final double LEFT_POSITION = 0.3162; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0.035;

    private double closePosition = 0;

    private double openPosition = 0;

    public Hammer(HardwareMap hardwareMap, String deviceName){
        this.hammer = hardwareMap.get(Servo.class, deviceName);
    }

    public void initialize(){
        hammer.setPosition(LEFT_POSITION);
    }


    public void setRight(){
        hammer.setPosition(RIGHT_POSITION);
    }

    public void setLeft(){
        hammer.setPosition(LEFT_POSITION);
    }

    public void setPosition(double closePosition, double openPosition){
        this.closePosition = closePosition;
        this.openPosition = openPosition;
    }

    public Action openGate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                hammer.setPosition(openPosition);

                return false;
            }
        };
    }
    public Action closeGate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                hammer.setPosition(closePosition);

                return false;
            }
        };
    }



    public Action left(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                    hammer.setPosition(LEFT_POSITION);

                return false;
            }
        };
    }
    public Action right(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                    hammer.setPosition(RIGHT_POSITION);

                return false;
            }
        };
    }
}
