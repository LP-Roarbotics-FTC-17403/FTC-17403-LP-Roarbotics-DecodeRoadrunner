package org.firstinspires.ftc.teamcode.SubSystems;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hammer {
    private Servo hammer = null;
    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    public Hammer(HardwareMap hardwareMap, String deviceName){
        this.hammer = hardwareMap.get(Servo.class, deviceName);
    }

    public void initialize(){
        hammer.setPosition(LEFT_POSITION);
    }

    public class Left implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            setLeft();
            return true;
        }
    }
    public Action moveLeft() {
        return new Hammer.Left();
    }

    public class Right implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            setRight();
            return true;
        }
    }

    public Action moveRight(){
        return new Hammer.Right();
    }
    public void setRight(){
        hammer.setPosition(RIGHT_POSITION);
    }

    public void setLeft(){
        hammer.setPosition(LEFT_POSITION);
    }

}
