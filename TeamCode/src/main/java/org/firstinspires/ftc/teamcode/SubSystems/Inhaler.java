package org.firstinspires.ftc.teamcode.SubSystems;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Inhaler {
    private DcMotorEx inhale = null;

    public static final double INHALE_ON                = 1.0;
    public static final double INHALE_OFF               = 0.0;


    public Inhaler(HardwareMap hardwareMap, String deviceName){
        this.inhale = hardwareMap.get(DcMotorEx.class, deviceName);
    }
    public void initialize(boolean reverse){
        if(reverse){
            inhale.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        inhale.setPower(INHALE_OFF);
    }
    public class IntakeOn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            inhale_on();
            return true;
        }
    }
    public Action intaking() {
        return new IntakeOn();
    }

    public class IntakeOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            inhale_off();
            return true;
        }
    }

    public Action notIntaking(){
        return new IntakeOff();
    }
    public void inhale_on(){
        inhale.setPower(INHALE_ON);
    }
    public void inhale_off(){
        inhale.setPower(INHALE_OFF);
    }
}
