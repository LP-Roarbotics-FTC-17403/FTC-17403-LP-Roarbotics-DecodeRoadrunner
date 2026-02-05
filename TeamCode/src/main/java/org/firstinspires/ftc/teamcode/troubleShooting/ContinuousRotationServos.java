package org.firstinspires.ftc.teamcode.troubleShooting;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ContinuousRotationServos {
    private CRServo  crServo= null;
    public ContinuousRotationServos(HardwareMap hardwareMap, String deviceName){
        this.crServo = hardwareMap.get(CRServo.class, deviceName);
    }

    public void initialize(double power){
        crServo.setPower(power);
    }

    public double getCurrentPower(){
        return crServo.getPower();
    }

}
