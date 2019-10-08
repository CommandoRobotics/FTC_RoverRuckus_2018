package org.firstinspires.ftc.roverruckus.teamcode.apis;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ClaimerAPI {

    private DcMotor claimer;
    
    public ClaimerAPI(HardwareMap hardwareMap) {
        claimer = hardwareMap.get(DcMotor.class, "claimer");
    }
    
    public void dropArm(double power) {
        claimer.setPower(Math.abs(power));
    }
    
    public void liftArm(double power) {
        claimer.setPower(-Math.abs(power));
    }
    
    public void stopArm() {
        claimer.setPower(0);
    }

}
