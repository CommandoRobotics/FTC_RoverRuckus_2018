package org.firstinspires.ftc.roverruckus.teamcode.apis;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class LanderAPI {

    private DcMotor lift;
    private DcMotor arm;

    public LanderAPI(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotor.class, "lander_lift");
        arm = hardwareMap.get(DcMotor.class, "lander_arm");
    }
    
    public void raiseLift(double power) {
        lift.setPower(Math.abs(power));
    }
    
    public void lowerLift(double power) {
        lift.setPower(-Math.abs(power));
    }
    
    public void stopLift() {
        lift.setPower(0);
    }
    
    public void extendArm() {
        arm.setPower(0.3);
    }
    
    public void retractArm() {
        arm.setPower(-0.3);
    }
    
    public void stopArm() {
        arm.setPower(0);
    }

}
