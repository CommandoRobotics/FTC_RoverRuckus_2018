package org.firstinspires.ftc.roverruckus.teamcode.apis;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class SamplerAPI {

    private static final float a_red = 0.0003043f;
    private static final float b_red = -0.1145f;
    private static final float c_red = 8.865f;

    private static final float a_green = 0.0008910f;
    private static final float b_green = -0.2260f;
    private static final float c_green = 12.11f;
    
    private static final float a = 2.3541E-4f;
    private static final float b = 0.52767f;
    private static final float c = 16.837f;
    
    private static final float maxSamplingDistance = 7f;

    private float initialRed, initialGreen;

    private LynxI2cColorRangeSensor colorSensor;
    private Servo arm;

    public SamplerAPI(HardwareMap hardwareMap) {
        this.colorSensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "sampling_color_sensor");
        this.arm = hardwareMap.get(Servo.class, "sampling_arm");
        
        initialRed = initialGreen = 30;
    }
    
    public void initialize() {
        initialRed = colorSensor.red();
        initialGreen = colorSensor.green();
        colorSensor.enableLed(false);
    }
    
    public void dropArm() {
        arm.setPosition(0.53);
    }
    
    public void liftArm() {
        arm.setPosition(1.0);
    }
    
    public boolean seeingGold() {
        float red = colorSensor.red();
        float green = colorSensor.green();
        float blue = colorSensor.blue();
        
        float expectedGreen = getExpectedGreen(red);
        
        if(red < 30) {
            return false;
        } else if(red < 35) {
            boolean condition = (blue - red < -5);
            return condition;
        } else if(red < 45) {
            boolean condition = (blue - red < -10);
            return condition;
        } else {
            boolean condition = red > green * 1.05f;
            return condition;
         }
    }
    
    public boolean seeingGold(Telemetry telemetry) {
        float red = colorSensor.red();
        float green = colorSensor.green();
        float blue = colorSensor.blue();
        
        float expectedGreen = getExpectedGreen(red);
        
        if(red < 30) {
            telemetry.addLine("Too far to determine; returning false.");
            return false;
        } else if(red < 35) {
            boolean condition = (blue - red < -5); //3
            telemetry.addLine("Red < 35; Blue = " + blue + "; Green - red = " + (green - red) + "; Returning " + !condition);
            return condition;
        } else if(red < 45) {
            boolean condition = (blue - red < -10); //3
            telemetry.addLine("Red < 45; Blue - red = " + (blue - red) + "; Green - red = " + (green - red) + "; Returning " + !condition);
            return condition;
        } else {
            boolean condition = red > green * 1.05f;
            telemetry.addLine("Red > 45; Red --> " + red + " > " + (green * 0.98) + "; Returning " + !condition);
            return condition;
         }
        //MR color sensor
    }
    
    private float getExpectedGreen(float red) {
        return a * red * red + b * red + c;
    }
    
    private float quadratic(float a, float b, float c) {
        float discriminant = (b * b) - (4f * a * c);
        if(discriminant < 0) return 0;
        else if(discriminant == 0) return (-b / (2f * a));
        else return ((-b + discriminant) / (2f * a));
    }
    
    private float approximateDistanceFromRed(float red) {
        return a_red * red * red + b_red * red + c_red;
    }
    
    private float approximateDistanceFromGreen(float green) {
        return a_green * green * green + b_green * green + c_green;
    }
    
    public void printColor(Telemetry telemetry) {
        telemetry.addLine("Color: " + colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
        telemetry.addLine("Range " + colorSensor.getRawLightDetected());
    }

}
