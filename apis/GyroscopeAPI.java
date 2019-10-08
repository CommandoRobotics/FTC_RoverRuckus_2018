/*
Copyright 2018 FIRST Tech Challenge Team 9898a

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.roverruckus.teamcode.apis;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class GyroscopeAPI {
    private BNO055IMU imu;
    private float xAngle, yAngle, zAngle;

    public GyroscopeAPI(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode    = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        
        this.xAngle = this.yAngle = this.zAngle = 0;
    }

    public void update() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        
        float startX = this.xAngle, startY = this.yAngle, startZ = this.zAngle;
        
        this.xAngle = getRelativeClosestAngle(this.xAngle, orientation.firstAngle);
        this.yAngle = getRelativeClosestAngle(this.yAngle, orientation.secondAngle);
        this.zAngle = getRelativeClosestAngle(this.zAngle, orientation.thirdAngle);
    }
    
    private float getRelativeClosestAngle(float currentAngle, float targetAngle) {
        while(targetAngle - currentAngle >= 360) {
            targetAngle -= 360;
        }
        
        while(targetAngle - currentAngle <= -360) {
            targetAngle += 360;
        }
        
        if(targetAngle != currentAngle) {
            float negativeDirectionTargetAngle = 0;
            float positiveDirectionTargetAngle = 0;
            
            if(targetAngle < currentAngle) {
                negativeDirectionTargetAngle = targetAngle;
                positiveDirectionTargetAngle = targetAngle + 360;
            } else {
                negativeDirectionTargetAngle = targetAngle - 360;
                positiveDirectionTargetAngle = targetAngle;
            }
            float negativeDirectionDistance = negativeDirectionTargetAngle - currentAngle;
            float positiveDirectionDistance = positiveDirectionTargetAngle - currentAngle;
            
            if(Math.abs(positiveDirectionDistance) < Math.abs(negativeDirectionDistance)) return positiveDirectionTargetAngle;
            else return negativeDirectionTargetAngle;
        }
        
        return targetAngle;
    }
    
    public float getX() {
        return xAngle;
    }
    
    public float getY() {
        return yAngle;
    }
    
    public float getZ() {
        return zAngle;
    }
}
