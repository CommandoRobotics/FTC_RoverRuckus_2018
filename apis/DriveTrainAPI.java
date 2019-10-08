package org.firstinspires.ftc.roverruckus.teamcode.apis;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Calendar;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;


public class DriveTrainAPI {
    
    //Wheels
    private WheelAPI frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel;
    
    //Drive OpMode
    private boolean fourWheelDrive;
    public static final int TWO_WHEEL_DRIVE = 0;
    public static final int FOUR_WHEEL_DRIVE = 1;
    
    //Encoder
    private double targetDistance;
    
    //Gyroscope
    private boolean hasGyroscope;
        private GyroscopeAPI gyroscope;
    
    //Drivng Straight
    public static final int STRAIGHT_FORWARDS = 1;
    public static final int STRAIGHT_BACKWARDS = 2;
    
    private boolean drivingStraight;
        private int driveStraightDirection;
        private long driveStraightStartTime;
        private double driveStraightStartOrientation;
    
    //Turning to Predetermined Rotation
    private boolean turning;
        private double turnTargetRotation;
        private long turnTime;
        private double turnStartOrientation;
    
    public DriveTrainAPI(HardwareMap hardwareMap, int driveMode) {
        
        fourWheelDrive = driveMode == FOUR_WHEEL_DRIVE;
        
        if(fourWheelDrive) {
            frontLeftWheel = new WheelAPI(hardwareMap.get(DcMotor.class, "front_left_wheel"), 2.5);
            frontLeftWheel.setMinimumSpeed(0.3);
            frontRightWheel = new WheelAPI(hardwareMap.get(DcMotor.class, "front_right_wheel"), 2.5);
            frontRightWheel.setMinimumSpeed(0.3);
            frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        }
        
        rearLeftWheel = new WheelAPI(hardwareMap.get(DcMotor.class, "rear_left_wheel"), 2.5);
            rearLeftWheel.setMinimumSpeed(0.3);
            rearLeftWheel.addStraightPID(new PidAPI(PidAPI.PI_MODE, 0.5, 0.0025, 0.0025, 0.001, 1e9));
            rearLeftWheel.addRotationPID(new PidAPI(PidAPI.P_MODE, 0, 0.012, 0.1, 0.1, 1e9));
        rearRightWheel = new WheelAPI(hardwareMap.get(DcMotor.class, "rear_right_wheel"), 2.5);
            rearRightWheel.setMinimumSpeed(0.3);
            rearRightWheel.addStraightPID(new PidAPI(PidAPI.PI_MODE, 0.5, -0.0025, -0.0025, -0.001, 1e9));
            rearRightWheel.addRotationPID(new PidAPI(PidAPI.P_MODE, 0, -0.012, -0.1, -0.1, 1e9));
            rearRightWheel.setDirection(DcMotor.Direction.REVERSE);
            
        this.drivingStraight = this.turning = false;
    }
    
    public boolean getDrivingStraight() {
        return drivingStraight;
    }
    
    public void setDrivingStraight(boolean b) {
        //If the rest of the function executes when setting drivingStraight to true when it is already true, the start time and start orientation variables will be erroneously reset.
        if(drivingStraight == b) return;
        if(!hasGyroscope) return;
        
        this.drivingStraight = b;

        if(this.drivingStraight) {
            this.turning = false;
            this.driveStraightStartTime = System.nanoTime();
            this.driveStraightStartOrientation = gyroscope.getZ();
        }
    }
    
    public boolean getTurning() {
        return turning;
    }
    
    public void setTurning(boolean b, double targetRotation) {
        //If the rest of the function executes when setting turning to true when it is already true, the start time and start orientation variables will be erroneously reset.
        
        if(turning == b) return;
        if(!hasGyroscope) return;
        
        this.turning = b;
        this.turnTargetRotation = targetRotation;
        
        if(this.turning) {
            this.drivingStraight = false;
            this.turnTime = System.nanoTime();
            this.turnStartOrientation = gyroscope.getZ();
        }
    }
    
    public void setPower(double leftPower, double rightPower) {
        rearLeftWheel.getMotor().setPower(leftPower);
        rearRightWheel.getMotor().setPower(rightPower);
            
            if(fourWheelDrive) {
                frontLeftWheel.getMotor().setPower(leftPower);
                frontRightWheel.getMotor().setPower(rightPower);
            }
    }
    
    public void update(double leftPower, double rightPower, Telemetry telemetry) {
        if(hasGyroscope) gyroscope.update();
        
        if(hasGyroscope && drivingStraight) {
            //How much time has passed since the last update? This is necessary for PID.
            double averagePower = (leftPower + rightPower) / 2;
            double dt = (double) (System.nanoTime() - driveStraightStartTime);
            
            rearLeftWheel.driveStraight(averagePower, gyroscope.getZ(), driveStraightStartOrientation, dt);
            rearRightWheel.driveStraight(averagePower, gyroscope.getZ(), driveStraightStartOrientation, dt);
            
            if(fourWheelDrive) {
                frontLeftWheel.setPower(rearLeftWheel.getPower());
                frontRightWheel.setPower(rearRightWheel.getPower());
            }
        } else if(hasGyroscope && turning) {
            float robotAngle = gyroscope.getZ();
            boolean turningRight = robotAngle > turnTargetRotation;
            
            telemetry.addData("Turning Right", turningRight);
            telemetry.addData("Target Rotation", turnTargetRotation);
            telemetry.addData("Orientation", robotAngle);
            
            double averagePower = (Math.abs(leftPower) + Math.abs(rightPower)) / 2;
            double dt = (double) (System.nanoTime() - turnTime);
            turnTime += dt;
            
            if(turningRight) {
                leftPower = averagePower;
                rightPower = -averagePower;
            } else {
                leftPower = -averagePower;
                rightPower = averagePower;
            }
            
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            
            if(Math.abs(gyroscope.getZ() - turnTargetRotation) < 1) {
                rearLeftWheel.stop();
                rearRightWheel.stop();
                turning = false;
            } else {
                if(turningRight) {
                    rearLeftWheel.getRotationPID().makeControllerGainPositive();
                    rearRightWheel.getRotationPID().makeControllerGainNegative();
                    
                    rearLeftWheel.turn(leftPower, robotAngle, turnTargetRotation, dt);
                    rearRightWheel.turn(rightPower, robotAngle, turnTargetRotation, dt);
                } else {
                    rearLeftWheel.getRotationPID().makeControllerGainPositive();
                    rearRightWheel.getRotationPID().makeControllerGainNegative();
                    
                    rearLeftWheel.turn(leftPower, robotAngle, turnTargetRotation, dt);
                    rearRightWheel.turn(rightPower, robotAngle, turnTargetRotation, dt);
                }
            }
            
            if(fourWheelDrive) {
                frontLeftWheel.getMotor().setPower(rearLeftWheel.getPower());
                frontRightWheel.getMotor().setPower(rearRightWheel.getPower());
            }
        } else {
            rearLeftWheel.setPower(leftPower);
            rearRightWheel.setPower(rightPower);
            
            if(fourWheelDrive) {
                frontLeftWheel.getMotor().setPower(leftPower);
                frontRightWheel.getMotor().setPower(rightPower);
            }
        }
    }
    
    public void stop() {
        rearLeftWheel.stop();
        rearRightWheel.stop();
        
        if(fourWheelDrive) {
            frontLeftWheel.stop();
            frontRightWheel.stop();
        }
        
        this.turning = this.drivingStraight = false;
    }
    
    public double getRearLeftWheelPower() {
        return rearLeftWheel.getPower();
    }
    
    public double getRearRightWheelPower() {
        return rearRightWheel.getPower();
    }
    
    public void beginMeasuringDistance(double targetDistance) {
        this.targetDistance = targetDistance;
        
        rearLeftWheel.beginMeasuringDistance();
        rearRightWheel.beginMeasuringDistance();
        if(fourWheelDrive) {
            frontLeftWheel.beginMeasuringDistance();
            frontRightWheel.beginMeasuringDistance();
        }
    }
    
    public double getDistanceTraveled() {
        double rLDistance = rearLeftWheel.getDistanceTraveled();
        double rRDistance = rearRightWheel.getDistanceTraveled();
        
        if(fourWheelDrive) {
            double fLDistance = frontLeftWheel.getDistanceTraveled();
            double fRDistance = frontRightWheel.getDistanceTraveled();
            
            return (rLDistance + rRDistance + fLDistance + fRDistance) / 4;
        }
        return (rLDistance + rRDistance) / 2;
    }
    
    public boolean reachedTargetDistance() {
        if(targetDistance < 0) return getDistanceTraveled() <= targetDistance;
        else return  getDistanceTraveled() >= targetDistance;
    }
    
    public void printWheelEncoderDistances(Telemetry telemetry) {
        telemetry.addData("Rear Left", rearLeftWheel.getPosition());
        telemetry.addData("Rear Right", rearRightWheel.getPosition());
        if(fourWheelDrive) {
            telemetry.addData("Front Left", frontLeftWheel.getPosition());
            telemetry.addData("Front Right", frontRightWheel.getPosition());
        }
    }
    
    public void addGyroscope(GyroscopeAPI gyroscope) {
        this.gyroscope = gyroscope;
        hasGyroscope = this.gyroscope != null;
    }
    
    public double getOrientation() {
        gyroscope.update();
        return gyroscope.getZ();
    }
}
