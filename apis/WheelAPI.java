package org.firstinspires.ftc.roverruckus.teamcode.apis;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class WheelAPI {
	
	//A motor power less than the dead zone power will be interpreted as a command to stop the motor.
	private static final double DEAD_ZONE = 0.05;
	
	private DcMotor motor;
	
	//PID
	private boolean hasStraightPID;
		private PidAPI straightPid;
	
	private boolean hasRotationPID;
		private PidAPI rotationPid;
	
	//Minimum Speed
	private double minimumSpeed;
	
	//Wheel Measurements
	private double radius;
	private double circumference;
	
	//Encoders
	private double currentPosition;
	private int measurementStartPosition;
	private double targetDistance;
	private double targetPosition;
	private static final double TICKS_PER_ROTATION = 1120;
	
	public WheelAPI(DcMotor motor, double radius) {
		this.motor = motor;
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			
		this.minimumSpeed = 0;
		
		this.radius = radius;
		this.circumference = 2 * Math.PI * radius;
	}
	
	public void setPower(double power) {
		if(Math.abs(power) > minimumSpeed) {
			motor.setPower(power);
		} else if(power < 0) {
			motor.setPower(-minimumSpeed);
		} else {
			motor.setPower(minimumSpeed);
		}
	}
	
	public void setPower(double power, Telemetry telemetry) {
		if(Math.abs(power) > minimumSpeed) {
			motor.setPower(power);
		} else if(power < 0) {
			motor.setPower(-minimumSpeed);
		} else {
			motor.setPower(minimumSpeed);
		}
	}
	
	public void stop() {
		motor.setPower(0);
	}
	
	public void addStraightPID(PidAPI straightPid) {
		this.straightPid = straightPid;
		this.hasStraightPID = this.straightPid != null;
	}
	
	public double getStraightPIDGain() {
		return straightPid.getPGain();
	}
	
	public PidAPI getStraightPID() {
		return straightPid;
	}
	
	public void addRotationPID(PidAPI rotationPid) {
		this.rotationPid = rotationPid;
		this.hasRotationPID = this.rotationPid != null;
	}
	
	public PidAPI getRotationPID() {
		return rotationPid;
	}
	
	public double getRotationPIDGain() {
		return rotationPid.getPGain();
	}
	
	public void driveStraight(double power, double robotOrientation, double targetOrientation, double dt) {
		if(!hasStraightPID) {
			setPower(power);
		} else {
			straightPid.setBias(power);
			setPower(straightPid.getOutput(robotOrientation, targetOrientation, dt));
		}
	}
	
	public void turn(double power, double robotOrientation, double targetOrientation, double dt) {
		if(!hasRotationPID) {
			setPower(power);
		} else {
			rotationPid.setBias(power);
			setPower(rotationPid.getOutput(robotOrientation, targetOrientation, dt));
		}
	}
	
	public void turn(double power, double robotOrientation, double targetOrientation, double dt, Telemetry telemetry) {
		if(!hasRotationPID) {
			setPower(power);
		} else {
			rotationPid.setBias(power);
			setPower(rotationPid.getOutput(robotOrientation, targetOrientation, dt, telemetry));
		}
	}
	
	public void setDirection(DcMotor.Direction direction) {
		motor.setDirection(direction);
	}
	
	public void setMinimumSpeed(double minimumSpeed) {
		this.minimumSpeed = Math.min(Math.max(0, Math.abs(minimumSpeed)), 1);
	}
	
	public double getMinimumSpeed() {
		return minimumSpeed;
	}
	
	public void setRadius(double radius) {
		this.radius = radius;
		this.circumference = 2 * Math.PI * radius;
	}
	
	public double getRadius() {
		return radius;
	}
	
	public double getCircumference() {
		return circumference;
	}
	
	public DcMotor getMotor() {
		return motor;
	}
	
	public double getPower() {
		return motor.getPower();
	}
	
	public int getPosition() {
		return motor.getCurrentPosition() - measurementStartPosition;
	}
	
	public void beginMeasuringDistance() {
		measurementStartPosition = motor.getCurrentPosition();
	}
	
	public double getDistanceTraveled() {
		return ((motor.getCurrentPosition() - measurementStartPosition) / TICKS_PER_ROTATION) * circumference;
	}
}
