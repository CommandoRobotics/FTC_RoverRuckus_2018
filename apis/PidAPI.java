package org.firstinspires.ftc.roverruckus.teamcode.apis;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class PidAPI {
	
	//The PID algorithm changes its output based on which mode it is calculating in: Proportional Integral (PI) mode or Proportional Derivative (PD) mode
	private int currentMode;
	public static final int P_MODE = 0;
	public static final int PI_MODE = 1;
	public static final int PD_MODE = 2;
	
	//Bias is the output value given no error.
	private double bias, pControllerGain, iControllerGain, dControllerGain, timeConstant;
	
	private double previousError, previousTime;
	
	public PidAPI(int mode, double bias, double pControllerGain, double iControllerGain, double dControllerGain, double timeConstant) {
		this.bias = bias;
		this.pControllerGain = pControllerGain;
		this.iControllerGain = iControllerGain;
		this.dControllerGain = dControllerGain;
		this.timeConstant = timeConstant;
		
		this.previousError = this.previousTime = 0;
		
		this.currentMode = mode;
	}
	
	public void setMode(int mode) {
		this.currentMode = mode;
	}
	
	public void setBias(double bias) {
		this.bias = bias;
	}
	
	public double getPGain() {
		return pControllerGain;
	}
	
	public void makeControllerGainPositive() {
		pControllerGain = Math.abs(pControllerGain);
	}
	
	public void makeControllerGainNegative() {
		pControllerGain = -Math.abs(pControllerGain);
	}
	
	public double getOutput(double currentValue, double targetValue, double dt) {
		if(currentMode == P_MODE) return pOutput(currentValue, targetValue);
		else if(currentMode == PI_MODE) return piOutput(currentValue, targetValue, dt);
		else if(currentMode == PD_MODE) return pdOutput(currentValue, targetValue, dt);
		
		return bias;
	}
	
	public double getOutput(double currentValue, double targetValue, double dt, Telemetry telemetry) {
		if(currentMode == P_MODE) {
			telemetry.addLine("P_MODE");
			return pOutput(currentValue, targetValue, telemetry);
		} else if(currentMode == PI_MODE) {
			telemetry.addLine("PI_MODE");
			return piOutput(currentValue, targetValue, dt, telemetry);
		} else if(currentMode == PD_MODE) {
			telemetry.addLine("PD_MODE");
			return pdOutput(currentValue, targetValue, dt, telemetry);
		}
		
		return bias;
	}
	
	public double pOutput(double currentValue, double targetValue) {
		double error = currentValue - targetValue;
		return bias + (pControllerGain * error);
	}
	
	public double pOutput(double currentValue, double targetValue, Telemetry telemetry) {
		double error = currentValue - targetValue;
		telemetry.addLine("P Output");
		telemetry.addLine("Bias: " + bias);
		telemetry.addLine("Current: " + currentValue);
		telemetry.addLine("Target: " + targetValue);
		telemetry.addLine("Error: " + error);
		telemetry.addLine("Alteration: " + (pControllerGain * error));
		telemetry.addLine("Final Power: " + (bias + (pControllerGain * error)));
		
		return bias + (pControllerGain * error);
	}
	
	public double piOutput(double currentValue, double targetValue, double dt) {
		double error = currentValue - targetValue;
		
		double errorDuration;
		
		if((previousError < 0 && error < 0) || (previousError > 0 && error > 0)) {
			errorDuration = previousTime + dt;
		} else {
			errorDuration = dt;
		}
		
		previousError = error;
		previousTime = errorDuration;
		
		return bias + (pControllerGain * error) + ((iControllerGain / timeConstant) * (error * errorDuration));
	}
	
	public double piOutput(double currentValue, double targetValue, double dt, Telemetry telemetry) {
		double error = currentValue - targetValue;
		
		double errorDuration;
		
		if((previousError < 0 && error < 0) || (previousError > 0 && error > 0)) {
			errorDuration = previousTime + dt;
		} else {
			errorDuration = dt;
		}
		
		previousError = error;
		previousTime = errorDuration;
		
		return bias + (pControllerGain * error) + ((iControllerGain / timeConstant) * (error * errorDuration));
	}
	
	public double pdOutput(double currentValue, double targetValue, double dt) {
		double error = currentValue - targetValue;
		
		double errorChange = (error - previousError) / dt;
		
		previousError = error;
		previousTime += dt;
		
		return bias + (pControllerGain * error) + (dControllerGain * (1 + error));
	}
	
	public double pdOutput(double currentValue, double targetValue, double dt, Telemetry telemetry) {
		double error = currentValue - targetValue;
		
		double errorChange = (error - previousError);
		
		telemetry.addData("Error", error);
		telemetry.addData("Time Step", dt);
		telemetry.addData("Error Change", errorChange);
		
		previousError = error;
		previousTime += dt;
		
		return bias + (pControllerGain * error) + (dControllerGain * (1 + errorChange));
	}
}
