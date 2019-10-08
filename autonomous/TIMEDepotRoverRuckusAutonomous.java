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
package org.firstinspires.ftc.roverruckus.teamcode.autonomous;

import org.firstinspires.ftc.roverruckus.teamcode.apis.*;
import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous
@Disabled
public class TIMEDepotRoverRuckusAutonomous extends LinearOpMode {

	private static final int SAMPLE_TIME_ADDITION = 4000;
	private static final int SAMPLE_START_DELAY = 500;

	@Override
	public void runOpMode() {
		
		DriveTrainAPI driveTrain = new DriveTrainAPI(hardwareMap, DriveTrainAPI.FOUR_WHEEL_DRIVE);
		GyroscopeAPI gyroscope = new GyroscopeAPI(hardwareMap);
			driveTrain.addGyroscope(gyroscope);
		SamplerAPI sampler = new SamplerAPI(hardwareMap);
		LanderAPI lander = new LanderAPI(hardwareMap);
		ClaimerAPI claimer = new ClaimerAPI(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		sampler.liftArm();

		//LOWER THE ROBOT OFF OF THE LANDER
		telemetry.addLine("Lower robot off of lander");
		telemetry.update();
		long raiseStartTime = System.currentTimeMillis();
		lander.raiseLift(0.5);
		while(System.currentTimeMillis() - raiseStartTime < 1200 && opModeIsActive()) {
			//Keep raising the lift
		}
		lander.stopLift();

		//DRIVE FORWARD A TINY BIT
		telemetry.addLine("Drive forward a tiny bit");
		telemetry.update();
		long driveAwayFromLanderStartTime = System.currentTimeMillis();
		driveTrain.setPower(0.12, 0.12);
		while(System.currentTimeMillis() - driveAwayFromLanderStartTime < 300 && opModeIsActive()) {
			//Keep driving forward into the handle on the lander
		}
		driveTrain.stop();

		//DRIVE BACK INTO THE LANDER TO STRAIGHTEN UP
		telemetry.addLine("Drive back into lander");
		telemetry.update();
		long driveIntoLanderStartTime = System.currentTimeMillis();
		driveTrain.setPower(-0.12, -0.12);
		while(System.currentTimeMillis() - driveIntoLanderStartTime < 500 && opModeIsActive()) {
			//Keep driving back into the lander
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT TO LET GYROSCOPE READINGS STABILIZE
		//WHILE WE'RE STAYING STILL, WE CAN RETRACT THE ARM
		telemetry.addLine("Stay still 1");
		telemetry.update();
		long stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 1000) {
			lander.retractArm();
		}
		
		//TAKE NOTE OF THE ANGLE AT WHICH THE ROBOT STARTS
		double startAngle = gyroscope.getZ();
		telemetry.addData("Start Angle", startAngle);
		
		//DRIVE AWAY FROM THE LANDER TO THE SAMPLING ROW
		telemetry.addLine("Drive away from lander");
		telemetry.update();
		driveTrain.setDrivingStraight(true);
		driveAwayFromLanderStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - driveAwayFromLanderStartTime < 750 && opModeIsActive()) {
			driveTrain.update(0.30, 0.30, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 2");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 200) {
			driveTrain.stop();
		}
		
		//TURN 60 DEGREES
		telemetry.addLine("Turn 50 degrees");
		telemetry.addData("Turn angle", (startAngle + 50));
		telemetry.update();
		driveTrain.setTurning(true, startAngle + 50);
		while(driveTrain.getTurning() && opModeIsActive()) {
			driveTrain.update(-0.35, 0.35, telemetry);
			telemetry.update();
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 3");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 200) {
			
		}
		
		//COME FORWARD MORE
		telemetry.addLine("Come forward more");
		telemetry.update();
		driveTrain.setDrivingStraight(true);
		driveAwayFromLanderStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - driveAwayFromLanderStartTime < 600 && opModeIsActive()) {
			driveTrain.update(0.30, 0.30, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 4");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 200 && opModeIsActive()) {
			
		}
		
		//TURN PARALLEL TO THE SAMPLING ROW
		telemetry.addLine("Turn parallel to sampling row");
		telemetry.addData("Turn angle", (startAngle + 92));
		telemetry.update();
		driveTrain.setTurning(true, startAngle + 92);
		while(driveTrain.getTurning() && opModeIsActive()) {
			driveTrain.update(-0.35, 0.35, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 5");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 500 && opModeIsActive()) {
			
		}
		
		//BACK UP TO THE BEGINNING OF THE SAMPLING ROW
		telemetry.addLine("Back up to beginning of sampling row");
		telemetry.update();
		long driveBackwardsStartTime = System.currentTimeMillis();
		driveTrain.setDrivingStraight(true);
		driveTrain.update(-0.30, -0.30, telemetry);
		while(System.currentTimeMillis() - driveBackwardsStartTime < 1750 && opModeIsActive()) {
			driveTrain.update(-0.30, -0.30, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 6");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 500) {
			//Stay still. Contemplate.
		}
		
		//CORRECT ROTATION TO BE PARALLEL TO THE SAMPLING ROW
		telemetry.addLine("Turn parallel to sampling row");
		telemetry.addData("Turn angle", (startAngle + 100));
		telemetry.update();
		driveTrain.setTurning(true, startAngle + 85);
		while(driveTrain.getTurning() && opModeIsActive()) {
			driveTrain.update(0.35, 0.35, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 7");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 200) {
			//Stay still. Be intropsective.
		}
		
		//SAMPLE
		boolean didSample = false;
		telemetry.addLine("Sample");
		telemetry.update();
		driveTrain.setDrivingStraight(false);
		long samplingStart = 0, armDropStart = 0, samplingEndTime = 5000;
		boolean hasDroppedArm = false;
		int goldCount = 0;
		driveTrain.setPower(0.18, 0.18);
		
		samplingStart = System.currentTimeMillis();
		while(System.currentTimeMillis() - samplingStart < samplingEndTime && opModeIsActive()) {
			if(hasDroppedArm) {
				long timeSinceArmDrop = System.currentTimeMillis() - armDropStart;
				
				telemetry.addLine("Time Since Arm Drop: " + timeSinceArmDrop);
				
				if(timeSinceArmDrop > SAMPLE_START_DELAY + 800) {
					driveTrain.setPower(0.18, 0.18);
					telemetry.addLine("Go forwards.");
				} else if(timeSinceArmDrop > SAMPLE_START_DELAY) {
					driveTrain.setPower(-0.18, -0.18);
					telemetry.addLine("Go backwards.");
					sampler.dropArm();
				}
				
				if(timeSinceArmDrop > SAMPLE_START_DELAY + 2400) {
					sampler.liftArm();
					telemetry.addLine("Lift the arm.");
				}
				
				telemetry.update();
			} else if(sampler.seeingGold()) {
				goldCount++;
				if(goldCount >= 3) {
					driveTrain.stop();
					hasDroppedArm = true;
					armDropStart = System.currentTimeMillis();
					samplingEndTime += SAMPLE_TIME_ADDITION;
					didSample = true;
				}
			} else {
				goldCount = 0;
			}
		}
		
		sampler.liftArm();
		
		if(!didSample) {
			//COME FORWARD MORE
			telemetry.addLine("Did not sample; come forward more");
			telemetry.update();
			driveTrain.setDrivingStraight(true);
			long didNotSampleCorrectionStartTime = System.currentTimeMillis();
			while(System.currentTimeMillis() - didNotSampleCorrectionStartTime < SAMPLE_TIME_ADDITION - SAMPLE_START_DELAY && opModeIsActive()) {
				driveTrain.update(0.18, 0.18, telemetry);
			}
			driveTrain.stop();
		}
		
		driveTrain.stop();
		
		//TURN TO FACE WALL
		telemetry.addLine("Turn to face wall");
		telemetry.addData("Turn angle", (startAngle - 30));
		telemetry.update();
		driveTrain.setTurning(true, startAngle - 30);
		while(driveTrain.getTurning() && opModeIsActive()) {
			driveTrain.update(0.40, -0.40, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 7");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 200) {
			//Stay still. Be intropsective.
		}
		
		//DRIVE TOWARDS WALL
		long driveToWallStartTime = System.currentTimeMillis();
		driveTrain.setDrivingStraight(true);
		driveTrain.update(0.40, 0.40, telemetry);
		while(System.currentTimeMillis() - driveToWallStartTime < 550 && opModeIsActive()) {
			driveTrain.update(0.40, 0.40, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 8");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 200) {
			//Stay still. Be intropsective.
		}
		
		//TURN TO THE DEPOT
		telemetry.addLine("Turn to face depot");
		telemetry.addData("Turn angle", (startAngle - 45));
		telemetry.update();
		driveTrain.setTurning(true, startAngle - 45);
		while(driveTrain.getTurning() && opModeIsActive()) {
			driveTrain.update(0.40, -0.40, telemetry);
		}
		driveTrain.stop();
		
		//STAY STILL FOR A MOMENT
		telemetry.addLine("Stay still 9");
		telemetry.update();
		stayStillStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - stayStillStartTime < 200) {
			//Stay still. Be intropsective.
		}
		
		//DRIVE TO DEPOT
		long driveToDepotStartTime = System.currentTimeMillis();
		driveTrain.setDrivingStraight(true);
		driveTrain.update(0.40, 0.40, telemetry);
		while(System.currentTimeMillis() - driveToDepotStartTime < 2100 && opModeIsActive()) {
			driveTrain.update(0.40, 0.40, telemetry);
		}
		driveTrain.stop();
		
		//CLAIM
		long claimStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - claimStartTime < 500 && opModeIsActive()) {
			claimer.dropArm(0.4);
		}
		claimer.stopArm();
		
		long claimPauseStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - claimPauseStartTime < 100 && opModeIsActive()) {
			claimer.stopArm();
		}
		
		long claimEndStartTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - claimEndStartTime < 300 && opModeIsActive()) {
			claimer.liftArm(0.3);
		}
		claimer.stopArm();
		
		//DRIVE TO CRATER
		long driveToCraterStartTime = System.currentTimeMillis();
		driveTrain.setDrivingStraight(true);
		driveTrain.update(-0.55, -0.55, telemetry);
		while(System.currentTimeMillis() - driveToCraterStartTime < 4100 && opModeIsActive()) {
			driveTrain.update(-0.55, -0.55, telemetry);
		}
		driveTrain.stop();
	}
}
