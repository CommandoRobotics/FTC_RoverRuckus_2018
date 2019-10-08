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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roverruckus.teamcode.apis.DriveTrainAPI;
import org.firstinspires.ftc.roverruckus.teamcode.apis.SamplerAPI;

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

public class SamplingTest extends LinearOpMode {
	
	private static final int SAMPLE_START_DELAY = 500;
	
	@Override
	public void runOpMode() {

		DriveTrainAPI driveTrain = new DriveTrainAPI(hardwareMap, DriveTrainAPI.TWO_WHEEL_DRIVE);
		SamplerAPI sampler = new SamplerAPI(hardwareMap);

		telemetry.addLine("Initialized stuffs");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)

		waitForStart();
		
		telemetry.addLine("Started program");
		telemetry.update();

		//Variables for sampling step
		long samplingStart = 0, armDropStart = 0;
		boolean hasDroppedArm = false;

		// run until the end of the match (driver presses STOP)
		telemetry.addLine("Starting loop");
		
		int goldCount = 0;
		driveTrain.update(0.18, 0.18, telemetry);
		
		samplingStart = System.currentTimeMillis();
		long samplingEndTime = 5000;
		while(System.currentTimeMillis() - samplingStart < samplingEndTime && opModeIsActive()) {
			if(hasDroppedArm) {
				long timeSinceArmDrop = System.currentTimeMillis() - armDropStart;
				
				telemetry.addLine("Time Since Arm Drop: " + timeSinceArmDrop);
				
				if(timeSinceArmDrop > SAMPLE_START_DELAY + 800) {
					driveTrain.update(0.18, 0.18, telemetry);
					telemetry.addLine("Go forwards.");
				} else if(timeSinceArmDrop > SAMPLE_START_DELAY) {
					driveTrain.update(-0.18, -0.18, telemetry);
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
					samplingEndTime += 4000;
				}
			} else {
				goldCount = 0;
			}
		}
		
		sampler.liftArm();
		driveTrain.stop();
		telemetry.addLine("Stopped Robot");
	}
}
