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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roverruckus.teamcode.apis.DriveTrainAPI;
import org.firstinspires.ftc.roverruckus.teamcode.apis.GyroscopeAPI;
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

public class EncoderDistanceTest extends LinearOpMode {

	@Override
	public void runOpMode() {

		DriveTrainAPI driveTrain = new DriveTrainAPI(hardwareMap, DriveTrainAPI.FOUR_WHEEL_DRIVE);
		driveTrain.addGyroscope(new GyroscopeAPI(hardwareMap));

		telemetry.addData("Status", "Initialized");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		
		//The drive train ought to proceed the number of inches passed as input in driveTrain.beginMeasuringDistance()
		driveTrain.beginMeasuringDistance(100);
		driveTrain.setDrivingStraight(true);

		// run until the end of the match (driver presses STOP)
		long startTime = System.currentTimeMillis();
		while (opModeIsActive()) {
			if(driveTrain.reachedTargetDistance()) {
				driveTrain.stop();
			} else {
				driveTrain.update(0.4, 0.4, telemetry);
			}
			
			driveTrain.printWheelEncoderDistances(telemetry);
			telemetry.addLine("Distance Traveled: " + driveTrain.getDistanceTraveled() + " inches");
			
			telemetry.addLine("Reached Goal: " + driveTrain.reachedTargetDistance());
			
			
			telemetry.addData("Status", "Running");
			telemetry.update();
		}
	}
}
