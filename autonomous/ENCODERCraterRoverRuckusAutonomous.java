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

public class ENCODERCraterRoverRuckusAutonomous extends LinearOpMode {

    private static final int SAMPLE_TIME_ADDITION = 4000;
    private static final int SAMPLE_START_DELAY = 1000;

    @Override
    public void runOpMode() {
        
        DriveTrainAPI driveTrain = new DriveTrainAPI(hardwareMap, DriveTrainAPI.FOUR_WHEEL_DRIVE);
        GyroscopeAPI gyroscope = new GyroscopeAPI(hardwareMap);
            driveTrain.addGyroscope(gyroscope);
        SamplerAPI sampler = new SamplerAPI(hardwareMap);
        LanderAPI lander = new LanderAPI(hardwareMap);
        ClaimerAPI claimer = new ClaimerAPI(hardwareMap);

        while(!opModeIsActive()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        
        sampler.liftArm();

        //LOWER THE ROBOT OFF OF THE LANDER
        telemetry.addLine("Lower robot off of lander");
        telemetry.update();
        long raiseStartTime = System.currentTimeMillis();
        lander.raiseLift(0.5);
        while(System.currentTimeMillis() - raiseStartTime < 1700 && opModeIsActive()) {
            //Keep raising the lift
        }
        lander.stopLift();

        //DRIVE FORWARD A TINY BIT
        telemetry.addLine("Drive forward a tiny bit");
        telemetry.update();
        driveTrain.beginMeasuringDistance(1);
        driveTrain.setPower(0.30, 0.30);
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
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
        driveTrain.beginMeasuringDistance(14);
        driveTrain.setDrivingStraight(true);
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
            driveTrain.update(0.30, 0.30, telemetry);
        }
        driveTrain.stop();
        
        //STAY STILL FOR A MOMENT
        telemetry.addLine("Stay still 2");
        telemetry.update();
        stayStillStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - stayStillStartTime < 100) {
            driveTrain.stop();
        }
        
        //TURN TO FACE THE LANDER
        telemetry.addLine("Turn to face lander");
        telemetry.addData("Turn angle", (startAngle + 98));
        telemetry.update();
        driveTrain.setTurning(true, startAngle + 98);
        while(driveTrain.getTurning() && opModeIsActive()) {
            driveTrain.update(-0.35, 0.35, telemetry);
            telemetry.update();
        }
        driveTrain.stop();
        
        //STAY STILL FOR A MOMENT
        telemetry.addLine("Stay still 3");
        telemetry.update();
        stayStillStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - stayStillStartTime < 100) {
            
        }
        
        //COME BACKWARDS TO THE SAMPLING ROW
        telemetry.addLine("Come backwards to the sampling row");
        telemetry.update();
        driveTrain.beginMeasuringDistance(-28);
        driveTrain.setDrivingStraight(true);
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
            driveTrain.update(-0.30, -0.30, telemetry);
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
        telemetry.addData("Turn angle", (startAngle + 90));
        telemetry.update();
        driveTrain.setTurning(true, startAngle + 90);
        while(driveTrain.getTurning() && opModeIsActive()) {
            driveTrain.update(-0.35, 0.35, telemetry);
        }
        driveTrain.stop();
        
        //STAY STILL FOR A MOMENT
        telemetry.addLine("Stay still 5");
        telemetry.update();
        stayStillStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - stayStillStartTime < 200 && opModeIsActive()) {
            
        }
        
        //BACK UP TO THE BEGINNING OF THE SAMPLING ROW
        telemetry.addLine("Back up to beginning of sampling row");
        telemetry.update();
        long driveBackwardsStartTime = System.currentTimeMillis();
        driveTrain.beginMeasuringDistance(-5);
        driveTrain.setDrivingStraight(true);
        driveTrain.update(-0.30, -0.30, telemetry);
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
            driveTrain.update(-0.30, -0.30, telemetry);
        }
        driveTrain.stop();
        
        //STAY STILL FOR A MOMENT
        telemetry.addLine("Stay still 6");
        telemetry.update();
        stayStillStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - stayStillStartTime < 200) {
            //Stay still. Contemplate.
        }
        
        //CORRECT ROTATION TO BE PARALLEL TO THE SAMPLING ROW
        telemetry.addLine("Turn parallel to sampling row");
        telemetry.addData("Turn angle", (startAngle + 85));
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
        while(System.currentTimeMillis() - stayStillStartTime < 100) {
            //Stay still. Be intropsective.
        }
        
        //SAMPLE
        boolean didSample = false;
        telemetry.addLine("Sample");
        telemetry.update();
        driveTrain.setDrivingStraight(false);
        long samplingStart = 0, goldTargetStart = 0, samplingEndTime = 5000;
        boolean hasSeenGold = false, hasDroppedArm = false, hasCompletedSample = false;
        int goldCount = 0;
        
        driveTrain.beginMeasuringDistance(54);
        driveTrain.setPower(0.20, 0.20);
        
        samplingStart = System.currentTimeMillis();
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
            telemetry.addLine("Distance Traveled: " + driveTrain.getDistanceTraveled());
            if(hasSeenGold) {
                if(hasDroppedArm && System.currentTimeMillis() - goldTargetStart > 1850) {
                    driveTrain.setPower(0.25, 0.25);
                    sampler.liftArm();
                    telemetry.addLine("Lift the arm.");
                    hasCompletedSample = true;
                } else if(System.currentTimeMillis() - goldTargetStart > 750 && !hasCompletedSample) {
                    driveTrain.setPower(-0.25, -0.25);
                    telemetry.addLine("Go backwards.");
                    sampler.dropArm();
                    hasDroppedArm = true;
                } else if(!hasDroppedArm) {
                    driveTrain.setPower(0.25, 0.25);
                }
            } else if(sampler.seeingGold()) {
                goldCount++;
                if(goldCount >= 4) {
                    hasSeenGold = true;
                    goldTargetStart = System.currentTimeMillis();
                    samplingEndTime += SAMPLE_TIME_ADDITION;
                }
            } else if(driveTrain.getDistanceTraveled() > 37 && !hasSeenGold) {
                hasSeenGold = true;
                goldTargetStart = System.currentTimeMillis();
                samplingEndTime += SAMPLE_TIME_ADDITION;
                didSample = true;
            } else {
                goldCount = 0;
            }
        
            telemetry.update();
        }
        
        sampler.liftArm();
        
        driveTrain.stop();
        
        //DRIVE TOWARDS WALL
        driveTrain.beginMeasuringDistance(22);
        driveTrain.setDrivingStraight(true);
        driveTrain.update(0.40, 0.40, telemetry);
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
            driveTrain.update(0.40, 0.40, telemetry);
        }
        driveTrain.stop();
        
        //STAY STILL FOR A MOMENT
        telemetry.addLine("Stay still 8");
        telemetry.update();
        stayStillStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - stayStillStartTime < 100) {
            //Stay still. Be intropsective.
        }
        
        //TURN TO THE DEPOT
        telemetry.addLine("Turn to face depot");
        telemetry.addData("Turn angle", (startAngle + 125));
        telemetry.update();
        driveTrain.setTurning(true, startAngle + 125);
        while(driveTrain.getTurning() && opModeIsActive()) {
            driveTrain.update(0.40, -0.40, telemetry);
        }
        driveTrain.stop();
        
        //STAY STILL FOR A MOMENT
        telemetry.addLine("Stay still 10");
        telemetry.update();
        stayStillStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - stayStillStartTime < 100) {
            //Stay still. Be intropsective.
        }
        
        //DRIVE TO DEPOT
        driveTrain.beginMeasuringDistance(42);
        driveTrain.setDrivingStraight(true);
        driveTrain.update(0.55, 0.55, telemetry);
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
            driveTrain.update(0.55, 0.55, telemetry);
        }
        driveTrain.stop();
        
        //CLAIM
        long claimStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - claimStartTime < 1000 && opModeIsActive()) {
            claimer.dropArm(0.4);
        }

        long claimEndStartTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - claimEndStartTime < 700 && opModeIsActive()) {
            claimer.liftArm(0.4);
        }
        claimer.stopArm();
        
        //FACE THE CRATER
        telemetry.addLine("Turn to face crater");
        telemetry.addData("Turn angle", (startAngle - 180));
        telemetry.update();
        driveTrain.setTurning(true, startAngle - 180);
        while(driveTrain.getTurning() && opModeIsActive()) {
            driveTrain.update(-0.35, 0.35, telemetry);
        }
        driveTrain.stop();
        
        //DRIVE TOWARDS CRATER
        driveTrain.beginMeasuringDistance(42);
        driveTrain.setDrivingStraight(true);
        driveTrain.update(-0.65, -0.65, telemetry);
        while(!driveTrain.reachedTargetDistance() && opModeIsActive()) {
            driveTrain.update(-0.65, -0.65, telemetry);
        }
        driveTrain.stop();
        
        //TURN TO FACE CRATER
        telemetry.addLine("Turn to face crater");
        telemetry.addData("Turn angle", (startAngle - 180));
        telemetry.update();
        driveTrain.setTurning(true, startAngle - 180);
        while(driveTrain.getTurning() && opModeIsActive()) {
            driveTrain.update(-0.35, 0.35, telemetry);
        }
        driveTrain.stop();
        
        //FACE THE CRATER AGAIN
        telemetry.addLine("Turn to face crater");
        telemetry.addData("Turn angle", (startAngle - 30));
        telemetry.update();
        driveTrain.setTurning(true, startAngle - 30);
        while(driveTrain.getTurning() && opModeIsActive()) {
            driveTrain.update(-0.35, 0.35, telemetry);
        }
        driveTrain.stop();
        
        //DRIVE TO CRATER
        long craterStartTime = System.currentTimeMillis();
        driveTrain.setDrivingStraight(true);
        driveTrain.update(-0.70, -0.70, telemetry);
        while(System.currentTimeMillis() - craterStartTime < 1500 && opModeIsActive()) {
            driveTrain.update(-0.70, -0.70, telemetry);
        }
        
        driveTrain.stop();

        //STOP IT
        while(opModeIsActive()) {
            driveTrain.stop();
        }
        
        driveTrain.stop();
        
        //CORRECT ANGLE
        /*
        telemetry.addLine("Correct angle");
        telemetry.addData("Turn angle", (startAngle + 128));
        telemetry.update();
        driveTrain.setTurning(true, startAngle + 128);
        while(driveTrain.getTurning() && opModeIsActive()) {
            driveTrain.update(0.35, 0.35, telemetry);
        }
        driveTrain.stop();
        */
    }
}
