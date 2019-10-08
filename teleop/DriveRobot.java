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
package org.firstinspires.ftc.roverruckus.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roverruckus.teamcode.apis.*;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class DriveRobot extends OpMode {

    private static final double DEAD_ZONE = 0.03;

    private DriveTrainAPI driveTrain;
    private LanderAPI lander;
    private SamplerAPI sampler;
    private ClaimerAPI claimer;

    private boolean startedDrivingStraight;
    private boolean toggleButtonPressed;
    
    public void init() {
        driveTrain = new DriveTrainAPI(hardwareMap, DriveTrainAPI.FOUR_WHEEL_DRIVE);
        driveTrain.addGyroscope(new GyroscopeAPI(hardwareMap));

        lander = new LanderAPI(hardwareMap);
        sampler = new SamplerAPI(hardwareMap);
        
        claimer = new ClaimerAPI(hardwareMap);
        
        toggleButtonPressed = false;
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        
    }

    @Override
    public void start() {
        sampler.liftArm();
    }

    @Override
    public void loop() {
        updateDriveTrain();
    }
    
    public void updateDriveTrain() {
        double leftPower = -gamepad1.left_stick_y, rightPower = -gamepad1.right_stick_y;
        
        if(Math.abs(leftPower) > DEAD_ZONE || Math.abs(rightPower) > DEAD_ZONE) {
            driveTrain.setDrivingStraight(false);
            driveTrain.setPower(leftPower, rightPower);
        } else {
            driveTrain.setDrivingStraight(false);                    
            driveTrain.stop();
        }
        
        if(gamepad1.y) {
            sampler.liftArm();
        } else if(gamepad1.a) {
            sampler.dropArm();
        }
        
        if(gamepad1.dpad_up) {
            lander.raiseLift(0.75);
        } else if(gamepad1.dpad_down) {
            lander.lowerLift(0.90);
        } else {
            lander.stopLift();
        }
        
        if(gamepad1.dpad_right) {
            lander.extendArm();
        } else if(gamepad1.dpad_left) {
            lander.retractArm();
        } else {
            lander.stopArm();
        }
        
        if(gamepad1.x) {
            claimer.liftArm(0.65);
        } else if(gamepad1.b) {
            claimer.dropArm(0.65);
        } else {
            claimer.stopArm();
        }
    }

    @Override
    public void stop() {

    }
}
