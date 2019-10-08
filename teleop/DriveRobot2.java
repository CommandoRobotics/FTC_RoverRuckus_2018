/*
Version: 2.0

Description:
This program uses only the joysticks, triggers, and bumpers to control the 
robot. You can switch what mode the controls are in by pressing x.

Copyright information:
This program belongs to team 6042, The Rocket Scientists. This program comes
with no warranty. The program may mess up your robot because it is meant to be 
used for our robot. We (the rocket scientists) claim no responsibility if 
anything breaks including, but not limited to, your robot, your driver station, 
your robot controller, or anything your robot runs into.  
*/
package org.firstinspires.ftc.roverruckus.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class DriveRobot2 extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DigitalChannel digitalTouch;
    private Blinker expansion_Hub_2;
    private DistanceSensor sensorColorRange;
    private DcMotor sweeper;
    private DcMotor elevator;
    private Servo hand;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        hand = hardwareMap.get(Servo.class, "hand");

        // set digital channel to input mode.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double leftTgtPower = 0;
        double rightTgtPower = 0;
        double sweeperTgtPower = 0;
        double elbowTgtPower = 0;
        boolean elbowActivate = false;
        boolean isUpPressed = false;
        boolean isDownPressed = false;
        boolean elbowReverseActivate = false;
        boolean sweeperReverseActivate = false;
        boolean sweeperActivate = false;
        boolean isRBPressed = false;
        boolean isLBPressed = false;
        boolean modeDrive = true;
        boolean isXPressed = false;
        boolean handUp = true;
        
        while (opModeIsActive()) {
            leftTgtPower = -this.gamepad1.left_stick_y;
            rightTgtPower = -this.gamepad1.right_stick_y;
            leftWheel.setPower(-1*leftTgtPower) ;
            rightWheel.setPower(rightTgtPower) ;
            
            if(gamepad1.x && isXPressed == false) {
                    if(modeDrive == false){
                        modeDrive = true;
                        isXPressed = true;
                        
                    } else {
                        modeDrive = false;
                        isXPressed = true;
                    }
                } else if(gamepad1.x == false) {
                    isXPressed = false;
                }
            
            
            if(modeDrive == true){
                telemetry.clearAll();
                telemetry.addLine("Mode : Drive");
                
                if(gamepad1.right_bumper && isRBPressed == false) {
                    if(sweeperActivate == false){
                        sweeperActivate = true;
                        isRBPressed = true;

                    } else {
                        sweeperActivate = false;
                        isRBPressed = true;
                    }
                } else if(gamepad1.right_bumper == false) {
                    isRBPressed = false;
                }
                
                if(gamepad1.left_bumper && isLBPressed == false) {
                    if(sweeperReverseActivate == false){
                        sweeperReverseActivate = true;
                        isLBPressed = true;
                        
                    } else {
                        sweeperReverseActivate = false;
                        isLBPressed = true;
                    }
                } else if(gamepad1.left_bumper == false) {
                    isLBPressed = false;
                }
                
                if(sweeperActivate == true){
                    sweeperTgtPower = .5;
                    sweeper.setPower(sweeperTgtPower);
                } else if(sweeperReverseActivate == true){
                    sweeperTgtPower = -.5;
                    sweeper.setPower(sweeperTgtPower);
                } else {
                    sweeperTgtPower = 0;
                    sweeper.setPower(sweeperTgtPower);
                }
                
                if(sweeperReverseActivate == true && sweeperActivate == true){
                 sweeperReverseActivate = false;
                 sweeperActivate = false;
                }
    
        
                
                if(gamepad1.right_trigger > 0){
                    hand.setPosition(0.9);
                    handUp = true;
                    
    
                } else if (gamepad1.left_trigger > 0){
                    hand.setPosition(0.625);
                    handUp=false;
                }
                
                if(handUp == true){
                    telemetry.addLine("Hand Possition : Up");
                } else {
                    telemetry.addLine("Hand Possition : Down");
                }
        }
            if(modeDrive == false){
                telemetry.clearAll();
                telemetry.addLine("Mode : Climb");
                
                if(gamepad1.right_trigger > 0){
                    elevator.setPower(gamepad1.right_trigger);
                    telemetry.addLine("Elevator : Moving Up");
                } else if(gamepad1.left_trigger > 0){
                    elevator.setPower(-1*gamepad1.left_trigger);
                    telemetry.addLine("Elevator : Moving Down");
                } else {
                    elevator.setPower(0);
                    telemetry.addLine("Elevator : Stationary");
                }
                
            }


            telemetry.update();

        }
    }
}
