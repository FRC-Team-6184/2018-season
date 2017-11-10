package org.usfirst.frc.team6184.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
//import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;

//import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot {

    Joystick stick = new Joystick(0); // set to ID 0 in DriverStation

    // Declare our motor controllers using their PWM ports from RoboRIO
    Victor mR = new Victor(0), mL = new Victor(1), tilt_shooter = new Victor(2), rotate_shooter = new Victor(3);

    // Pass 0 as the port number of the Pnemuatic Controller in the Control Area Network (CAN) 
    // and also we are inherently assuming a default Node ID of 0
    Solenoid tshirtSolenoid = new Solenoid(0);

    // Initialize things in the class constructor
    public Robot() {
    }

    // This will run when the robot is in TeleOp mode
    @Override
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            
            // Rotate the tshirt cannon using buttons 3 and 4
            boolean isclickedright = stick.getRawButton(3);
            boolean isclickedleft = stick.getRawButton(4);
            if (isclickedright){
                rotate_shooter.setSpeed(1);
            }
            else if (!isclickedright && isclickedleft){
                rotate_shooter.setSpeed(-1);
            }
               else rotate_shooter.setSpeed(0);

           // Use the little lever on joystick to control tilt
           tilt_shooter.setSpeed(stick.getX());

           // Open the solenoid if button 5 is clicked
           if(stick.getRawButton(5)) tshirtSolenoid.set(true);
           else tshirtSolenoid.set(false);


           // Drive the robot using the 2 drive-train motors
            double m;
            if (stick.getRawButton(3))
                m = 1; // Boost power
            else
                m = .7;
            double y = stick.getY(); // How much "y"
            double z = stick.getZ(); // How much "z"
            if (y > .1 || y <= -.1) {
                mR.setSpeed(-y * m);
                mL.setSpeed(y * m);
            } else if (z > .1 || z <= -.1) {
                mR.setSpeed(z*m);
                mL.setSpeed(z*m);
            } else {
                mR.setSpeed(0);
                mL.setSpeed(0);
            }
         }
    }


    // This will run when robot is in autonomous mode
    @Override
    public void autonomous() {
    }
}