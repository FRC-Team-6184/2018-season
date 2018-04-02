/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6184.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This is a sample program demonstrating how to use an ultrasonic sensor and
 * proportional control to maintain a set distance from an object.
 */

public class testing_theultrasonicjava extends IterativeRobot {
	// distance in inches the robot wants to stay from an object
	private static final double kHoldDistance = 100.0;
	 private BuiltInAccelerometer accel = new BuiltInAccelerometer();
	// factor to convert sensor values to a distance in inches
	private static final double kValueToInches = 0.125;
	  private Joystick m_leftStick;

	    private Joystick m_rightStick;

	// proportional speed constant
	private static final double kP = 0.05;

	private static final int kLeftMotorPort = 0;
	private static final int kRightMotorPort = 1;
	private static final int kUltrasonicPort = 0;
	 final double BUFFER = 0.05;
	private AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
	private DifferentialDrive m_robotDrive
			= new DifferentialDrive(new Spark(1),
			new Spark(0));
	Spark mR = new Spark(0), mL = new Spark(1);
	/**
	 * Tells the robot to drive to a set distance (in inches) from an object
	 * using proportional control.
	 */  public void robotInit() {
    	 
	      

		   

	        m_leftStick = new Joystick(0);

	        m_rightStick = new Joystick(1);
	 }
	public void teleopPeriodic() {
		// sensor returns a value from 0-4095 that is scaled to inches
		double currentDistance = m_ultrasonic.getValue() * kValueToInches;
		//  System.out.println(accel.getX() + ", " + accel.getY() + ", " + accel.getZ());
		// convert distance error to a motor speed
		double currentSpeed = (kHoldDistance - currentDistance) * kP;
		  System.out.println(currentDistance);
		// drive robot
		m_robotDrive.arcadeDrive(currentSpeed, 0);
	
	
	
        double m;

        if (m_leftStick.getRawButton(3))

            m = 0.6; // Boost power

        else

            m = .6;

        double y = m_leftStick.getY(); // How much "y"

        double z = m_leftStick.getZ(); // How much "z"

        if (y > BUFFER || y <= -BUFFER) {

            mR.setSpeed(-y * currentSpeed);

            mL.setSpeed(y * currentSpeed);

        } else if (z > BUFFER || z <= -BUFFER) {

            mR.setSpeed(-z*currentSpeed);

            mL.setSpeed(z*currentSpeed);

        } else {

            mR.setSpeed(0);

            mL.setSpeed(0);

 }
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	}
}
      

