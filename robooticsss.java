/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6184.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends IterativeRobot {
	private static final int kFrontLeftChannel = 2;
	private static final int kRearLeftChannel = 3;
	private static final int kFrontRightChannel = 1;
	private static final int kRearRightChannel = 0;

	private static final int kJoystickChannel = 0;

	private MecanumDrive m_robotDrive;
	private Joystick m_stick;
	Solenoid tshirtSolenoid = new Solenoid(0);
	  DoubleSolenoid exampleDouble = new DoubleSolenoid(0, 1);    
	@Override
	public void robotInit() {
		Spark frontLeft = new Spark(kFrontLeftChannel);
		Spark rearLeft = new Spark(kRearLeftChannel);
		Spark frontRight = new Spark(kFrontRightChannel);
		Spark rearRight = new Spark(kRearRightChannel);

		// Invert the left side motors.
		// You may need to change or remove this to match your robot.
		frontLeft.setInverted(true);
		rearLeft.setInverted(true);

		m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
		
		m_stick = new Joystick(kJoystickChannel);
	}

	@Override
	public void teleopPeriodic() {
		boolean clicked8 = m_stick.getRawButton(8);
        boolean clicked7 = m_stick.getRawButton(7);
		if(m_stick.getRawButton(1)) tshirtSolenoid.set(true);
		else tshirtSolenoid.set(false);
		if(m_stick.getRawButton(1)) System.out.println("FIRE");
	    
		if (clicked7){
            exampleDouble.set(DoubleSolenoid.Value.kForward);
         }
         else if (!clicked7 && clicked8){
            exampleDouble.set(DoubleSolenoid.Value.kReverse);
         }
         else           {    
            exampleDouble.set(DoubleSolenoid.Value.kOff);
         }
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		m_robotDrive.driveCartesian(m_stick.getX(), m_stick.getY(),
				m_stick.getZ(), 0.0);
	}
	
	
}
