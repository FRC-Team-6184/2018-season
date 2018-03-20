package org.usfirst.frc.team6184.robot;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SafePWM;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
public class Robot extends SampleRobot {
    Joystick stick = new Joystick(0);// set to ID 0 in DriverStation(Drive Movement)
    Joystick stick2= new Joystick(1); // set ID to 1 in DriverStation(Everything Else)
    private RobotDrive drivetrain;   
    // Declare our motor controllers using their PWM ports from RoboRIO
    Victor mR = new Victor(0), mL = new Victor(1), tilt_shooter = new Victor(2), rotate_shooter = new Victor(3);
    DoubleSolenoid exampleDouble = new DoubleSolenoid(0, 1);
    Timer timer = new Timer();
   //mechanum arraylist
    // RobotDrive m_robotDrive = new RobotDrive(1, 2, 3, 4);
     AnalogInput ultraSonic = new AnalogInput(0); //Assumes 0 for the Analog
    // Input on RoboRIO
     final double ultraInputVoltage = 5.0f; //What's the voltage input for
    // HRLV-MaxSonar-EZ1 relative to Pin 7 (GND)
     final double ultraScaling = ultraInputVoltage/5120; //(about 0.977mV per
    //1mm at 5V input voltage) Scaling specs for HRLV-MaxSonar-EZ1
     DigitalOutput ultraOut = new DigitalOutput(0); 
    // Pass 0 as the port number of the Pnemuatic Controller in the Control Area Network (CAN) 
    // and also we are inherently assuming a default Node ID of 0
    //Solenoid tshirtSolenoid = new Solenoid(0);
    // Initialize things in the class constructor
     
     public Robot() {
    	 CameraServer.getInstance().startAutomaticCapture(); }
    
    // This will run when the robot is in TeleOp mode
    @Override
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
             ultraOut.set(true);
             System.out.println(getDistanceUltra()); //Print distance in mm
             // mechanum controll
           //  m_robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(),0);
       
        
    
             
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
    // This will run when robot is in autonomous mode
    //@Override
    //public void autonomous()
            
            // Rotate the tshirt cannon using buttons 3 and 4
                //Controls the solenoid valves via button inputs 11 and 12
                boolean clicked11 = stick2.getRawButton(8);
                boolean clicked12 = stick2.getRawButton(7);
                 if (clicked11){
                    exampleDouble.set(DoubleSolenoid.Value.kForward);
                 }
                 else if (!clicked11 && clicked12){
                    exampleDouble.set(DoubleSolenoid.Value.kReverse);
                 }
                 else               
                    exampleDouble.set(DoubleSolenoid.Value.kOff);
  
                 // Rotate the tshirt cannon using buttons 3 and 4
                 boolean isclickedright = stick2.getRawButton(3);
                 boolean isclickedleft = stick2.getRawButton(4);
                 if (isclickedright){
                     rotate_shooter.setSpeed(.3);
                 }
                 else if (!isclickedright && isclickedleft){
                     rotate_shooter.setSpeed(-.3);
                 }
                    else rotate_shooter.setSpeed(0);
                }
              // Use the little lever on joystick to control tilt
                tilt_shooter.setSpeed(stick2.getX());
                  }
    public void stop() {
        mR.setSpeed(0);
        mL.setSpeed(0);
    }
    public void autoDriveForward() {
        System.out.println("Driving Backwards");
        double m_speed = .7;
        mR.setSpeed(-0.5 * m_speed);
        mL.setSpeed(0.5 * m_speed);
        timer.delay(7.3);
        System.out.println("Stopping");
        stop();
}
           // Open the solenoid if button 5 is clicked
      //     if(stick.getRawButton(5)) tshirtSolenoid.set(true);
        //   else tshirtSolenoid.set(false);
        
    @Override
    public void autonomous() {
        System.out.println("Doing Autonomous");
//      autoShoot(1); // Postive is for blue side, negative is for red
        // side
    //  autoDriveForward();
//      autoGear();
        System.out.println("Done with autonomous");}
        // Uses Ultrasonic sensor to find distance in mm
         private double getDistanceUltra() {
         double volts = ultraSonic.getVoltage();
         ultraOut.set(false);
     return volts/ultraScaling;
}
            
   
   
    
    
}