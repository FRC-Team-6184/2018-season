package org.usfirst.frc.team6184.robot;

import org.opencv.core.Core;

import org.opencv.core.Mat;

import org.opencv.core.Point;

import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.cscore.CvSink;

import edu.wpi.cscore.CvSource;

import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.CameraServer;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;

/** 

 * This is a 2018 raw code made to upload in class

 */

public class Robot extends IterativeRobot {

    private Joystick m_leftStick;

    private Joystick m_rightStick;

    DoubleSolenoid exampleDouble = new DoubleSolenoid(0, 1);    

    Compressor c = new Compressor();

    Talon lift = new Talon(3);

    Thread m_visionThread;

    Spark grabber = new Spark(2);

    DigitalInput topLimitSwitch = new DigitalInput(8);

    DigitalInput botLimitSwitch = new DigitalInput(9);

    final double BUFFER = 0.05;
   // private static final int kUltrasonicPort = 0;

   // private AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);

  //  private static final double kValueToInches = 0.125;

    Spark mR = new Spark(0), mL = new Spark(1);

    

    @Override

    

    public void robotInit() {

      

   

        m_leftStick = new Joystick(0);

        m_rightStick = new Joystick(1);

        

      CameraServer.getInstance().startAutomaticCapture();

        
//
//        new Thread(() -> {
//
//            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
//
//            camera.setResolution(640, 480);
//
//            
//
//            CvSink cvSink = CameraServer.getInstance().getVideo();
//
//            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
//
//            
//
//            Mat source = new Mat();
//
//            Mat output = new Mat();
//
//            
//
//            while(!Thread.interrupted()) {
//
//                cvSink.grabFrame(source);
//
//                output = source;
//
////                Core.`
//
////                Imgproc.circle(output, new Point(300,300), 5, new Scalar(255,0,0));
//
////                Highgui.imshow("test", output);
//
////                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
//
//                outputStream.putFrame(output);
//
////                System.out.println("YES WE ARE PUTTING BLUR IMAGE TO FEED");
//
//            }
//
//        }).start();
      c.setClosedLoopControl(true);
    }
    
    @Override

    public void teleopPeriodic() {

           double m;

           if (m_leftStick.getRawButton(3))

               m = 0.6; // Boost power

           else

               m = .6;

           double y = m_leftStick.getY(); // How much "y"

           double z = m_leftStick.getZ(); // How much "z"

           if (y > BUFFER || y <= -BUFFER) {

               mR.setSpeed(-y * m);

               mL.setSpeed(y * m);

           } else if (z > BUFFER || z <= -BUFFER) {

               mR.setSpeed(z*m);

               mL.setSpeed(z*m);

           } else {

               mR.setSpeed(0);

               mL.setSpeed(0);

    }

        boolean clicked1 = m_rightStick.getRawButton(1);
        boolean clicked2= m_rightStick.getRawButton(2);
        boolean clicked8 = m_rightStick.getRawButton(8);
            boolean clicked7 = m_rightStick.getRawButton(7);
            boolean clicked9 = m_rightStick.getRawButton(9);
            boolean clicked6 = m_rightStick.getRawButton(6);
            boolean clicked11 = m_rightStick.getRawButton(11);
            boolean clicked12= m_rightStick.getRawButton(12);
     

        if (clicked7){
            exampleDouble.set(DoubleSolenoid.Value.kForward);
         }
         else if (!clicked7 && clicked8){
            exampleDouble.set(DoubleSolenoid.Value.kReverse);
         }
         else           {    
            exampleDouble.set(DoubleSolenoid.Value.kOff);
         }
        
   
        
        if (clicked1) {
            grabber.set(-0.7);
        }
        else if (!clicked1 && clicked2){
            grabber.set(0.7);
        }
        else { 
            grabber.set(0.0);}
        
        //ultra cool sonic
//        double currentDistance = m_ultrasonic.getValue() * kValueToInches;
//        System.out.println(currentDistance);
        
       
//        if (clicked12) {
//            lift.set(-0.7);
//        }
//        else if (!clicked12 && clicked11){
//            lift.set(0.7);
//        }
//        else { 
//            lift.set(0.0);}
//    	}
        
        
        
        if (topLimitSwitch.get() && clicked12)
           lift.set(-0.7);
        else if (botLimitSwitch.get() && clicked11) 
            lift.set(0.7);
        else{ lift.set(0); // set accepts between [-1, 1]. But now, it will only recieve [-0.2, 0.2]
    	}}
        
    public void autoGrab(){
    	grabber.set(.7);
    	grabber.set(-.7);
    }
    
    String gameData;{
        gameData =
        		DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length()>0)
        {
        if(gameData.charAt(0)=='L')
       	{
        	//Put left auto code here
        }else{
        	//Put right auto code here 
        }
        }
        }
    
    public void autoDRPGRB(){
    	grabber.set(0.0);
    }
    
    public void autoDrive(){
    	double m_speed = .8;
        double t_speed = .5;
        double m2_speed = .6;
    	double l_speed = .25;
    	
    	System.out.println("I'm Gonna Go Forward");
    	mR.setSpeed(.5*m_speed);
        mL.setSpeed(-.675*m_speed);
    	Timer.delay (3);
    
    	System.out.println("I'm Gonna Turn Now");
    	mL.setSpeed(.5*t_speed);
    	Timer.delay(1);
    	
    	System.out.println("I'm Going Forward Again");
    	mR.setSpeed(.5*m2_speed);
    	mL.setSpeed(-.5*m2_speed);
    	Timer.delay(3);
    	
    	System.out.println("I'm turning one more time!");
    	mR.setSpeed(.3*t_speed);
    	Timer.delay(1);
    	
    	System.out.println("Can I stop going forward please");
    	mR.setSpeed(.5*m_speed);
    	mL.setSpeed(-.5*m_speed);
    	Timer.delay(.5);
    	
    	System.out.println("Going Up!");
    	lift.set(-.2*l_speed);
    	lift.set(.2*l_speed);
    	
    	System.out.println("I'm bored. Can someone control me?"); 
    }

    @Override
    public void autonomousInit() {
    	System.out.println("I'm gonna run in autonomous");
    	//double s_speed = (1.2);
    	
    	autoGrab();
    	autoDrive();
    	Timer.delay(2);
    	
    	autoDRPGRB();
    	
    	System.out.println("I thinnk I'm done");
    	Timer.delay(2);
    	//mR.setSpeed (1*s_speed);
        //System.out.println("WOOOOOOOOOOOOOOOOOOOOOO");
    	Timer.delay(4);
    	System.out.println("So is someone gonna control me or what?");
    }

    public void autonomousPeriodic(){
	}
	
	@Override
	public void disabledInit(){
	}
	}
