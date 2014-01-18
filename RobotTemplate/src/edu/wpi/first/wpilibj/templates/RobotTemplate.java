/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    // Driver Station update lines
    //
    private String line1LCD = "";
    private String line2LCD = "";
    private String line3LCD = "";
    private String line4LCD = "";
    private String line5LCD = "";
    private String line6LCD = "";
    
    Jaguar leftDriveMotor;
    Jaguar rightDriveMotor;
    RobotDrive robotDrive;
    
    Victor pickUpMotor;
    
    Victor frontShooterMotor;
    Victor backShooterMotor;
    
    DoubleSolenoid shifterSolenoid;
    boolean shiftBool;
    
    DoubleSolenoid climberSolenoid;
    
    DoubleSolenoid kickerSolenoid;
    boolean triggerPull = true;
    boolean kickerAutoDelay = true;
    
    Compressor pump;
    
    Relay redLED;
    Relay greenLED;
    Relay blueLED;
    
    Encoder leftDriveEncoder;
    Encoder rightDriveEncoder;
    Encoder frontShooterEncoder;
    Encoder backShooterEncoder;
    
    AnalogChannel ultrasonicSensor;
    double range;
    
    Gyro gyroSensor;
    
    AxisCamera camera;  
    Servo cameraX;
    Servo cameraY;
            
    Joystick leftStick;
    Joystick rightStick;
    Joystick opperatorController;

    Timer clock;
    Timer clockLED;
    Timer clockKicker;
    Timer clockMatch;
    Timer clockStrobe;
    Timer autoClock;

    //
    //
    int driveIndex = 0;
    double drivePower = 0.0;
    double driveDistance = 0.0;
    boolean finshedDrive[] = new boolean[3];
    boolean startShooting = true;
    boolean driveSecondReset = true;

    
    //
    //
    public RobotTemplate()
    {
        // Init Analog ports.
        //
        gyroSensor = new Gyro(2);
                
        ultrasonicSensor = new AnalogChannel(3);
        
        // Init PWM / Motor Contorler ports
        //
        rightDriveMotor = new Jaguar(2);
        leftDriveMotor = new Jaguar(3);

        frontShooterMotor = new Victor(5);
        backShooterMotor = new Victor(6);

        pickUpMotor = new Victor(7);

        cameraX = new Servo(8);
        cameraY = new Servo(9); 

        // Init Relay ports.
        //        
        pump = new Compressor(3, 1);    // DIO 3 for the switch, Relay 1 for the pump

        redLED = new Relay(3);
        greenLED = new Relay(4);
        blueLED = new Relay(5);

        // Init DIO ports.
        
        frontShooterEncoder = new Encoder(4, 5);
        backShooterEncoder = new Encoder(6, 7);
        rightDriveEncoder = new Encoder(8, 9); //not working need to check encoder continuitiy '3/28/2013 12:45'
        leftDriveEncoder = new Encoder(10, 11);

        // Init 24V Solenoid ports.
        //        
        shifterSolenoid = new DoubleSolenoid(1, 2);
        shiftBool = true;

        kickerSolenoid = new DoubleSolenoid(3, 4);

        climberSolenoid = new DoubleSolenoid(5, 6);

        // Setup Encoders
        //
        frontShooterEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
        frontShooterEncoder.setDistancePerPulse(1.0 / 250.0);
        frontShooterEncoder.setReverseDirection(true);
        frontShooterEncoder.start();
        
        backShooterEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
        backShooterEncoder.setDistancePerPulse(1.0 / 250.0);
        backShooterEncoder.setReverseDirection(true);
        backShooterEncoder.start();
        
        leftDriveEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        leftDriveEncoder.setDistancePerPulse(Math.PI * 8.0 / 250.0);
        leftDriveEncoder.setReverseDirection(true);
        leftDriveEncoder.start();
        
        rightDriveEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        rightDriveEncoder.setDistancePerPulse(Math.PI * 8.0 / 250.0);
        rightDriveEncoder.start();
        
        // Init Drive and Joysticks
        //
        robotDrive = new RobotDrive(leftDriveMotor, rightDriveMotor);
        leftStick = new Joystick(1);
        rightStick = new Joystick(2);
        opperatorController  = new Joystick(3);
        
        // Init Timers
        //
        clock = new Timer();
        clockLED = new Timer();
        clockKicker = new Timer();
        clockMatch = new Timer();
        clockStrobe = new Timer();
        autoClock = new Timer();
    }
 
    public void robotInit()
    {
        //camera = AxisCamera.getInstance();
        
        pump.start();
        
        shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
        kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
        
        cameraX.setAngle(90);
        cameraY.setAngle(90);
        
        updateStatus();
    }

    public void disabledInit()
    {
        updateStatus();
    }

    public void disabledPeriodic()
    {
        Watchdog.getInstance().feed();
        
        getFrontShooterRate();
        getBackShooterRate();
        
        updateStatus();
    }
    
    public void autonomousInit()
    {
        Watchdog.getInstance().feed();
        
        shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
        kickerSolenoid.set(DoubleSolenoid.Value.kReverse);

        gyroSensor.reset();
        
        driveReset(0, 0.7, 50.0);
        if(leftStick.getZ() >= 0.0 && leftStick.getZ() < 0.5)
        {
            driveReset(0, 0.6, 55.0);
        }
        else if(leftStick.getZ() >= 0.5 && leftStick.getZ() <= 1.0)
        {
            driveReset(0, 0.5, 0.0);
        }
        
        
        resetShooterRates();
        
        frontShooterMotor.set(-0.55);
        backShooterMotor.set(-0.55);
        
        updateStatus();
    }
    
    double rateUpdateTime = 0.0;
    double[] frontShooterRateTable = new double[20];
    int frontShooterRateTableIndex = 0;
    double frontShooterRate = 0.0;
    
    double[] backShooterRateTable = new double[20];
    int backShooterRateTableIndex = 0;
    double backShooterRate = 0.0;
    
    public void autonomousPeriodic()
    {
        Watchdog.getInstance().feed();
        //robotDrive.tankDrive(0.0, 0.0);
        getFrontShooterRate();
        getBackShooterRate();
        
        
        if(leftStick.getZ() >= -1.0 && leftStick.getZ() < -0.5)//DriveF, Shoot 3, Pick-up, Shoot 1, DriveF, Pick-up, DriveB, Shoot 1
        {
            /*driveForward(driveIndex, drivePower, driveDistance);

            if(finshedDrive[0])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 0.5 && autoClock.get() < (0.5 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 1.25 && autoClock.get() < (1.25 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 1.75 && autoClock.get() < (1.75 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);                
                }
                else if(autoClock.get() >= 2.50 && autoClock.get() < (2.50 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 3.25 && autoClock.get() < (3.25 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    pickUpMotor.set(1.0);
                }
                else if(autoClock.get() >= 5.75 && autoClock.get() < (5.75 + 0.2))
                {
                    pickUpMotor.set(0.0);
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                    gyroSensor.reset();
                }
                else if(autoClock.get() >= 6.25 && autoClock.get() < (6.25 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    driveReset(1, 0.9, 85.0);
                    
                }
            }

            if (finshedDrive[1])
            {
                 if(startShooting)
                 {
                     pickUpMotor.set(1.0);
                     startShooting = false;
                     gyroSensor.reset();
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 2.5 && autoClock.get() < (2.5 + 0.2))//Pulls back after first shot
                {
                    pickUpMotor.set(0.0);
                    driveReset(2, 0.9, -85.0);
                }
            }

            if (finshedDrive[2])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     autoClock.reset();
                     autoClock.start();
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                 }

                if(autoClock.get() >= 0.75 && autoClock.get() < (0.75 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    frontShooterMotor.set(0.0);
                    backShooterMotor.set(0.0);
                }
            }*/
            
            driveForward(driveIndex, drivePower, driveDistance);

            if(finshedDrive[0])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 0.5 && autoClock.get() < (0.5 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 1.25 && autoClock.get() < (1.25 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 1.75 && autoClock.get() < (1.75 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);                
                }
                else if(autoClock.get() >= 2.50 && autoClock.get() < (2.50 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                    gyroSensor.reset();
                }
                else if(autoClock.get() >= 3.25 && autoClock.get() < (3.25 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    pickUpMotor.set(1.0);
                    driveReset(1, 0.8, 85.0);
                }
            }

            if (finshedDrive[1])
            {
                 if(startShooting)
                 {
                     pickUpMotor.set(1.0);
                     startShooting = false;
                     gyroSensor.reset();
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 1.0 && autoClock.get() < (1.0 + 0.2))//Pulls back after first shot
                {
                    pickUpMotor.set(0.0);
                    driveReset(2, 0.8, -82.0);
                }
            }

            if (finshedDrive[2])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     autoClock.reset();
                     autoClock.start();
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                 }

                if(autoClock.get() >= 0.75 && autoClock.get() < (0.75 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 1.5 && autoClock.get() < (1.5 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 2.25 && autoClock.get() < (2.25 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    frontShooterMotor.set(0.0);
                    backShooterMotor.set(0.0);
                    pickUpMotor.set(0.0);                    
                }                
        }
        }
        else if(leftStick.getZ() >= -0.5 && leftStick.getZ() < 0.0)//DriveF, Shoot 3, Pick-up, Shoot 1
        {
            /*driveForward(driveIndex, drivePower, driveDistance);

            if(finshedDrive[0])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 0.5 && autoClock.get() < (0.5 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 1.25 && autoClock.get() < (1.25 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 1.75 && autoClock.get() < (1.75 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);                
                }
                else if(autoClock.get() >= 2.50 && autoClock.get() < (2.50 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 3.0 && autoClock.get() < (3.0 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);                
                }
                else if(autoClock.get() >= 3.75 && autoClock.get() < (3.75 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                
                else if(autoClock.get() >= 4.25 && autoClock.get() < (4.25 + 0.2))//Pulls back after forth shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    pickUpMotor.set(1.0);
                    driveReset(1, 0.5, 20.0);
                }
                else if(autoClock.get() >= 6.75 && autoClock.get() < (6.75 + 0.2))
                {
                    driveReset(2, 0.5, -20.0);
                }
                else if(autoClock.get() >= 9.0 && autoClock.get() < (9.0 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 9.75 && autoClock.get() < (9.75 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);                  
                }
            }*/
            
            
            
            driveForward(driveIndex, drivePower, driveDistance);

            if(finshedDrive[0])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     //kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 1.5 && autoClock.get() < (1.5 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);                    
                }
                else if(autoClock.get() >= 2.25 && autoClock.get() < (2.25 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 2.75 && autoClock.get() < (2.75 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);                
                }
                else if(autoClock.get() >= 3.50 && autoClock.get() < (3.50 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    gyroSensor.reset();
                }
                else if(autoClock.get() >= 4.25 && autoClock.get() < (4.25 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 5.0 && autoClock.get() < (5.0 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    pickUpMotor.set(1.0);
                    driveReset(1, 0.6, 30.0);
                }
            }

            if (finshedDrive[1])
            {
                 if(startShooting)
                 {
                     pickUpMotor.set(1.0);
                     startShooting = false;
                     gyroSensor.reset();
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 1.5 && autoClock.get() < (1.5 + 0.2))//Pulls back after first shot
                {
                    pickUpMotor.set(0.0);
                    driveReset(2, 0.6, -23.5);
                }
            }

            if (finshedDrive[2])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     autoClock.reset();
                     autoClock.start();
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                 }

                if(autoClock.get() >= 0.75 && autoClock.get() < (0.75 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 1.5 && autoClock.get() < (1.5 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 2.25 && autoClock.get() < (2.25 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    frontShooterMotor.set(0.0);
                    backShooterMotor.set(0.0);
                    pickUpMotor.set(0.0);                    
                }                
        }
        }
        else if(leftStick.getZ() >= 0.0 && leftStick.getZ() < 0.5)//DriveF more, Shoot 3
        {
            driveForward(driveIndex, drivePower, driveDistance);

            if(finshedDrive[0])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                     autoClock.reset();
                     autoClock.start();
                 }

                if(autoClock.get() >= 1.0 && autoClock.get() < (1.0 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 2.0 && autoClock.get() < (2.0 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 3.0 && autoClock.get() < (3.0 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);                
                }
                else if(autoClock.get() >= 4.0 && autoClock.get() < (4.0 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 5.0 && autoClock.get() < (5.0 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                    pickUpMotor.set(0.0);
                }
                else if(autoClock.get() >= 6.0 && autoClock.get() < (6.0 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 7.0 && autoClock.get() < (7.0 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 8.0 && autoClock.get() < (8.0 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 9.0 && autoClock.get() < (9.0 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }                
            }            
        }
        else if(leftStick.getZ() >= 0.5 && leftStick.getZ() <= 1.0)//Shoot 3
        {
            driveForward(driveIndex, drivePower, driveDistance);

            if(finshedDrive[0])
            {
                 if(startShooting)
                 {
                     startShooting = false;
                     autoClock.reset();
                     autoClock.start();
                 }
                 
                if(autoClock.get() >= 3.0 && autoClock.get() < (3.0 + 0.2))
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 4.0 && autoClock.get() < (4.0 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 5.0 && autoClock.get() < (5.0 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 6.0 && autoClock.get() < (6.0 + 0.2))//Pull back
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);                
                }
                else if(autoClock.get() >= 7.0 && autoClock.get() < (7.0 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 8.0 && autoClock.get() < (8.0 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 9.0 && autoClock.get() < (9.0 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 10.0 && autoClock.get() < (10.0 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }       
                else if(autoClock.get() >= 11.0 && autoClock.get() < (11.0 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 12.0 && autoClock.get() < (12.0 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else if(autoClock.get() >= 13.0 && autoClock.get() < (13.0 + 0.2)) // Fire
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                else if(autoClock.get() >= 14.0 && autoClock.get() < (14.0 + 0.2))//Pulls back after third shot
                {
                    kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
                }                
            }            
        }
        /*if(clockKicker.get() > 1000)
        {
            kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
            clockKicker.reset();
        }*/

        // make LEDs turn white 
        if(kickerSolenoid.get() == DoubleSolenoid.Value.kForward)
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kForward);           
        }
        else
        {
            setLEDsAllianceColor();
        }
        
        updateStatus();
    }
    
    
    public void teleopInit()
    {
        Watchdog.getInstance().feed();

        gyroSensor.reset();

        clockLED.reset();
        clockLED.start();
        clockMatch.reset();
        clockMatch.start();
        
        shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
        kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
        
        resetShooterRates();
        
        updateStatus();
    }

    public void teleopPeriodic()
    {
        Watchdog.getInstance().feed();
        updateLEDs();
        
        getFrontShooterRate();
        getBackShooterRate();
        
        //distance = ultrasonicSensor.getVoltage();
        /*if(kickerTriangle.get() == DoubleSolenoid.Value.kForward)
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kForward);           
        }*/
        
        robotDrive.tankDrive(leftStick, rightStick); //*****Driving*****
                //System.out.println("robotDrive.tankDrive called");
        
        if(leftStick.getTrigger()) // *****Shifting*****
        {
            if (shiftBool)
            {
                shiftBool = false;
                if(shifterSolenoid.get() == DoubleSolenoid.Value.kForward)
                {
                    shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
                else
                {
                    shifterSolenoid.set(DoubleSolenoid.Value.kForward);
                }
            }
        }
        else
        {
            shiftBool = true;
        }
        
        if(opperatorController.getRawButton(9))// ***** Kicker *****
        {
            kickerSolenoid.set(DoubleSolenoid.Value.kForward);
            triggerPull = false;
            clockKicker.start();
        }
        
        if(clockKicker.get() > 0.500)
        {
            kickerSolenoid.set(DoubleSolenoid.Value.kReverse);
            triggerPull = true;
            clockKicker.reset();
        }
        
        
        if(opperatorController .getRawButton(4))// *****PickUp*****
        {
            pickUpMotor.set(1.0);
        }
        else if (opperatorController.getRawButton(2))
        {
            pickUpMotor.set(-1.0);
        }
        else
        {
            pickUpMotor.set(0);
        }
        
        if(opperatorController.getRawButton(5) && opperatorController.getRawButton(6))// *****Climbing*****
        {
            climberSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        if(opperatorController.getRawButton(7) && opperatorController.getRawButton(8))
        {
            climberSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        
        if(opperatorController.getRawButton(3))// *****Shooter*****
        {
            backShooterMotor.set(-Math.abs(rightStick.getZ())); 
            frontShooterMotor.set(-Math.abs(rightStick.getZ()));
        }
        else if(opperatorController.getRawButton(1))
        {
            backShooterMotor.set(Math.abs(rightStick.getZ())); 
            frontShooterMotor.set(Math.abs(rightStick.getZ()));  
        }
        else
        {
            backShooterMotor.set(0.0); 
            frontShooterMotor.set(0.0);
        }
        
        if(opperatorController.getRawButton(11))// ***** Camera Servos *****
        {
            cameraY.setAngle(cameraY.getAngle()+4.0);
        }
        else if(rightStick.getRawButton(10))
        {
            cameraY.setAngle(cameraY.getAngle()-4.0);
        }
        if(rightStick.getRawButton(9))
        {
            cameraX.setAngle(cameraX.getAngle()+4.0);
        }
        else if(rightStick.getRawButton(8))
        {
            cameraX.setAngle(cameraX.getAngle()-4.0);
        }
        if(rightStick.getRawButton(7))
        {
            cameraY.setAngle(90);
            cameraX.setAngle(90);
        }
        
        if(leftStick.getRawButton(11))
        {
            leftDriveEncoder.reset();
            leftDriveEncoder.start();
            rightDriveEncoder.reset();
            rightDriveEncoder.start();
        }

        updateStatus();
    }
    
    public void driveReset(int driveNum, double power, double distance)
    {
        leftDriveEncoder.reset();
        leftDriveEncoder.start();
        rightDriveEncoder.reset();
        rightDriveEncoder.start();

        for(int i = 0; i < finshedDrive.length; i++)
        {
          finshedDrive[i] = false;
        }
        
        startShooting = true;        
        driveIndex = driveNum;
        drivePower = power;
        driveDistance = distance;
    }
    
    //Note, dont use a power of over .8
    //
    public void driveForward(int driveNum, double power, double distance) 
    {
        if(distance >= 0)
        {
            {
                if(distance > leftDriveEncoder.getDistance())
                {
                    rightDriveMotor.set(power);
                    leftDriveMotor.set(-(power - (((gyroSensor.getAngle()))*.03)));
                }
                else
                {
                    rightDriveMotor.set(0.0);
                    leftDriveMotor.set(0.0);
                    finshedDrive[driveNum] = true;
                }
            }
        }
        else
        {
            if(distance < leftDriveEncoder.getDistance())
            {
                rightDriveMotor.set(-power);
                leftDriveMotor.set((power + (((gyroSensor.getAngle()))*.03)));
            }
            else
            {
                rightDriveMotor.set(0.0);
                leftDriveMotor.set(0.0);
                finshedDrive[driveNum] = true;
            }
        }
    }
    
    public double getFrontShooterRate()
    {
        frontShooterRateTable[frontShooterRateTableIndex] = frontShooterEncoder.getRate();
        frontShooterRateTableIndex++;        
        if (frontShooterRateTableIndex >= frontShooterRateTable.length)
        {
            frontShooterRateTableIndex = 0;
        }
        
        frontShooterRate = 0.0;
        for (int i = 0; i < frontShooterRateTable.length; i++)
        {
            frontShooterRate += frontShooterRateTable[i];
        }
        frontShooterRate = frontShooterRate / (double)frontShooterRateTable.length;

        return frontShooterRate;
    }
    
    public double getBackShooterRate()
    {
        backShooterRateTable[backShooterRateTableIndex] = backShooterEncoder.getRate();
        backShooterRateTableIndex++;
        if (backShooterRateTableIndex >= backShooterRateTable.length)
        {
            backShooterRateTableIndex = 0;
        }
        
        backShooterRate = 0.0;
        for (int i = 0; i < backShooterRateTable.length; i++)
        {
            backShooterRate += backShooterRateTable[i];
        }
        backShooterRate = backShooterRate / (double)backShooterRateTable.length;
        
        return backShooterRate;
    }
    
    public void resetShooterRates()
    {
        frontShooterEncoder.reset();
        frontShooterEncoder.start();
        backShooterEncoder.reset();
        frontShooterEncoder.reset();
        
        frontShooterRate = 0.0;
        for (int i = 0; i < frontShooterRateTable.length; i++)
        {
            frontShooterRateTable[i] = 0.0;
        }
        
        backShooterRate = 0.0;
        for (int i = 0; i < backShooterRateTable.length; i++)
        {
            backShooterRateTable[i] = 0.0;
        }
    }
    
    public void setLEDsAllianceColor()
    {
        DriverStation ds = DriverStation.getInstance();
        
        if(ds.getAlliance().equals(DriverStation.Alliance.kRed))
        {
            redLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kForward);
        }
        else 
        {   
            redLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kForward);
        }
    }
    
    public void updateLEDs()
    {
        DriverStation ds = DriverStation.getInstance();
        
        double ledClockValue = clockLED.get();
        
        if((ledClockValue > 0.0) && (ledClockValue < 0.500))//Red
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kForward);
        }
        else if((ledClockValue >= 0.500) && (ledClockValue < 1.000))//Yellow
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kForward);
        }
        else if((ledClockValue >= 1.000) && (ledClockValue < 1.500))//Green
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kForward);
        }
        else if((ledClockValue >= 1.500) && (ledClockValue < 2.000))//Teal
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kReverse);
        }
        else if((ledClockValue >= 2.000) && (ledClockValue < 2.500))//Blue
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kReverse);
        }
        else if((ledClockValue >= 2.500) && (ledClockValue < 3.000))//Purple
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kReverse);
        }
        else if((ledClockValue >= 3.000) && (ledClockValue < 3.500))//White
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kReverse);
        }
        if((ledClockValue > 3.500) && (ledClockValue < 4.000))//Red
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kForward);
        }
        else if((ledClockValue >= 4.000) && (ledClockValue < 4.500))//Yellow
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kForward);
        }
        else if((ledClockValue >= 4.500) && (ledClockValue < 5.000))//Green
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kForward);
        }
        else if((ledClockValue >= 5.000) && (ledClockValue < 5.500))//Teal
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kReverse);
        }
        else if((ledClockValue >= 5.500) && (ledClockValue < 6.000))//Blue
        {
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kReverse);
        }
        else if((ledClockValue >= 6.000) && (ledClockValue < 6.500))//Purple
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kForward);
            blueLED.set(Relay.Value.kReverse);
        }
        else if((ledClockValue >= 6.500) && (ledClockValue < 7.000))//White
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kReverse);
        }
        else if(ledClockValue>=7.000  && (kickerSolenoid.get().equals(DoubleSolenoid.Value.kForward)))
        {
            redLED.set(Relay.Value.kReverse);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kReverse);
        }
        else
        {
            int time100 = (int)(clockMatch.get() * 100.0);
            if (clockMatch.get() > 105.0 && (time100%50 >= 25))
            {
                redLED.set(Relay.Value.kForward);
                blueLED.set(Relay.Value.kForward);
                greenLED.set(Relay.Value.kForward);
            }
            else
            {
                setLEDsAllianceColor();
            }
        }
        
        if(rightStick.getTrigger())
        {           
            redLED.set(Relay.Value.kForward);
            greenLED.set(Relay.Value.kReverse);
            blueLED.set(Relay.Value.kForward);
        }
    }
    
    /**
     *
     */
    private void updateStatus()
    {
        Watchdog.getInstance().feed();

        if(leftStick.getZ() >= -1.0 && leftStick.getZ() < -0.5)
        {
            line1LCD = "Auto: 5 Disk" + "  ST: " + (int)((Math.abs(rightStick.getZ()) * 100) + 0.5);            
        }
        else if(leftStick.getZ() >= -0.5 && leftStick.getZ() < 0.0)
        {
            line1LCD = "Auto: 4 Disk" + "  PT: " + (int)((Math.abs(rightStick.getZ()) * 100) + 0.5);             
        }        
        else if(leftStick.getZ() >= 0.0 && leftStick.getZ() < 0.5)
        {
            line1LCD = "Auto: 3 D M" + "  PT: " + (int)((Math.abs(rightStick.getZ()) * 100) + 0.5);             
        }
        else if(leftStick.getZ() >= 0.5 && leftStick.getZ() <= 1.0)
        {
            line1LCD = "Auto: 3 D" + "  PT: " + (int)((Math.abs(rightStick.getZ()) * 100) + 0.5);             
        } 
        else
        {
            line1LCD = "Auto: " + (int)((leftStick.getZ() * 100) + 0.5) + "  PT: " + (int)((Math.abs(rightStick.getZ()) * 100) + 0.5);
        }
        line2LCD = "Range: " + (ultrasonicSensor.getVoltage() * 100.0);
        line3LCD = "Gyro Angle: " + (gyroSensor.getAngle());
        line4LCD = "BSR: " + (int)(backShooterRate + 0.5) + "  FSR: " + (int)(frontShooterRate + 0.5);
        line5LCD = "RDE: " + (int)(rightDriveEncoder.getDistance() + 0.5) + "  LDE: " + (int)(leftDriveEncoder.getDistance() + 0.5);
      
        if(shifterSolenoid.get().equals(DoubleSolenoid.Value.kReverse))
        {
            line6LCD = "LOW/SLOW GEAR";
        }
        if(shifterSolenoid.get().equals(DoubleSolenoid.Value.kForward))
        {
            line6LCD = "HIGH/FAST GEAR";
        }

        updateHighPriDashboard();
        updateLowPriDashboard();
        updateDriverStationLCD();
        
        Watchdog.getInstance().feed();
    }

    void updateLowPriDashboard()
    {
        Dashboard lowDashData = DriverStation.getInstance().getDashboardPackerLow();
        lowDashData.addCluster();
        {
            lowDashData.addCluster();
            {     //analog modules
                lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++)
                    {
                        lowDashData.addFloat((float) AnalogModule.getInstance(1).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();
                lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++) {
                        lowDashData.addFloat((float) 0.0);
// No Second Card                 lowDashData.addFloat((float) AnalogModule.getInstance(2).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();
            }
            lowDashData.finalizeCluster();

            lowDashData.addCluster();
            { //digital modules
                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 1;
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 1;  // we don't have a module 2
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayReverse());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

            }
            lowDashData.finalizeCluster();

            lowDashData.addByte(Solenoid.getAllFromDefaultModule());
        }
        lowDashData.finalizeCluster();
        lowDashData.commit();

    }
    
    
     /**
     * Update the high priority dashboard data.
     */
    private void updateHighPriDashboard()
    {
        Dashboard dashBoard = DriverStation.getInstance().getDashboardPackerHigh();

        dashBoard.addCluster();
        {
            dashBoard.addCluster();
            { // Tracking Data
                dashBoard.addDouble(0.0);           // X
                dashBoard.addDouble(0.0);           // Angle
                dashBoard.addDouble(0.0);           // Angle Rate
                dashBoard.addDouble(0.0);           // X
            }
            dashBoard.finalizeCluster();

            dashBoard.addCluster();
            { // Target Info
                dashBoard.addArray();
                { // Targets
                    dashBoard.addCluster();
                    {
                        dashBoard.addDouble(0.0); // Target Score
                        dashBoard.addCluster();
                        { // Circle Descriptor
                            dashBoard.addCluster();
                            { // Position
                                dashBoard.addFloat((float)0.0); // X
                                dashBoard.addFloat((float)0.0); // Y
                            }
                            dashBoard.finalizeCluster();

                            dashBoard.addDouble(0.0);   // Angle
                            dashBoard.addDouble(0.0);   // Major Radius
                            dashBoard.addDouble(0.0);   // Minor Radius
                            dashBoard.addDouble(1.0);   // Raw Score
                        }
                        dashBoard.finalizeCluster();
                    }
                    dashBoard.finalizeCluster();
                }
                dashBoard.finalizeArray();

                dashBoard.addInt(0);
            }
            dashBoard.finalizeCluster();
        }
        dashBoard.finalizeCluster();

        dashBoard.commit();
    }

    private void updateDriverStationLCD()
    {
       String padding = "                     ";   // 21 spaces to make sure that line is complety replaced with desired text.

       DriverStationLCD driverStationUserMessages = DriverStationLCD.getInstance();

       driverStationUserMessages.println(DriverStationLCD.Line.kUser1, 1, line1LCD + padding);
       driverStationUserMessages.println(DriverStationLCD.Line.kUser2, 1, line2LCD + padding);
       driverStationUserMessages.println(DriverStationLCD.Line.kUser3, 1, line3LCD + padding);
       driverStationUserMessages.println(DriverStationLCD.Line.kUser4, 1, line4LCD + padding);
       driverStationUserMessages.println(DriverStationLCD.Line.kUser5, 1, line5LCD + padding);
       driverStationUserMessages.println(DriverStationLCD.Line.kUser6, 1, line6LCD + padding);

       driverStationUserMessages.updateLCD(); //update the LCD lines to the driver station
    }
}