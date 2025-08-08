// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// **************************************************************
// FRC 2022 Rapid React - Team 7454 - Huskies on Hogs
// Mentor: Abram Devonshire - abramdevonshire@gmail.com
// 
// Shift+F5 to deploy
// **************************************************************

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.RobotController;  //RoboRIO functions
//import edu.wpi.first.wpilibj.DriverStation; // get station info from DriverStation
//import edu.wpi.first.wpilibj.AnalogInput; // RoboRio ANALOG IN 0-3
import edu.wpi.first.wpilibj.DigitalInput; // RoboRIO DIO Ports
import edu.wpi.first.wpilibj.PneumaticsModuleType; // Pneumatics Control Module
import edu.wpi.first.wpilibj.Compressor; // ability to use compressor
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // JOYSTICKS
  private final Joystick flight = new Joystick(0);
  private final Joystick controller = new Joystick(1);
  // Controller Buttons
  // private static final int aButton = 1;
  // private static final int bButton = 2;
  private static final int xButton = 3;
  private static final int yButton = 4;
  private static final int lbButton = 5;
  private static final int rbButton = 6;
  private static final int backButton = 7;
  private static final int startButton = 8;
  // Flight Buttons
  private static final int flight1 = 1; // trigger
  private static final int flight2 = 2; // thumb
  private static final int flight3 = 3;
  private static final int flight4 = 4;
  // private static final int flight5 = 5;
  // private static final int flight6 = 6;
  private static final int flight7 = 7;
  private static final int flight8 = 8;
  private static final int flight9 = 9;
  private static final int flight10 = 10;
  private static final int flight11 = 11;
  private static final int flight12 = 12;
  // Flight Other
  private static final int flightPaddle = 3;

  // CAMERAS
  UsbCamera shooterCamera;
  UsbCamera intakeCamera;
  NetworkTableEntry cameraSelection; // which camera for dashboard

  // DRIVE MOTORS and DRIVE SETTINGS
  // Assign motors to groups, assign groups to Drive
  private final Spark m_frontRioSide = new Spark(0);
  private final Spark m_rearRioSide = new Spark(1);
  private final MotorControllerGroup mg_RioSide = new MotorControllerGroup(m_frontRioSide, m_rearRioSide);
  private final Spark m_frontAirSide = new Spark(2);
  private final Spark m_rearAirSide = new Spark(3);
  private final MotorControllerGroup mg_AirSide = new MotorControllerGroup(m_frontAirSide, m_rearAirSide);
  private final DifferentialDrive drive_Main = new DifferentialDrive(mg_RioSide, mg_AirSide);
  private double speedMultiplier = 0;
  double xCorrect = .65; // used to smooth out turning

  // SHOOTER / INTAKE MOTORS
  private final Spark m_ShooterBottom = new Spark(4);
  private final Spark m_ShooterTop = new Spark(5);
  private final Spark m_ShooterMiddle = new Spark(6);
  private final MotorControllerGroup mg_Shooter = new MotorControllerGroup(m_ShooterTop, m_ShooterBottom, m_ShooterMiddle);
  private final MotorControllerGroup mg_Intake = new MotorControllerGroup(m_ShooterTop, m_ShooterBottom);

  // ENCODERS
  Encoder enc_RioSide;
  Encoder enc_AirSide;

  // DIGITAL INPUT PORTS
  // DigitalInput(0), DigitalInput(1) are mapped to encoder enc_AirSide
  // DigitalInput(2), DigitalInput(3) are mapped to Encoder enc_RioSide
  private final DigitalInput s_ballSensor = new DigitalInput(4); // photoelectric
  private final DigitalInput ls_climbAirSide = new DigitalInput(5);
  private final DigitalInput ls_climbRioSide = new DigitalInput(6);
  private final DigitalInput switchAutoMode = new DigitalInput(7);
  // private final DigitalInput dio_8 = new DigitalInput(8);
  // private final DigitalInput dio_9 = new DigitalInput(9);

  // COMPRESSOR AND PNEUMATICS
  private final Compressor c_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid solenoidShort = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); // Short Extend on PCM 0, Retract on PCM 1
  private final DoubleSolenoid solenoidLong = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3); // Long Extend on PCM 2, Retract on PCM 3
  private final DoubleSolenoid solenoidMedium = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5); // Medium Extend on PCM 4, Retract on PCM 5

  // Ultrasonic
  public double voltageScaleFactor = 1;
  Ultrasonic ultrasonicSensor = new Ultrasonic(9, 8);
  double ultrasonicOffset = 12.95; 

  // CONTROL VARIABLES
  boolean bUseDistanceSensorToShoot = true;
  boolean debugMode = false;

  // GLOBAL VARIABLES
  boolean forwardDriveToggle = true;
  boolean intakeToggle = true;
  String gameInfoAlliance = "Unknown";
  int gameInfoStation = 0;
  boolean bClimberHooked = false;
  boolean bClimberAbort = false;

  // shooter
  // TODO Fine tune these distances with ultrasonic on and off.
  double shooterMaxDistance = 40; // How many inches can we accurately shoot when motors running at 100%?
  double shooterDefaultMotorSpeed = .65;  // This is our safe average speed.  Will be used if sensors are off.
  double shooterMaxMotorSpeed = 1;  // Motor top speed to safely shoot
  double shooterMinMotorSpeed = .50; // Speed to use against wall
  double shooterSafeShootDistanceInches = 40; // What we use for Shuffleboard visual
  // intake
  double intakeSpeed = .6;
  double intakeDumpSpeed = -1;
  // autonomous variables
  int autoRunCounter = 0;
  int autoEncoderCounter = 0;
  boolean autoBallShot = false;
  boolean auto6BackCargo = false;
  boolean autoBallShot2 = false;

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Set reverse motor set as inverted and disable safety check
    mg_AirSide.setInverted(true);
    drive_Main.setSafetyEnabled(false);

    // Init Encoders
    enc_AirSide = new Encoder(1, 0, false, Encoder.EncodingType.k2X);
    enc_RioSide = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
    enc_AirSide.setDistancePerPulse(Math.PI * 6 / 2625); // distance per pulse is pi* (wheel diameter / counts per revolution)
    enc_RioSide.setDistancePerPulse(Math.PI * 6 / 2625); // distance per pulse is pi* (wheel diameter / counts per revolution)

    // Invert shooter motors and disable safety check
    mg_Shooter.setInverted(true);
    mg_Intake.setInverted(true);
    m_ShooterBottom.setSafetyEnabled(false);
    m_ShooterTop.setSafetyEnabled(false);
    m_ShooterMiddle.setSafetyEnabled(false);

    // Turn Compressor on
    c_compressor.enableDigital();

    // Turn Camaras on
    shooterCamera = CameraServer.startAutomaticCapture(0);
    intakeCamera = CameraServer.startAutomaticCapture(1);
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    Ultrasonic.setAutomaticMode(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Encoder Details
    SmartDashboard.putNumber("Encoder Rio Side", enc_RioSide.getDistance());
    SmartDashboard.putNumber("Encoder Air Side", enc_AirSide.getDistance());
    // Drive Helpers
    SmartDashboard.putNumber("xCorrect", xCorrect);
    SmartDashboard.putNumber("speedMultiplier", speedMultiplier);
    SmartDashboard.putBoolean("Ball Sensor", s_ballSensor.get());
    SmartDashboard.putBoolean("Forward Drive", forwardDriveToggle);
    SmartDashboard.putBoolean("Intake Status", intakeToggle);
    SmartDashboard.putNumber("Y", flight.getY());
    SmartDashboard.putNumber("X", flight.getX());
    // Climber Helpers
    SmartDashboard.putBoolean("LS Airside", ls_climbAirSide.get());
    SmartDashboard.putBoolean("LS RioSide", ls_climbRioSide.get());
    // Sensors
    SmartDashboard.putBoolean("At Pressure", c_compressor.getPressureSwitchValue());
    SmartDashboard.putNumber("Shoot Speed", this.getShooterSpeeed());
    SmartDashboard.putString("Auto Switch", switchAutoMode.get() ? "4 Point" : "6 Point");
    //ultrasonic
    SmartDashboard.putNumber("ultrasonicSensorRangeInches", getultrasonicSensorRangeInches());
    SmartDashboard.putBoolean("Shoot Safe", getultrasonicSensorRangeInches() > shooterSafeShootDistanceInches ? false : true);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Reset Incoders when Enabled button is clicked.
    enc_AirSide.reset();
    enc_RioSide.reset();

    intakeToggle = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // ********************************
    // * DRIVE CODE
    // ********************************
    // Drive with arcade drive. Y axis drives forward and backward, and the X turns left and right.
    // Shooter side is the front of the robot.

    speedMultiplier = (((flight.getRawAxis(flightPaddle) * -1) * 0.2) + .6);
    drive_Main.arcadeDrive(flight.getY() * speedMultiplier, flight.getX() * xCorrect);

    // Flip Logic
    if (flight.getRawButton(flight2)) {
      if (forwardDriveToggle == true) {
        forwardDriveToggle = false;
        mg_AirSide.setInverted(false);
        mg_RioSide.setInverted(true);
        xCorrect = Math.abs(xCorrect) * -1;
      } else {
        forwardDriveToggle = true;
        mg_AirSide.setInverted(true);
        mg_RioSide.setInverted(false);
        xCorrect = Math.abs(xCorrect);
      }
      // prevent doubleclick of the flip button
      try {
        Thread.sleep(300);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    // ********************************
    // * SHOOTER & INTAKE
    // ********************************
    if (intakeToggle) { // completely Disable shooter
      if (flight.getRawButton(flight1)) {
        mg_Shooter.set(getShooterSpeeed());
      } else if (controller.getRawButton(rbButton)) { // controller X button
        mg_Shooter.set(getShooterSpeeed());
      } else if (controller.getRawButton(lbButton)) { // reverse balls with sensor on
        mg_Shooter.set(-.75);
      } else {
        mg_Shooter.stopMotor();
      }

      // INTAKE MANAGEMENT
      if (controller.getRawButton(yButton)) { // Reverse Dump other team ball
        mg_Shooter.set(intakeDumpSpeed);
      } else if (s_ballSensor.get() == true) {
        mg_Intake.set(intakeSpeed);
      }
    }

    if (flight.getRawButton(flight3)) {
      if (intakeToggle) {
        intakeToggle = false;
      } else {
        intakeToggle = true;
      }
      // Prevent double click
      try {
        Thread.sleep(300);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    if (controller.getRawButton(backButton) == true && controller.getRawButton(startButton) == true) {
      climbPart1();
      climbPart2();
      if (!bClimberAbort) {
       climbPart3();
      } else {
        bClimberHooked = false;
      }
    }

    // Small manual
    if (flight.getRawButton(flight7) == true) {
      solenoidShort.set(DoubleSolenoid.Value.kReverse);
      wait(25);
      solenoidShort.set(DoubleSolenoid.Value.kOff);
    } else if (flight.getRawButton(flight8) == true) {
      solenoidShort.set(DoubleSolenoid.Value.kForward);
      wait(25);
      solenoidShort.set(DoubleSolenoid.Value.kOff);
    }
    // Medium manual
    if (flight.getRawButton(flight9) == true) {
      solenoidMedium.set(DoubleSolenoid.Value.kReverse);
      wait(25);
      solenoidMedium.set(DoubleSolenoid.Value.kOff);
    } else if (flight.getRawButton(flight10) == true) {
      solenoidMedium.set(DoubleSolenoid.Value.kForward);
      wait(25);
      solenoidMedium.set(DoubleSolenoid.Value.kOff);
    }
    // Long manual
    if (flight.getRawButton(flight11) == true) {
      solenoidLong.set(DoubleSolenoid.Value.kReverse);
      wait(25);
      solenoidLong.set(DoubleSolenoid.Value.kOff);
    } else if (flight.getRawButton(flight12) == true) {
      solenoidLong.set(DoubleSolenoid.Value.kForward);
      wait(25);
      solenoidLong.set(DoubleSolenoid.Value.kOff);
    }

    if (controller.getRawButton(xButton)) {

    } else {

    }

  } // END teleopPeriodic

  // ********************************
  // * AUTONOMOUS SECTION
  // ********************************
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    printInConsole("0 autoRunCounter: " + autoRunCounter);
    printInConsole("0 auto6BackCargo: " + auto6BackCargo);
    printInConsole("0 autoBallShot: " + autoBallShot);
    printInConsole("0 getultrasonicSensorRangeInches: " + getultrasonicSensorRangeInches());
    printInConsole("0 getShooterSpeeed(): " + this.getShooterSpeeed());
    printInConsole("0 enc_RioSide.getDistance(): " + enc_RioSide.getDistance());
    printInConsole("0 enc_AirSide.getDistance(): " + enc_AirSide.getDistance());
    printInConsole("------------------------------------------------------");

    while (isAutonomous() && isEnabled()) {
      
      // 6 POINT SWITCH
      if (switchAutoMode.get() == false) {        
        // 1 . Shoot
        if (autoRunCounter == 0 && !autoBallShot) {
          mg_Shooter.set(getShooterSpeeed());
          wait(1500); // run and make sure balls clear
          mg_Shooter.stopMotor();
          autoBallShot = true;
        }

        // 2 . Backup
        if (autoRunCounter == 0 && autoBallShot) {
          driveStraightWithEncoder(-.6, 7);
          while (getultrasonicSensorRangeInches() > 15) {
            driveStraightWithEncoder(.6, .5);
          }
          drive_Main.tankDrive(0, 0);
          wait(10);
          mg_Shooter.set(getShooterSpeeed());
          wait(1500); // run and make sure balls clear
          drive_Main.stopMotor();
          mg_Shooter.stopMotor();
          autoRunCounter++;
          printInConsole("7454: Auto6Points complete");
          autoRunCounter++;
        }  
      } // END 6 POINT SWITCH   
      else {
        // 4 POINTS
        printInConsole("7454: Auto4Points: Selected");

        // 1 . Shoot
        if (autoRunCounter == 0 && !autoBallShot) {
          //shoot
          mg_Shooter.set(getShooterSpeeed());
          wait(1800); // run and make sure balls clear
          mg_Shooter.stopMotor();
          autoBallShot = true;
        }

        // 2 . Backup
        if (autoRunCounter == 0 && autoBallShot) {
          driveStraightWithEncoder(-.6, 5.2);
          autoRunCounter++;
          printInConsole("7454: Auto4Points complete");
        }
      } // 4 POINT SWITCH
    } // END while(isAutonomous() && isEnabled()) {
  } // END autonomousPeriodic()

  // ********************************
  // * CUSTOM FUNCTIONS
  // ********************************

  /** This function creates a wait */
  public static void wait(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }

  // This function will raise medium solenoid, push out small, and move robot into
  // posistion using limit sensors
  public void climbPart1() {
    bClimberHooked = false;
    bClimberAbort = false;

    if (!forwardDriveToggle) {
      mg_AirSide.setInverted(true);
      mg_RioSide.setInverted(false);
      xCorrect = xCorrect * -1;
    }
    solenoidMedium.set(DoubleSolenoid.Value.kReverse);
    wait(150);
    solenoidMedium.set(DoubleSolenoid.Value.kOff);
    solenoidShort.set(DoubleSolenoid.Value.kReverse);
    wait(100);
    solenoidShort.set(DoubleSolenoid.Value.kOff);
  }

  // This function will get the medium hook on the bar
  public void climbPart2() {
    while (!bClimberHooked) {
      if (flight.getRawButton(flight4) == true) { // abort climb
        printInConsole("7454: Climb  - Abort.");
        drive_Main.stopMotor();
        bClimberHooked = true;
        bClimberAbort = true;
      } else if (ls_climbRioSide.get() && ls_climbAirSide.get()) { // zero touching
        drive_Main.arcadeDrive(-.55, 0);
        printInConsole("7454: Climb - no touch");
      } else if (ls_climbRioSide.get() && !ls_climbAirSide.get()) { // AirSide is touching, RioSide is not = turn right
        mg_RioSide.set(-.40);
        mg_AirSide.set(-.10);
        printInConsole("7454: Climb  - Airside touching, Rioside not");
      } else if (!ls_climbRioSide.get() && ls_climbAirSide.get()) { // RioSide is touching, AirSide is not = turn left
        mg_AirSide.set(-.40);
        mg_RioSide.set(-.10);
        printInConsole("7454: Climb  - Airside touching, Rioside not");
      } else if (!ls_climbRioSide.get() && !ls_climbAirSide.get()) { // both touching
        printInConsole("7454: Climb  - Hooked!!!");
        drive_Main.stopMotor();
        bClimberHooked = true;
      }
    }
  }

  // This function will complete the climb, medium in, long out, short in, long in
  public void climbPart3() {
    solenoidMedium.set(DoubleSolenoid.Value.kForward);
    wait(3500);
    solenoidMedium.set(DoubleSolenoid.Value.kOff);
    solenoidLong.set(DoubleSolenoid.Value.kReverse);
    wait(3000);
    solenoidLong.set(DoubleSolenoid.Value.kOff);
    solenoidShort.set(DoubleSolenoid.Value.kForward);
    wait(1300);
    solenoidShort.set(DoubleSolenoid.Value.kOff);
    solenoidLong.set(DoubleSolenoid.Value.kForward);
    wait(2000);
    solenoidLong.set(DoubleSolenoid.Value.kOff);
    bClimberHooked = false;
  }

  /**
   * What to set shooter speed to based on lidar distance. We set the shooter motors to 100%
   * to determind shooterMaxDistance. Then we pit against wall to get shooterMinMotorSpeed.
   * Formula is the minSpeed + (the current lidar distance/(maxdistance/(max speed-min speed)));
   * If the distance is farther than the shooterMaxDistance, then motors will run at 100%.
   * Distance and speed variables are set in globals.
   * 
   * @return Speed to set Shoot Motors
   */
  public double getShooterSpeeed() {
    if (bUseDistanceSensorToShoot) {
      double sensorDistanceInches = getultrasonicSensorRangeInches();
      
      if (sensorDistanceInches > shooterMaxDistance) {
        // too far, set motor speed to 100%
        return 1;
      }
      return shooterMinMotorSpeed + (sensorDistanceInches / (shooterMaxDistance / (shooterMaxMotorSpeed - shooterMinMotorSpeed)));
    }
    else {
      // in auto mode without sensor we want a little extra bump to make sure ball makes it.
      double extraShooterPower = 0;      
      if (isAutonomous() && isEnabled()) {
        extraShooterPower = .05;
      }      
      return shooterDefaultMotorSpeed + extraShooterPower ;
    }
  }

  /**
   * Gets inches 
   * 
   * @return distance to 
   */
  public double getultrasonicSensorRangeInches() {
    if (bUseDistanceSensorToShoot) {
      return ultrasonicSensor.getRangeInches() - ultrasonicOffset;
    }
    else {
      return 999;
    }
  }

  // TODO - Test this
  /**
  * Drive Straight using encoders
  * @parm power = Speed to drive (negative intake side, postive shooter side)
  * @parm distance = how far to drive in feet
  */
  public void driveStraightWithEncoder(double power, double distance) {
     double error = 0;
     double kP = 1; // Proportional control for driving straight
     boolean gotCargo = false;
     mg_AirSide.setInverted(false);
     mg_RioSide.setInverted(true);

     // reset encoders
     enc_RioSide.reset();
     enc_AirSide.reset();

     printInConsole("7454: driveStraightWithEncoder Stating");

    while (((Math.abs((enc_RioSide.getDistance() + enc_AirSide.getDistance())/2)) < distance) && !gotCargo) {
     error = enc_RioSide.getDistance() - enc_AirSide.getDistance();
     drive_Main.tankDrive(power + kP * error, power - kP * error);
     if (s_ballSensor.get() == true) {
      drive_Main.stopMotor();
      mg_Intake.set(intakeSpeed);
      wait(400);
      mg_Shooter.stopMotor();
      auto6BackCargo = true;
      gotCargo = true;
      printInConsole("7454: driveStraightWithEncoder: Got Cargo");
     }
    }
     drive_Main.stopMotor();
     gotCargo = false;
     printInConsole("7454: driveStraightWithEncoder Stoping");
  }

  public void printInConsole(String x) {
    if (debugMode = true) {
      System.out.println(x);
    }
  }

  // ********************************
  // * MODES WE DON"T USE, BUT COULD
  // ********************************
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    driveStraightWithEncoder(.5, 2);
    wait(1500);
    driveStraightWithEncoder(-.5, 2);
    wait(1500);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

} // END Robot