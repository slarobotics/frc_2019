/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.vision.VisionThread;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends TimedRobot implements PIDOutput {

  // Joysticks
  private Joystick leftStick;
  private Joystick rightStick;
  private Joystick controlPanel;

  // Drivetrain Motors
  private CANSparkMax leftTop;
  private CANSparkMax rightTop;
  private CANSparkMax leftBottom;
  private CANSparkMax rightBottom;

  // Elevator Motors
  private CANSparkMax elevator;

  // Forebar Motors
  private CANSparkMax forebar;

  // Claw Motors
  private TalonSRX claw;

  // Climber Motors
  private TalonSRX leftClimb;
  private TalonSRX rightClimb;

  // Gear Shift
  private Solenoid driveTrainShift = new Solenoid(0);

  // Climber Piston
  private Solenoid frontClimb;
  private Solenoid backClimb;
  private Solenoid nullSetting;

  // Claw Pistons
  private Solenoid clawPiston;

  // Gyro
  private AHRS ahrs;
  private PIDController turnController;
  private double rotateToAngleRate;
  private double targetAngle;

  // Camera

  // private UsbCamera camera =
  // CameraServer.getInstance().startAutomaticCapture("intake",
  // "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");

  private UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  private CvSource outputStream;
  private VisionThread gripPipeline;
  private final Object imgLock = new Object();

  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system. Note that the */
  /* SmartDashboard in Test mode has support for helping you tune */
  /* controllers by displaying a form where you can enter new P, I, */
  /* and D constants and test the mechanism. */

  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  /* This tuning parameter indicates how close to "on target" the */
  /* PID Controller will attempt to get. */

  static final double kToleranceDegrees = 2.0f;

  // Shifting Mode
  public boolean shiftMode = false;

  // Auto Align
  public boolean autoAlignEnabled = false;
  public double distanceToTarget;

  // Limit Switches
  private DigitalInput limitBottomE;
  private DigitalInput limitTopE;

  @Override
  public void robotInit() {
    // Inits the Joysticks
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    controlPanel = new Joystick(2);

    // Inits the Motors
    leftTop = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless); // roborio side, near roborio
    leftTop.setInverted(true);
    leftBottom = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless); // roborio side, far from rio
    leftBottom.follow(leftTop, false);
    rightTop = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless); // pdp side, near pdp
    rightBottom = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless); // pdp side, far from pdp
    rightBottom.follow(rightTop, false);
    elevator = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless); // under elevator
    forebar = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless); //

    claw = new TalonSRX(0); // TODO: Make sure this # is right

    // Inits the solenoids
    frontClimb = new Solenoid(3);
    backClimb = new Solenoid(1);
    clawPiston = new Solenoid(0);
    nullSetting = new Solenoid(2);

    // Inits Vision Pipeline
    outputStream = CameraServer.getInstance().putVideo("overlay", 320, 240);
    camera.setResolution(320, 240);
    gripPipeline = new VisionThread(camera, new GripPipeline(), pipeline -> { // No point in drawling nothing is there.
      if (!pipeline.filterContoursOutput().isEmpty()) { // Puts overlay output on camera stream
        outputStream.putFrame(pipeline.overlayOutput);
        synchronized (imgLock) { // Will let us grab data from GripPipeline
          targetAngle = pipeline.targetAngle;
          distanceToTarget = pipeline.targetAngle;
        }
      }
    });
    gripPipeline.start();

    // Inits Gyro
    try {
      ahrs = new AHRS(SerialPort.Port.kUSB);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }

    turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);

    ahrs.zeroYaw();
  }

  public void controlLoop() {
    // Drive Train
    gearShift();
    driveRobot();

    // Mechs
    runForebar();
    forebarAngles();
    elevatorHeights();
    setClimber(controlPanel.getRawButton(9), controlPanel.getRawButton(7));
    setClawPiston(controlPanel.getRawButton(5));
    setClawMotors(controlPanel.getRawButton(8), controlPanel.getRawButton(1));

    // Auton
    /*
     * if (controlPanel.getRawButtonReleased(2)) { autoAlignEnabled =
     * !autoAlignEnabled; if (autoAlignEnabled) { autoAlign(); } }
     */
    // Shuffleboard
    displayShuffleboard();
  }

  // Control Loops

  @Override
  public void teleopPeriodic() {
    controlLoop();
  }

  @Override
  public void autonomousPeriodic() {
    controlLoop();
  }

  // Drivetrain

  public void driveRobot() {
    double scale = getDrivePowerScale();
    double leftSpeed = scale * leftStick.getY();
    double rightSpeed = scale * rightStick.getY();
    setDriveMotors(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("speed", Math.max(leftSpeed, rightSpeed));
  }

  public void gearShift() {
    // This method checks what gear shift mode we should be in.
    if (leftStick.getRawButtonReleased(2)) {
      shiftMode = true;
    } else if (rightStick.getRawButtonReleased(2)) {
      shiftMode = false;
    }

    driveTrainShift.set(shiftMode);
  }

  public double getDrivePowerScale() {
    /*
     * This figures out what to set the drive train speed to.
     * 
     * No trigger = 0.65
     * 
     * One trigger = 0.85
     * 
     * Both triggers = 1.0
     */

    double scale = 0.65;

    if (leftStick.getTrigger() || rightStick.getTrigger()) {
      scale = 0.85;
    }

    if (leftStick.getTrigger() && rightStick.getTrigger()) {
      scale = 1;
    }

    return scale;
  }

  public void setDriveMotors(double left, double right) {
    leftTop.set(left);
    rightTop.set(right);
  }

  // Mechs

  public void runForebar() {
    // Code for 4bar mech... nothing too complex.

    if (controlPanel.getRawButton(2)) {
      setForebar(0.5);
    } else if (controlPanel.getRawButton(1)) { // TODO: change button
      setForebar(-0.5);
    } else {
      setForebar(0);
    }
  }

  public void setClawPiston(boolean state) {
    // Code setting the state of the claw pistons.
    clawPiston.set(state);
  }

  public void setClawMotors(boolean in, boolean out) {
    if (in) {
      claw.set(ControlMode.PercentOutput, .75);
    } else if (out) {
      claw.set(ControlMode.PercentOutput, -.75);
    } else {
      claw.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setClimber(boolean front, boolean back) {
    if (controlPanel.getRawButton(6)) {
      leftClimb.set(ControlMode.PercentOutput, .5);
      rightClimb.set(ControlMode.PercentOutput, .5);
    } else if (controlPanel.getRawButton(3)) {
      leftClimb.set(ControlMode.PercentOutput, -.5);
      rightClimb.set(ControlMode.PercentOutput, -.5);
    }
    System.out.println(ahrs.getRoll());
    if (front && back) {
      /*
       * if (ahrs.getRoll() < -15) { frontClimb.set(true); backClimb.set(false); }
       * else
       */
      if (ahrs.getRoll() > 1.5) { // 1.5 is my fav num.... fight me
        frontClimb.set(true);
        nullSetting.set(true);
        backClimb.set(true);
      } else {
        frontClimb.set(true);
        nullSetting.set(false);
        backClimb.set(true);
      }
    } else {
      if (front) {
        frontClimb.set(true);
        backClimb.set(false);
        nullSetting.set(false);
      } else if (back) {
        frontClimb.set(false);
        backClimb.set(true);
        nullSetting.set(true);
      } else {
        frontClimb.set(false);
        backClimb.set(false);
        nullSetting.set(false);
      }
    }
  }

  public void setForebar(double speed) { // makes forebar motors speed up incrementally so mech doesn't go from 0 to 100
                                         // and break
    boolean isNegative = false;
    if (speed < 0) {
      isNegative = true;
      speed = speed * -1;
    }

    if (speed == 0) {
      forebar.set(0);
    } else {
      for (int i = 0; i <= (speed * 1000); i++) {
        if (isNegative) {
          forebar.set((i / 1000) * -1);
        } else {
          forebar.set(i / 1000);
        }
        Timer.delay(.0001);
      }
    }
  }

  /*
   * public void runElevator() { // incremental up/down elevator buttons
   * (temporary, POV buttons not working) if (controlPanel.getRawButton(8)) {
   * setElevator(0.5); } else if (controlPanel.getRawButton(1)) {
   * setElevator(-0.5); } else { setElevator(0); } }
   */
  public void setElevator(double speed) { // makes elevator motors speed up incrementally so mech doesn't go from 0 to
                                          // 100 and break
    boolean isNegative = false;
    if (speed < 0) {
      isNegative = true;
      speed = speed * -1;
    }

    if (speed == 0) {
      elevator.set(0);
    } else {
      for (int i = 0; i <= (speed * 1000); i++) {
        if (isNegative) {
          elevator.set((i / 1000) * -1);
        } else {
          elevator.set(i / 1000);
        }
        Timer.delay(.0001);
      }
    }
  }

  // Shuffleboard

  public void displayShuffleboard() {
    SmartDashboard.putBoolean("Gear Shift", shiftMode);

    if (isPOVup(controlPanel)) { // lvl 1
      SmartDashboard.setDefaultNumber("Elevtor Level", 1);
    } else if (isPOVdown(controlPanel)) { // lvl 2
      SmartDashboard.setDefaultNumber("Elevator Level", 2);
    } else if (controlPanel.getRawButton(4)) { // lvl 3
      SmartDashboard.setDefaultNumber("Elevator Level", 3);
    }
  }

  // Auton

  public void autoAlign() {
    // TODO: Score!

    while (autoAlignEnabled) {
      if (!turnController.isEnabled()) {
        turnController.setSetpoint(targetAngle);
        rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        turnController.enable();
      }

      System.out.println("Angle " + ahrs.getAngle());
      System.out.println("left: " + rotateToAngleRate + " right: " + (rotateToAngleRate * -1));

      if (rotateToAngleRate <= 0.05) { // TODO: Is this right?
        if (distanceToTarget >= 10) { // TODO: Get real distance.
          autoAlignEnabled = false;
          setDriveMotors(0, 0);
        } else {
          setDriveMotors(0.1, 0.1);
        }
      } else {
        setDriveMotors(rotateToAngleRate, (rotateToAngleRate * -1));
      }
    }
  }

  public void elevatorHeights() {
    // This is for the elevator up button
    if (!limitTopE.get() && isPOVright(controlPanel)) {
      System.out.println("up button");
      setElevator(.5);
    }
    // This is for the elevator down button
    else if (!limitBottomE.get() && isPOVleft(controlPanel)) {
      System.out.println("down button");
      setElevator(-.5);
    }
    // This is the lvl 2 button
    else if (!limitTopE.get() && isPOVdown(controlPanel)) {
      setElevator(.5);
      if (elevator.getEncoder().getPosition() >= 69 && elevator.getEncoder().getPosition() <= 70) {
        setElevator(0);
      }
      // This is the lvl 3 button
    } else if (!limitTopE.get() && controlPanel.getRawButton(4)) {
      setElevator(.5);
      if (elevator.getEncoder().getPosition() >= 95 && elevator.getEncoder().getPosition() <= 96) {
        setElevator(0);
      }
    } else {
      setElevator(0);
    }
  }

  public void forebarAngles() { // runs forebar motor until reaches ideal angle
    // lvl 3
    if (controlPanel.getRawButton(4)) {
      setForebar(.5);
      System.out.println("button pressed, motor on");
      if (forebar.getEncoder().getPosition() >= 215 && forebar.getEncoder().getPosition() <= 216) {
        setForebar(0);
        System.out.println("motor off");
      }
      // lvl 1
    } else if (isPOVup(controlPanel)) {
      setForebar(-.5);
      if (forebar.getEncoder().getPosition() >= -111 && forebar.getEncoder().getPosition() <= -110) {
        setForebar(0);
      }
    }
  }

  // Extras: Methods that help us program the robot

  public boolean isPOVup(Joystick joystick) { // creates boolean to use later for lvl 1 control panel button
    return joystick.getPOV() == 0;
  }

  public boolean isPOVdown(Joystick joystick) { // creates boolean to use later for lvl 1 control panel button
    return joystick.getPOV() == 180;
  }

  public boolean isPOVright(Joystick joystick) { // creates boolean for POV right button
    return joystick.getPOV() == 270;
  }

  public boolean isPOVleft(Joystick joystick) { // creates boolean for POV left button
    return joystick.getPOV() == 90;
  }

  public static double calculateTotalHeight(double forebar_angle) { // Returns inches
    // This function assumes that we are perfectly mounted at 90 degrees at the
    // start going up and down at that point

    // Constants in inches
    double forebar_width = 3.633; // Width of forebar mechanism
    double forebar_length = 44.59; // length of forebar mechanism
    int elevator_height = 56; // height of elevator mechanism
    int elevator_width = 17; // width of elevator mechanism

    return (Math.sin(Math.toRadians(forebar_angle)) * (forebar_length - elevator_width))
        + (elevator_height - forebar_width) + forebar_width;
  }

  @Override
  /* This function is invoked periodically by the PID Controller, */
  /* based upon navX-MXP yaw angle input and PID Coefficients. */
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }
}