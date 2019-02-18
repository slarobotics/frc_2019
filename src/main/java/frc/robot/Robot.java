/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

  // Arm Motors
  // private TalonSRX arm;

  // Gear Shift
  private Solenoid driveTrainShift = new Solenoid(0);

  // Climber Piston
  private Solenoid frontClimb;
  private Solenoid backClimb;

  // Arm Pistons
  private Solenoid armPiston;

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
    forebar = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless); //

    // arm = new TalonSRX(0); // TODO: Make sure this # is right

    // Inits the solenoids
    frontClimb = new Solenoid(3);
    backClimb = new Solenoid(1);
    armPiston = new Solenoid(2);

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
      ahrs = new AHRS(SPI.Port.kMXP);
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
    // runElevator();
    elevatorHeights();

    setClimber(controlPanel.getRawButton(6), controlPanel.getRawButton(3));
    setArmPiston(controlPanel.getRawButton(7));
    setArmMotors(controlPanel.getRawButton(8), controlPanel.getRawButton(1));

    // Auton
    if (controlPanel.getRawButtonReleased(4)) {
      autoAlignEnabled = !autoAlignEnabled;
      if (autoAlignEnabled) {
        autoAlign();
      }
    }

    displayShuffleboard();
  }

  @Override
  public void teleopPeriodic() {
    controlLoop();
  }

  @Override
  public void autonomousPeriodic() {
    controlLoop();
  }

  public void driveRobot() {
    double scale = getDrivePowerScale();
    double leftSpeed = scale * leftStick.getY();
    double rightSpeed = scale * rightStick.getY();
    setDriveMotors(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("speed", Math.max(leftSpeed, rightSpeed));
  }

  public void displayShuffleboard() {
    SmartDashboard.putBoolean("Gear Shift", shiftMode);
    SmartDashboard.putNumber("Elevator Level", 1); // TODO
  }

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

  public void gearShift() {
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

  public void runForebar() {
    if (controlPanel.getRawButton(8)) {
      System.out.println("8");
      setForebar(0.25);
    } else if (controlPanel.getRawButton(1)) {
      System.out.println("1");
      setForebar(-0.25);
    } else {
      setForebar(0);
    }
  }

  public void setArmPiston(boolean state) {
    armPiston.set(state);
  }

  public void setArmMotors(boolean in, boolean out) {
    boolean isNegative;
    double speed = .75;
    if (in) {
      isNegative = false;
    } else if (out) {
      isNegative = true;
    } else {
      isNegative = false;
      speed = 0;
    }

    /*
     * if (speed == 0) { arm.set(ControlMode.PercentOutput, 0); } else { for (int i
     * = 0; i <= (speed * 1000); i++) { if (isNegative) {
     * arm.set(ControlMode.PercentOutput, (i / 1000) * -1); } else {
     * arm.set(ControlMode.PercentOutput, i / 1000); } Timer.delay(.0001); } }
     */
  }

  public void setClimber(boolean front, boolean back) {
    frontClimb.set(front);
    backClimb.set(back);
  }

  public void setDriveMotors(double left, double right) {
    leftTop.set(left);
    rightTop.set(right);
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

  public void runElevator() { // incremental up/down elevator buttons (temporary, POV buttons not working)
    if (controlPanel.getRawButton(8)) {
      setElevator(0.25);
      System.out.println("8");
    } else if (controlPanel.getRawButton(1)) {
      System.out.println("1");
      setElevator(-0.25);
    } else {
      setElevator(0);
    }
  }

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

  public void elevatorHeights() {
    // This is the // This is for the elevator up button
    if (isPOVright(controlPanel)) {
      System.out.println("up button");
      setElevator(.25); // Edited to go slow
    }
    // This is for the elevator down button
    else if (isPOVleft(controlPanel)) {
      System.out.println("down button");
      setElevator(-.25);
    }
    // This is the lvl 1 button
    else if (isPOVup(controlPanel)) {
      setElevator(0);
    }
    // This is the lvl 2 button
    else if (isPOVdown(controlPanel)) {
      setElevator(.25);
      if (elevator.getEncoder().getPosition() >= 69 || elevator.getEncoder().getPosition() <= 70) {
        setElevator(0);
      }
      // This is the lvl 3 button
    } else if (controlPanel.getRawButton(4)) {
      setElevator(.25);
      if (elevator.getEncoder().getPosition() >= 95 || elevator.getEncoder().getPosition() <= 96) {
        setElevator(0);
      }
    } else {
      setElevator(0);
    }
  }

  /*
   * public void runElevator() { if (isPOVright(controlPanel)) { setElevator(.75);
   * } else if (isPOVleft(controlPanel)) { setElevator(-.75); } else {
   * setElevator(0); } }
   */

  public void forebarAngles() { // runs forebar motor until reaches ideal angle(s)
    // lvl 1
    if (isPOVup(controlPanel)) {
      setForebar(-.25);
      if (forebar.getEncoder().getPosition() >= -111 || forebar.getEncoder().getPosition() <= -110) {
        setForebar(0);
      }
      // lvl 3
    } else if (controlPanel.getRawButton(4)) {
      setForebar(.25);
      System.out.println("button pressed, motor on");
      if (forebar.getEncoder().getPosition() >= 215 || forebar.getEncoder().getPosition() <= 216) {
        setForebar(0);
        System.out.println("motor off");
      }
    }
  }

  public static double calculateTotalHeight(double arm_angle) { // Returns inches
    // This function assumes that we are perfectly mounted at 90 degrees at the
    // start going up and down at that point

    // Constants in inches
    double arm_width = 3.633; // Width of arm mechanism
    double arm_length = 44.59; // length of arm mechanism
    int elevator_height = 56; // height of elevator mechanism
    int elevator_width = 17; // width of elevator mechanism

    return (Math.sin(Math.toRadians(arm_angle)) * (arm_length - elevator_width)) + (elevator_height - arm_width)
        + arm_width;
  }

  @Override
  /* This function is invoked periodically by the PID Controller, */
  /* based upon navX-MXP yaw angle input and PID Coefficients. */
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }
}