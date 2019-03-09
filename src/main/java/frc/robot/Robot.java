/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
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
  private Solenoid driveTrainShift;

  // Climber Piston
  private Solenoid frontClimb;
  private Solenoid backClimb;
  private Solenoid nullSettingFront;
  private Solenoid nullSettingBack;

  // Climber Pause
  private boolean climberPause;

  // Claw Pistons
  private Solenoid clawPiston;

  // Preasure Reading
  private AnalogInput preasureRead;

  // Gyro
  private AHRS ahrs;
  private PIDController turnController;
  private double rotateToAngleRate;
  private double targetAngle;

  // Camera

  // private UsbCamera camera =
  // CameraServer.getInstance().startAutomaticCapture("intake",
  // "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");

  private UsbCamera camera;
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
  private DigitalInput limitForebar;

  @Override
  public void robotInit() {
    // Inits camera
    camera = CameraServer.getInstance().startAutomaticCapture();

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

    claw = new TalonSRX(2);
    leftClimb = new TalonSRX(1);
    rightClimb = new TalonSRX(0);

    // Limit Switches
    limitTopE = new DigitalInput(2);
    limitBottomE = new DigitalInput(0);
    limitForebar = new DigitalInput(1);

    // Inits the solenoids
    frontClimb = new Solenoid(4);
    backClimb = new Solenoid(3);
    clawPiston = new Solenoid(1);
    nullSettingFront = new Solenoid(2);
    nullSettingBack = new Solenoid(5);
    driveTrainShift = new Solenoid(0);

    // Inits preasure reader
    preasureRead = new AnalogInput(0);

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
      // ahrs = new AHRS(SerialPort.Port.kUSB);
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
    elevatorHeights();
    setClimber(controlPanel.getRawButton(9), controlPanel.getRawButton(7));
    setClawPiston(controlPanel.getRawButton(5));
    setClawMotors(controlPanel.getRawButton(1), controlPanel.getRawButton(8));

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
      forebar.set(1.0);
    } else if (controlPanel.getRawButton(10)) {
      forebar.set(-1.0);
    } else {
      forebar.set(0);
    }
  }

  public void setClawPiston(boolean state) {
    // Code setting the state of the claw pistons.
    clawPiston.set(state);
  }

  public void setClawMotors(boolean in, boolean out) {
    if (in) {
      claw.set(ControlMode.PercentOutput, .4);
    } else if (out) {
      claw.set(ControlMode.PercentOutput, -.4);
    } else {
      claw.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setClimber(boolean front, boolean back) {
    if (controlPanel.getRawButton(6)) {
      System.out.println("go forward");
      leftClimb.set(ControlMode.PercentOutput, .35);
      rightClimb.set(ControlMode.PercentOutput, .35);
    } else if (controlPanel.getRawButton(3)) {
      System.out.println("go backwards");
      leftClimb.set(ControlMode.PercentOutput, -.35);
      rightClimb.set(ControlMode.PercentOutput, -.35);
    } else {
      leftClimb.set(ControlMode.PercentOutput, 0);
      rightClimb.set(ControlMode.PercentOutput, 0);
    }

    if (controlPanel.getRawButtonReleased(4)) {
      if (front && back) {

      } else {
        nullSettingFront.set(true);
        nullSettingBack.set(true);
      }
    } else {
      nullSettingFront.set(false);
      nullSettingBack.set(false);

      if (front && back) {
        if (ahrs.getRoll() > 1.5) { // 1.5 is my fav num.... fight me
          frontClimb.set(true);
          nullSettingBack.set(false);
          nullSettingFront.set(true);
          backClimb.set(true);
        } else {
          frontClimb.set(true);
          nullSettingBack.set(false);

          nullSettingFront.set(false);
          backClimb.set(true);
        }
      } else {
        nullSettingBack.set(false);
        nullSettingFront.set(false);
        if (front) {
          frontClimb.set(true);
          backClimb.set(false);
        } else if (back) {
          frontClimb.set(false);
          backClimb.set(true);
        } else {
          frontClimb.set(false);
          backClimb.set(false);
        }
      }
    }
  }

  public double readPreasure() {
    return 250.0 * preasureRead.getVoltage() / 5.0 - 25.0;
  }

  // Shuffleboard

  public void displayShuffleboard() {
    SmartDashboard.putBoolean("Gear Shift", shiftMode);

    if (isPOVup(controlPanel)) { // lvl 1
      SmartDashboard.setDefaultNumber("Elevtor Level", 1);
    } else if (isPOVdown(controlPanel)) { // lvl 2
      SmartDashboard.setDefaultNumber("Elevator Level", 2);
    }

    SmartDashboard.putNumber("Preasure", readPreasure());
  }

  // Auton

  public void elevatorHeights() {
    if (limitBottomE.get()) {
      System.out.print("Bottom Limit Switch");
    }

    // This is for the elevator up button
    if (isPOVright(controlPanel)) {
      System.out.println("up button");
      elevator.set(-.35);
    }
    // This is for the elevator down button
    else if (isPOVleft(controlPanel)) {
      System.out.println("down button");
      elevator.set(.35);
    } else {
      elevator.set(0);
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