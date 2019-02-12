/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

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
  private TalonSRX arm;

  // Gear Shift
  // DoubleSolenoid driveTrainShift = new DoubleSolenoid(0, 0, 1);

  // Climber Pistons
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
  private VisionThread rectDraw;
  private double centerX = 0.0;
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

  @Override
  public void robotInit() {
    // Inits the Joysticks
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    controlPanel = new Joystick(2);

    // Inits the Motors
    leftTop = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftBottom = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightTop = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBottom = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevator = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    forebar = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless); // TODO: Make sure this # is right
    arm = new TalonSRX(0);

    // Inits the solenoids
    frontClimb = new Solenoid(0);
    backClimb = new Solenoid(1);
    armPiston = new Solenoid(2);

    // Inits Vision Pipeline
    outputStream = CameraServer.getInstance().putVideo("overlay", 320, 240);
    camera.setResolution(320, 240);

    gripPipeline = new VisionThread(camera, new GripPipeline(), pipeline -> {
      // No point in drawling nothing is there.
      if (!pipeline.filterContoursOutput().isEmpty()) {
        // Puts overlay output on camera stream
        outputStream.putFrame(pipeline.overlayOutput);
        synchronized (imgLock) { // let us grab data from GripPipeline
          targetAngle = pipeline.targetAngle;
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

  @Override
  public void teleopPeriodic() {
    // Code for driving robot.
    double scale = getDrivePowerScale();
    double leftSpeed = scale * leftStick.getY();
    double rightSpeed = scale * rightStick.getY();
    adaptiveDrive(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("speed", Math.max(leftSpeed, rightSpeed));

    gearShift();

    setClimber(controlPanel.getRawButton(9), controlPanel.getRawButton(5));
    setArmPiston(controlPanel.getRawButton(7));
    setArmMotors(controlPanel.getRawButton(8), controlPanel.getRawButton(1));

    autoAlign();

    displayShuffleboard();

    System.out.println("POV " + controlPanel.getPOV());

    if (controlPanel.getRawButton(3)) { // TODO: Can this be removed?
      System.out.print("Button 3 is clicked.");
      leftBottom.set(1);
    } else {
      leftBottom.set(0);
    }

    elevatorHeights();
  }

  // Future coders... always use this.
  public void adaptiveDrive(double l, double r) {
    // alpha is a parameter between 0 and 1
    final double alpha = 0.5;
    double c = 0.5 * (l + r);
    double d = 0.5 * (l - r);
    double scale = (1 - (alpha * c * c));
    d *= scale;

    // GYRO CORRECTION -- high if d is close to zero, low otherwise
    double gRate = ahrs.getRate();
    final double CORR_COEFF = 0.5;
    double corr = 0.0;
    if (Math.abs(d) < 0.05)
      corr = (gRate * Math.abs(c) * CORR_COEFF * (1 - Math.abs(d)));
    d -= corr;

    double l_out = c + d;
    double r_out = c - d;

    setDriveMotors(l_out, r_out);
  }

  public void displayShuffleboard() {
    SmartDashboard.putBoolean("Gear Shift", shiftMode);
    SmartDashboard.putNumber("Elevator Level", 1); // TODO
  }

  public void autoAlign() {
    /*
     * TODO:
     * 
     * - Wire to button/motor
     * 
     * - Make the robot go forward based on the distance
     * 
     * - Score!
     */

    if (!turnController.isEnabled()) {
      turnController.setSetpoint(-20.0f);
      rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
      turnController.enable();
    }

    System.out.println("Angle " + ahrs.getAngle());

    // TODO: May need to be both positive.
    System.out.println("left: " + rotateToAngleRate + " right: " + (rotateToAngleRate * -1));

    if (ahrs.getAngle() <= -19 && ahrs.getAngle() >= -21) {
      System.out.println("Done!");
    }
  }

  public void gearShift() { // FIXME
    if (controlPanel.getRawButtonReleased(2)) {
      shiftMode = !shiftMode;
    }

    if (shiftMode) {
      // driveTrainShift.set(DoubleSolenoid.Value.kForward);
    } else {
      // driveTrainShift.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public double getDrivePowerScale() {

    /*
     * This figures out what to set the drive train speed to.
     * 
     * No trigger = 0.75
     * 
     * One trigger = 0.85
     * 
     * Both triggers = 1.0
     */

    double scale = 0.75;

    if (leftStick.getTrigger() || rightStick.getTrigger()) {
      scale = 0.85;
    }

    if (leftStick.getTrigger() && rightStick.getTrigger()) {
      scale = 1;
    }

    return scale;
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

    if (speed == 0) {
      arm.set(ControlMode.PercentOutput, 0);
    } else {
      for (int i = 0; i <= (speed * 1000); i++) {
        if (isNegative) {
          arm.set(ControlMode.PercentOutput, (i / 1000) * -1);
        } else {
          arm.set(ControlMode.PercentOutput, i / 1000);
        }
        Timer.delay(.0001);
      }
    }
  }

  public void setClimber(boolean front, boolean back) {
    frontClimb.set(front);
    backClimb.set(back);
  }

  public void setDriveMotors(double left, double right) {
    leftTop.set(left);
    leftBottom.set(left);
    rightTop.set(right);
    rightBottom.set(right);
  }

  public void setForebar(double speed) {
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

  public void setElevator(double speed) {
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

  public boolean isPOVright(Joystick joystick){
    return joystick.getPOV() == 270;
  }

  public boolean isPOVleft(Joystick joystick){
    return joystick.getPOV() == 90;
  }

  public void elevatorHeights() {
    // This is for the elevator up button
    if (isPOVright(controlPanel)) {
      System.out.println("up button");
      setElevator(.75);
    }
    // This is for the elevator down button
    else if (isPOVleft(controlPanel)) {
      System.out.println("down button");
      setElevator(-1);
    }
    // This is the lvl 1 button
    else if (isPOVup(controlPanel)) {
      setElevator(1);
      if (elevator.getEncoder().getPosition() == 30.5) {
        setElevator(0);
      }
    }
    // This is the lvl 2 button
    else if (isPOVdown(controlPanel)) {
      setElevator(1);
      if (elevator.getEncoder().getPosition() == 75.5) {
        setElevator(0);
      }
    } else {
      setElevator(0);
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
