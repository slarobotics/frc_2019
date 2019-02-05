/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import com.revrobotics.*;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot implements PIDOutput {

  // Joysticks
  private Joystick leftStick;
  private Joystick rightStick;
  private Joystick controlPanel;
  private Joystick elevStick;

  // Motors
  private CANSparkMax leftTop;
  private CANSparkMax rightTop;
  private CANSparkMax leftMiddle;
  private CANSparkMax rightMiddle;
  private CANSparkMax leftBottom;
  private CANSparkMax rightBottom;
  private CANSparkMax elevator;

  // Gyro
  private AHRS ahrs;
  private PIDController turnController;
  private double rotateToAngleRate;

  // Camera
  private UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("intake",
      "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");
  private CvSource intakeOutputStream;
  private VisionThread gripPipeline;
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

  @Override
  public void robotInit() {
    // Inits the Joysticks
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    controlPanel = new Joystick(2);
    elevStick = new Joystick(3);

    // Inits the Motors
    leftTop = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightTop = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMiddle = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMiddle = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftBottom = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBottom = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevator = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    // Inits Vision Pipeline
    gripPipeline = new VisionThread(camera, new GripPipeline(), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
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
  }

  @Override
  public void teleopPeriodic() {
    runRobot(leftStick.getY(), rightStick.getY());

    if (controlPanel.getRawButton(3)) {
      System.out.print("Button 3 is clicked.");
      leftBottom.set(1);
    } else {
      leftBottom.set(0);
    }
  }

  public void runRobot(double left, double right) {
    leftTop.set(left);
    leftMiddle.set(left);
    leftBottom.set(left);
    rightTop.set(right);
    rightMiddle.set(right);
    rightBottom.set(right);
  }

  public void elevatorHeights() {
    // This is for the elevator up button
    if (controlPanel.getRawButton(2)) {
      System.out.println("up button");
      elevator.set(1);
      System.out.println(elevator.getEncoder().getVelocity());
    }
    // This is for the elevator down button
    else if (controlPanel.getRawButton(3)) {
      System.out.println("down button");
      elevator.set(-1);
    } else {
      elevator.set(0);
    }
  }

  public static double calculateTotalHeight(double arm_angle) { // Returns inches
    // This function assumes that we are perfectly mounted at 90 degrees at the
    // start going up and down at that point

    // Constants in inches
    int arm_width = 1; // Width of arm mechanism
    int arm_length = 13; // length of arm mechanism
    int elevator_height = 15; // height of elevator mechanism
    int elevator_width = 1; // width of elevator mechanism

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