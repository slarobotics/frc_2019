package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.vision.VisionThread;

import com.revrobotics.*;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot implements PIDOutput {

  // Joysticks
  /*
   * private Joystick leftStick; private Joystick rightStick; private Joystick
   * controlPanel;
   */

  // Motors
  private CANSparkMax leftTop;
  private CANSparkMax rightTop;
  private CANSparkMax leftMiddle;
  private CANSparkMax rightMiddle;
  private CANSparkMax leftBottom;
  private CANSparkMax rightBottom;
  private CANSparkMax elevator;

  // Limit Switches
  private DigitalInput limitSwitchOne;
  private DigitalInput limitSwitchTwo;
  private DigitalInput limitSwitchThree;

  // Gyro
  private AHRS ahrs;
  private PIDController turnController;
  private double rotateToAngleRate;

  // Camera
  /*
   * private UsbCamera camera =
   * CameraServer.getInstance().startAutomaticCapture("intake",
   * "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");
   */

  private UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  private CvSource outputStream;
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
    /*
     * leftStick = new Joystick(0); rightStick = new Joystick(1); controlPanel = new
     * Joystick(2);
     */

    // Inits the Motors
    leftTop = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightTop = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMiddle = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMiddle = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftBottom = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBottom = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevator = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    // Inits limit switch
    limitSwitchOne = new DigitalInput(1);
    limitSwitchTwo = new DigitalInput(2);
    limitSwitchThree = new DigitalInput(3);

    // Inits Vision Pipeline
    outputStream = CameraServer.getInstance().putVideo("overlay", 320, 240);
    camera.setResolution(320, 240);

    gripPipeline = new VisionThread(camera, new GripPipeline(), pipeline -> {
      outputStream.putFrame(pipeline.overlayOutput);
      if (!pipeline.convexHullsOutput().isEmpty()) {
        System.out.println("Convex Hull was not empty");
        Rect r = Imgproc.boundingRect(pipeline.convexHullsOutput().get(0));
        pipeline.findTarget(pipeline.convexHullsOutput());
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
          System.out.println("Center X: " + centerX);
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
    /*
     * runRobot(leftStick.getY(), rightStick.getY());
     * 
     * if (controlPanel.getRawButton(3)) { System.out.print("Button 3 is clicked.");
     * leftBottom.set(1); } else { leftBottom.set(0); }
     */
  }

  public void runRobot(double left, double right) {
    leftTop.set(left);
    leftMiddle.set(left);
    leftBottom.set(left);
    rightTop.set(right);
    rightMiddle.set(right);
    rightBottom.set(right);
  }

  public void limitSwitches() {
    // level one
    if (limitSwitchOne.get() && limitSwitchTwo.get() == false && limitSwitchThree.get() == false) {
      System.out.println("level one");
    }
    // level two
    else if (limitSwitchOne.get() && limitSwitchTwo.get() && limitSwitchThree.get() == false) {
      System.out.println("level two");
    }
    // level three
    else if (limitSwitchOne.get() && limitSwitchTwo.get() && limitSwitchThree.get()) {
      System.out.println("level three");
    } else {
      System.out.println("error");
    }
    /*
     * controlPanel = new Joystick(0); // This is for the elevator up button if
     * (controlPanel.getRawButton(2)) { System.out.println("up button");
     * elevator.set(1); } // This is for the elevator down button else if
     * (controlPanel.getRawButton(3)) { System.out.println("down button");
     * elevator.set(-1); } else { elevator.set(0); }
     */
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
