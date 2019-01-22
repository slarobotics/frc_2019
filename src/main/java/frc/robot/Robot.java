/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.*;
import com.revrobotics.*;

public class Robot extends TimedRobot {

  // Joysticks
  private Joystick leftStick;
  private Joystick rightStick;
  private Joystick controlPanel;

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

  @Override
  public void robotInit() {
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    controlPanel = new Joystick(2);

    leftTop = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightTop = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMiddle = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMiddle = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftBottom = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBottom = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevator = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    CameraServer.getInstance().startAutomaticCapture();

    limitSwitchOne = new DigitalInput(1);
    limitSwitchTwo = new DigitalInput(2);
    limitSwitchThree = new DigitalInput(3);
  }

  @Override
  public void teleopPeriodic() {
    runRobot(leftStick.getY(), rightStick.getY());
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
    controlPanel = new Joystick(0);
    // This is for the elevator up button
    if (controlPanel.getRawButton(2)) {
      System.out.println("up button");
      elevator.set(1);
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
}
