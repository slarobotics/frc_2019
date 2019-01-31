/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.*;

public class test extends TimedRobot {
  // Lol what is this
  private CANSparkMax Leftsmall;
  private CANSparkMax Rightsmaller;

  @Override
  public void robotInit() {
    // hopefully the motors
    Leftsmall = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    Rightsmaller = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

  }

  @Override
  public void teleopPeriodic() {

    // runMotors(1, 1);
  }

  public void runMotors(double left, double right) {
    Leftsmall.set(left);
    Rightsmaller.set(left);

  }
}
