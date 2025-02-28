// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OtherSubsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  // SparkMax motor = new SparkMax(13, Constants.motorType);
  // RelativeEncoder encoder = motor.getEncoder();
  double speed = 0.0;


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

  }

  @Override
  public void periodic() {
    // motor.set(speed);
  }

  public void setSpeed(double s) {
    speed = s;
  }
}
