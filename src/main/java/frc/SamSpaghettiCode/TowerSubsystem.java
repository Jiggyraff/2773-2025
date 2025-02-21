// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.SamSpaghettiCode;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TowerSubsystem extends SubsystemBase {
  /** Creates a new tower. */
  SparkMax motor = new SparkMax(26, SparkMax.MotorType.kBrushless);
  SparkMax motor2 = new SparkMax(24, SparkMax.MotorType.kBrushless);

  XboxController j = new XboxController(1);
  public TowerSubsystem() {

  }

  @Override
  public void periodic() {
    var s = j.getLeftX() * 0.2f;
    motor.set(s);
    // motor2.set(-s);
  }

}
