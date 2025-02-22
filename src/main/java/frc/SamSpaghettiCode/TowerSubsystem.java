// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.SamSpaghettiCode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TowerSubsystem extends SubsystemBase {
  /** Creates a new tower. */
  SparkMax motor = new SparkMax(26, SparkMax.MotorType.kBrushless);
  SparkMax motor2 = new SparkMax(24, SparkMax.MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  RelativeEncoder encoder2 = motor2.getEncoder();
  PIDController leftPID = new PIDController(0.15, 0.005, 0.005);
  private double height;

  public TowerSubsystem() {

  }

  //IMPORTANT: the elevator's range is from 0 to -20, with -20 being the top
  @Override
  public void periodic() {
    leftPID.setSetpoint(height);
    double speed = MathUtil.clamp(leftPID.calculate(encoder.getPosition()), -Constants.MaxTowerSpeed, Constants.MaxTowerSpeed);
    motor.set(speed);
    motor2.set(-speed);
    // System.out.println("Encoder: " + encoder.getPosition() + "; Height: " + height);
  }

  public void setHeight(double height) {
    this.height = height;
  }

  public void setDifferenceHeight(double d) {
    height += d;
  }

  public void setProportionalHeight(double d) {
      setHeight(-(d*10+10));
  }

  public void zeroEncoders() {
    encoder.setPosition(0);
    encoder2.setPosition(0);
  }

}
