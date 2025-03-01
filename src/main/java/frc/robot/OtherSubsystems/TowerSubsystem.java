// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OtherSubsystems;

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
  PIDController pid = new PIDController(0.1, 0.001, 0.001);
  
  
  private double height = 20;
  private boolean automatic = false;

  public TowerSubsystem() {
    pid.setTolerance(1);
    pid.reset();
  }

  // IMPORTANT: the elevator's range is from 0 to 20, with 0 being the top
  @Override
  public void periodic() {
    pid.setIntegratorRange(-0.001, 0.001);

      pid.setSetpoint(height);
      double speed = MathUtil.clamp(pid.calculate(encoder.getPosition()), -Constants.MaxTowerSpeed,
          Constants.MaxTowerSpeed);
      runMotors(speed);
      // System.out.println("Encoder: " + encoder.getPosition() + "; Height: " +
      // height + "Speed: " + speed + "Error: " + pid.getAccumulatedError());
  }

  public void setHeight(double d) {
    height = MathUtil.clamp(d, -0.5, 19.7);
  }

  public void setDifferenceHeight(double d) {
    setHeight(height + d);
  }

  public void runMotors(double speed) {
      // System.out.println("Enocders: " + encoder.getPosition() + ", Speed: " + speed);
      motor.set(speed);
      motor2.set(-speed);
  }

  public void stopMotors() {
    motor.set(0);
    motor2.set(0);
  }

  public void percentageHeight(double d) {
    setHeight(20 - (d * 20));
  }

  public void zeroEncoders() {
    encoder.setPosition(0);
    encoder2.setPosition(0);
  }

  public void setAutomatic(boolean enabled) {
    if (this.automatic != enabled) {
      // Mode was changed
      if (enabled) {
        // Sync hieght with current encoder.
        // height = encoder.getPosition();
      }
    }
    this.automatic = enabled;
  }

public boolean atSetpoint() {
    return pid.atSetpoint();
}

}
