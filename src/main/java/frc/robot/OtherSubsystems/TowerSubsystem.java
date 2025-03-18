// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OtherSubsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TowerSubsystem extends SubsystemBase {
  /** Creates a new tower. */
  SparkMax elevatorMotor = new SparkMax(26, SparkMax.MotorType.kBrushless);
  SparkMax elevatorMotor2 = new SparkMax(24, SparkMax.MotorType.kBrushless);
  RelativeEncoder encoder = elevatorMotor.getEncoder();
  RelativeEncoder encoder2 = elevatorMotor2.getEncoder();
  SparkMax algaeMotor = new SparkMax(13, SparkMax.MotorType.kBrushless);
  SparkMax algaeMotor2 = new SparkMax(15, SparkMax.MotorType.kBrushless);
  SparkMax coralRotateMotor = new SparkMax(30, SparkMax.MotorType.kBrushless);
  SparkMax coralMotor = new SparkMax(31, SparkMax.MotorType.kBrushless);

  private final double algaeSpeed = 0.2;
  private final double coralRotateSpeed = 0.2;
  private final double coralSpeed = 0.1;
  
  
  PIDController pid = new PIDController(0.2, 0.001, 0.001);
  
  
  private double height = 0;
  private double speed;
  private boolean automatic = false;

  public TowerSubsystem() {
    pid.setTolerance(1);
    pid.reset();
    Shuffleboard.getTab("Tower").addDouble("Encoder", () -> {return encoder.getPosition();});
    Shuffleboard.getTab("Tower").addDouble("Height", () -> {return height;});
    Shuffleboard.getTab("Tower").addDouble("Speed", () -> {return speed;});
    Shuffleboard.getTab("Tower").addDouble("Integral Error", () -> {return pid.getAccumulatedError();});
  }

  // IMPORTANT: the elevator's range is from 0 to 20, with 0 being the top
  @Override
  public void periodic() {
    pid.setIntegratorRange(-0.001, 0.001);

      pid.setSetpoint(height);
      speed = MathUtil.clamp(pid.calculate(encoder.getPosition()), -Constants.MaxTowerSpeed,
          Constants.MaxTowerSpeed);
      runElevatorMotors(speed);
      // System.out.println(encoder.getPosition());
      System.out.println("Encoder: " + encoder.getPosition() + "; Height: " +
      height + "Rotation: " + coralRotateMotor.getEncoder().getPosition());
  }

  public void setHeight(double d) {
    height = MathUtil.clamp(d, -19, 0);
  }

  public void setDifferenceHeight(double d) {
    setHeight(height + d);
  }

  public void runElevatorMotors(double speed) {
      // System.out.println("Enocders: " + encoder.getPosition() + ", Speed: " + speed);
      elevatorMotor.set(speed);
      elevatorMotor2.set(-speed);
  }

  public void stopElevatorMotors() {
    elevatorMotor.set(0);
    elevatorMotor2.set(0);
  }

  public void percentageHeight(double d) {
    setHeight(-19 + (d * 19));
  }

  public void zeroElevatorEncoders() {
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

  public void setAlgaeMotors(double d) {
    d = MathUtil.clamp(d, -algaeSpeed, algaeSpeed);
    algaeMotor.set(d);
    algaeMotor2.set(-d);
  }

  public void setCoralRotateMotors(double d) {
    d = MathUtil.clamp(d, -coralRotateSpeed, coralRotateSpeed);
    coralRotateMotor.set(-d);
  }

  public void setCoralMotors(double d) {
    d = MathUtil.clamp(d, -coralSpeed, coralSpeed);
    coralMotor.set(d);
  }

public boolean elevatorAtSetpoint() {
    return pid.atSetpoint();
}

}
