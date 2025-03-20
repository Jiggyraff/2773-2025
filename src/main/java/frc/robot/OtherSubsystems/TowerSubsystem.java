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
  RelativeEncoder coralEncoder = coralRotateMotor.getEncoder();
  SparkMax coralMotor = new SparkMax(31, SparkMax.MotorType.kBrushless);

  private final double algaeSpeed = 0.2;
  private final double coralRotateSpeed = 0.2;
  private final double coralSpeed = 0.1;
  
  
  PIDController pid = new PIDController(0.2, 0.001, 0.001);
  PIDController coralPid = new PIDController(0.1, 0, 0);
  
  
  private double height = 0;
  private double speed;
  private boolean automatic = false;
  private double rotation = 0;
  private double rotationSpeed;

  public TowerSubsystem() {
    pid.setTolerance(1);
    pid.reset();
    coralPid.setTolerance(1);
    coralPid.reset();
    Shuffleboard.getTab("Tower").addDouble("Encoder", () -> {return encoder.getPosition();});
    Shuffleboard.getTab("Tower").addDouble("Height", () -> {return height;});
    Shuffleboard.getTab("Tower").addDouble("Speed", () -> {return speed;});
    Shuffleboard.getTab("Tower").addDouble("Integral Error", () -> {return pid.getAccumulatedError();});
  }

  // IMPORTANT: the elevator's range is from 0 to 20, with 0 being the top
  double oldHeight = 0;
  @Override
  public void periodic() {
    pid.setIntegratorRange(-0.001, 0.001);
    coralPid.setIntegratorRange(-0.001, 0.001);

      pid.setSetpoint(height);
      if (oldHeight - height > -1) {
      speed = MathUtil.clamp(pid.calculate(encoder.getPosition()), -0.1,
          0.1);
      } else {
        speed = MathUtil.clamp(pid.calculate(encoder.getPosition()), -0.01,
          0.01);
      }
      runElevatorMotors(speed);

      coralPid.setSetpoint(rotation);
      rotationSpeed = MathUtil.clamp(coralPid.calculate(coralEncoder.getPosition()),
       -coralRotateSpeed,
       coralRotateSpeed);
      runCoralRotateMotors(-rotationSpeed);
      // System.out.println(encoder.getPosition());
      System.out.println("Encoder: " + encoder.getPosition() + "; Height: " +
      height + " REncoder: " + coralEncoder.getPosition() + " Rotation: " + rotation);

      // System.out.println("Coral Encoder: " + coralEncoder.getPosition() +
      // "; Rotation: " + rotation + "; Speed: " + rotationSpeed + "; Error: " + coralPid.getAccumulatedError());
      oldHeight = height;
  }

  public void setHeight(double d) {
    height = MathUtil.clamp(d, -19, 0);
  }

  public void setDifferenceHeight(double d) {
    setHeight(height + MathUtil.clamp(d, -0.05, 0.05));
  }
  
  public void setRotation(double d) {
    System.out.println("########### "+d);
    rotation = MathUtil.clamp(d, 45.26, -30.95);
  }

  public void setDifferenceRotation(double d) {
    setRotation(rotation + d);
  }
  
  public void runElevatorMotors(double speed) {
    // System.out.println("Enocders: " + encoder.getPosition() + ", Speed: " + speed);
    elevatorMotor.set(speed);
    elevatorMotor2.set(-speed);
  }

  public void runAlgaeMotors(double speed) {
    speed = MathUtil.clamp(speed, -algaeSpeed, algaeSpeed);
    algaeMotor.set(speed);
    algaeMotor2.set(-speed);
  }

  public void runCoralRotateMotors(double speed) {
    speed = MathUtil.clamp(speed, -coralRotateSpeed, coralRotateSpeed);
    coralRotateMotor.set(-speed);
  }


  public void runCoralMotors(double speed) {
    speed = MathUtil.clamp(speed, -coralSpeed, coralSpeed);
    coralMotor.set(speed);
  }

  
  public void zeroElevatorEncoders() {
    encoder.setPosition(0);
    encoder2.setPosition(0);
  }
  
  public boolean elevatorAtSetpoint() {
    return pid.atSetpoint();
  }
  
  public void percentageHeight(double d) {
    setHeight(-19 + (d * 19));
  }
}
