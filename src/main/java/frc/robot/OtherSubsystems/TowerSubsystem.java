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
  SparkMax coralMotor = new SparkMax(32, SparkMax.MotorType.kBrushless);

  private final double algaeSpeed = 0.2;
  private final double coralRotateSpeed = 0.2;
  private final double coralSpeed = 0.1;
  
  
  PIDController pid = new PIDController(0.1, 0.001, 0.001);
  PIDController coralPid = new PIDController(0.1, 0, 0);
  final double pidTolerance = 0.1;
  final double coralPidTolerance = 1;
  
  
  private double height = 0;
  private double speed;

  private double r = 0;

  public TowerSubsystem() {
    coralEncoder.setPosition(0);
    pid.setTolerance(pidTolerance);
    pid.reset();
    coralPid.setTolerance(coralPidTolerance);
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
    coralPid.setIntegratorRange(-0.01, 0.01);

      pid.setSetpoint(height);
      
      
      // double errorMax = 0.1;
      // if (Math.abs(encoder.getPosition() - height) > errorMax) {
        speed = MathUtil.clamp(-0.0373 + 2 * Math.pow(height - encoder.getPosition(), 2) * Math.signum(height - encoder.getPosition()), -0.25, 0.25);
      
        // } else {
      //   speed = MathUtil.clamp(pid.calculate(encoder.getPosition()), -0.25, 0.25);
      // }

      // double flatline = 0;
      // speed = flatline;
      // if (encoder.getPosition() - height > 0 && encoder.getPosition() - height < 0.5) { //Up
      //   speed += 0;
      // } else if (encoder.getPosition() - height > 0 && encoder.getPosition() - height < 5) {
      //   speed += -0.1;
      // } else if (encoder.getPosition() - height > 0 && encoder.getPosition() - height > 5) {
      //   speed += -0.2;
      // }
      // if (encoder.getPosition() - height < 0 && encoder.getPosition() - height < 0.5) { //Up
      //   speed += 0;
      // } else if (encoder.getPosition() - height < 0 && encoder.getPosition() - height < 5) {
      //   speed += 0.1;
      // } else if (encoder.getPosition() - height < 0 && encoder.getPosition() - height > 5) {
      //   speed += 0.2;
      // }
      // if (encoder.getPosition() - height > 0 && encoder.getPosition() - height > 0.4) {
      //   speed *= (encoder.getPosition() - height + 0.8);
      // }
      // if (encoder.getPosition() - height < 0 && encoder.getPosition() - height < -0.4) {
      //   speed *= (encoder.getPosition() - height  - 0.8);
      // }
      runElevatorMotors(speed);

      if (pid.atSetpoint()) {
        pid.reset();
      }

      coralPid.setSetpoint(r);
      double rotationSpeed = -MathUtil.clamp(coralPid.calculate(coralEncoder.getPosition()),
       -coralRotateSpeed,
       coralRotateSpeed);
      runCoralRotateMotors(rotationSpeed);
      // System.out.println(encoder.getPosition());
      System.out.println("Encoder: " + encoder.getPosition() + "; Height: " +
      height + " REncoder: " + coralEncoder.getPosition() + " Rotation: " + r);
      System.out.println("Error: " + pid.getError());
      System.out.println("Speed: " + speed);

      System.out.println("Coral Encoder: " + coralEncoder.getPosition() +
      "; Rotation: " + r + "; Speed: " + rotationSpeed + "; Error: " + coralPid.getAccumulatedError());
      oldHeight = height;
  }

  public void setHeight(double d) {
    height = MathUtil.clamp(d, -12.0, 0);
  }

  public void setDifferenceHeight(double d) {
    setHeight(height + MathUtil.clamp(d, -0.1, 0.1));
  }
  
  public void setRotation(double d) {
    r = MathUtil.clamp(d, 0, 100);
  }
  
  public void setDifferenceRotation(double d) {
    System.out.println("########### " + d);
    setRotation(r + d);
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
