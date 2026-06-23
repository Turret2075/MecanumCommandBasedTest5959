// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mecanismos;
import frc.robot.Constants.Mecanum;

public class Intake extends SubsystemBase {
  private final SparkMax IntakeFijo = new SparkMax(Mecanismos.kIntakePort, MotorType.kBrushless);
  private final SparkMaxConfig IntakeFijoConfig = new SparkMaxConfig();

  public Intake() {
    IntakeFijoConfig.inverted(false).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(30).voltageCompensation(Mecanum.kMaxVolts);
    
    IntakeFijo.configure(
      IntakeFijoConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
  }

  public void controlWithSpeed(double speed){
    IntakeFijo.set(speed);
  }

  public void stopRollers(){
    IntakeFijo.set(0.0);
  }

  public void intakeWithValue(){
    IntakeFijo.set(1.0);
  }

  public void reverseWithValue(){
    IntakeFijo.set(-0.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
