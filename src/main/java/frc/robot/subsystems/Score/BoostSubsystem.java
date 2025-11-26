// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Score;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BoostSubsystem extends SubsystemBase {
 SparkMax downMotor = new SparkMax(Constants.BOOST_DOWN_MOTOR_ID, SparkMax.MotorType.kBrushed);
 SparkMax upMotor = new SparkMax(Constants.BOOST_UP_MOTOR_ID, SparkMax.MotorType.kBrushed);

  public BoostSubsystem() {

    SparkMaxConfig downConfig = new SparkMaxConfig();
    downConfig.idleMode(IdleMode.kBrake);
    downConfig.inverted(false);

    downMotor.configure(downConfig, 
    ResetMode.kResetSafeParameters, 
    PersistMode.kPersistParameters);

    SparkMaxConfig upConfig = new SparkMaxConfig();
    upConfig.idleMode(IdleMode.kBrake);
    upConfig.inverted(false);

    upMotor.configure(upConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
    
  }
  public double setpower(double power ){
    downMotor.set(power);
    upMotor.set(power);
    return power;
  }

  public void stopMotors(){
    downMotor.stopMotor();
    upMotor.stopMotor();
  }


  @Override
  public void periodic() {
  SmartDashboard.putNumber("Boost Down Motor Power", downMotor.get());
  SmartDashboard.putNumber("Boost Up Motor Power", upMotor.get());

  }
}
