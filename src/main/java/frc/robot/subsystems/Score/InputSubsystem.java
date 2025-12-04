
package frc.robot.subsystems.Score;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InputSubsystem extends SubsystemBase {
  SparkMax inputMotor = new SparkMax(Constants.INPUT_MOTOR_ID, SparkMax.MotorType.kBrushed);

  public InputSubsystem() {
    SparkMaxConfig inpuConfig = new SparkMaxConfig();
    inpuConfig.idleMode(IdleMode.kBrake);
    inpuConfig.inverted(false);


    inputMotor.configure(inpuConfig, 
    ResetMode.kResetSafeParameters, 
    PersistMode.kPersistParameters);
  }

  public void setSubsystemPower(double power) {
    inputMotor.set(power);
  }

  public void stopMotor() {
    inputMotor.stopMotor();
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Input Motor Power", inputMotor.get());
    
  }
}
