package frc.robot.subsystems.Score;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectSubsystem extends SubsystemBase {

  private final SparkMax collectMotor = new SparkMax(Constants.COLLECT_MOTOR_ID, SparkMax.MotorType.kBrushed);
  private final SparkMax retractMotor = new SparkMax(Constants.RETRACT_MOTOR_ID, SparkMax.MotorType.kBrushless);

  private final DigitalInput gamePieceSensor = new DigitalInput(Constants.GAME_PIECE_SENSOR_ID);

  public CollectSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);

    collectMotor.configure(config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    collectMotor.set(power);
  }

  public void stop() {
    collectMotor.stopMotor();
  }

  public void setRetractPower(double power) {
    retractMotor.set(power);
  }

  public void stopRetract() {
    retractMotor.stopMotor();
  }
  
  public boolean gamePieceDetected() {
    return !gamePieceSensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Sensor on?", gamePieceDetected());
  }
}
