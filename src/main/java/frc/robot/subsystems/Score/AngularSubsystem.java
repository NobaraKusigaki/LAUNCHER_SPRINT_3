package frc.robot.subsystems.Score;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AngularSubsystem extends SubsystemBase {

    private final SparkMax motor = new SparkMax(Constants.ANGULAR_MOTOR_ID, MotorType.kBrushed);

    public AngularSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        motor.configure(config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void stop() {
        motor.set(0);
    }
}
