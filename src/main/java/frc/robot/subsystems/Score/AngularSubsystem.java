package frc.robot.subsystems.Score;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AngularSubsystem extends SubsystemBase {

    private final SparkMax motor = new SparkMax(Constants.ANGULAR_MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(Constants.ANGULAR_ENCODER_ID);

    private double encoderOffsetDeg = 0.0;

    public AngularSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        motor.configure(config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public double getAngleDeg() {
        double raw = absEncoder.get() * 360.0;
        double angle = raw + encoderOffsetDeg;
        return ((angle % 360.0) + 360.0) % 360.0;
    }

    public void setEncoderOffset(double offset) {
        this.encoderOffsetDeg = offset;
    }

    public double getOffsetDeg() {
        return encoderOffsetDeg;
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void stop() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angular/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("Angular/OffsetDeg", encoderOffsetDeg);
    }
}
