package frc.robot.subsystems.Locomotion;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final VictorSPX leftFront = motorFactory(Constants.Drivetrain.frenteLeft, Constants.Drivetrain.leftInvertido);
    private final VictorSPX leftBack = motorFactory(Constants.Drivetrain.trasLeft, Constants.Drivetrain.leftInvertido);
    private final VictorSPX rightFront = motorFactory(Constants.Drivetrain.frenteRight, Constants.Drivetrain.rightInvertido);
    private final VictorSPX rightBack = motorFactory(Constants.Drivetrain.trasRight, Constants.Drivetrain.rightInvertido);

    public DriveSubsystem() {}

    private VictorSPX motorFactory(int id, boolean inverted) {
        VictorSPX motor = new VictorSPX(id);
        motor.configFactoryDefault();
        motor.setInverted(inverted);
        motor.configNeutralDeadband(Constants.Drivetrain.deadzone);
        motor.setNeutralMode(NeutralMode.Brake);
        return motor;
    }

    public void drive(double leftSpeed, double rightSpeed) {
        leftFront.set(ControlMode.PercentOutput, leftSpeed);
        leftBack.set(ControlMode.PercentOutput, leftSpeed);
        rightFront.set(ControlMode.PercentOutput, rightSpeed);
        rightBack.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void stop() {
        drive(0, 0);
    }

    public double getLeftMotorOutput() {
        return leftFront.getMotorOutputPercent();
    }

    public double getRightMotorOutput() {
        return rightFront.getMotorOutputPercent();
    }
}