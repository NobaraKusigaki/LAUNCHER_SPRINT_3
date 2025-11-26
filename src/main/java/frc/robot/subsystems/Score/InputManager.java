package frc.robot.subsystems.Score;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InputManager extends SubsystemBase {

  public enum InputState {
    DISABLED,
    MANUAL
  }

  private final InputSubsystem input;
  private InputState currentState = InputState.DISABLED;

  public InputManager() {
    input = new InputSubsystem();
  }

  public void setManual() {
    currentState = InputState.MANUAL;
  }

  public void setManualPower(double power) {

    if (currentState != InputState.MANUAL) {
      currentState = InputState.MANUAL;
    }
    power = Math.max(
        Math.min(power, Constants.INPUT_MAX_OUTPUT),
        -Constants.INPUT_MAX_OUTPUT);
    input.setPower(power);
  }

  public void stopManual() {
    input.stopMotor();
    currentState = InputState.DISABLED;
  }

  @Override
  public void periodic() {
    input.periodic();
    switch (currentState) {
      case MANUAL:
        break;

      case DISABLED:
      default:
        input.stopMotor();
        break;
    }
  }
}
