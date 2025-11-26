package frc.robot.subsystems.Score;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BoostManager extends SubsystemBase {

  public enum BoostState {
    OFF,
    RUNNING,
    WAITING_TO_STOP
  }

  private final BoostSubsystem boost;
  private final CollectSubsystem collect;

  private BoostState state = BoostState.OFF;
  private final Timer timer = new Timer();

 
  private int stableCounter = 0;
  private boolean stableDetected = false;
  private double currentPower = 0.0;
 

  public BoostManager(BoostSubsystem boost, CollectSubsystem collect) {
    this.boost = boost;
    this.collect = collect;
  }

  private double rampRate(double target) {
    if (currentPower < target) {
      currentPower = Math.min(currentPower + Constants.RAMP_RATE, target);
    } else if (currentPower > target) {
      currentPower = Math.max(currentPower - Constants.RAMP_RATE, target);
    }
    boost.setpower(currentPower);
    return currentPower;
  }

  private void updateDebouncedSensor() {
    boolean raw = collect.gamePieceDetected();

    if (raw == stableDetected) {
      stableCounter = 0;
    } else {
      stableCounter++;
      if (stableCounter >= Constants.DEBOUNCE_COUNT) {
        stableDetected = raw;
        stableCounter = 0;
      }
    }
  }

  @Override
  public void periodic() {
    updateDebouncedSensor();
    boolean detected = stableDetected;

    switch (state) {

      case OFF:
        rampRate(0.0);

        if (detected) {
          state = BoostState.RUNNING;
          timer.stop();
        }
        break;

      case RUNNING:
        rampRate(Constants.BOOST_POWER);

        if (!detected) {
          timer.reset();
          timer.start();
          state = BoostState.WAITING_TO_STOP;
        }
        break;

      case WAITING_TO_STOP:
        if (detected) {
          timer.stop();
          state = BoostState.RUNNING;
          break;
        }

        if (timer.hasElapsed(3.0)) {
          state = BoostState.OFF;
        }

        rampRate(0.0);
        break;
    }

    SmartDashboard.putString("Boost State", state.name());
    SmartDashboard.putBoolean("Boost Sensor Stable", stableDetected);
    SmartDashboard.putNumber("Boost Current Power", currentPower);
    SmartDashboard.putNumber("Boost Timer", timer.get());
  }
}
