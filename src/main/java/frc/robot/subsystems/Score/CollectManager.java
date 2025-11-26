package frc.robot.subsystems.Score;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectManager extends SubsystemBase {

  private final CollectSubsystem collect;
  private boolean lastState = false;
  private boolean piecein = false;

  public CollectManager(CollectSubsystem collect) {
    this.collect = collect;
  }

  @Override
  public void periodic() {

    boolean currentState = collect.gamePieceDetected();
    piecein = (currentState && !lastState);
    lastState = currentState;
  }

  public boolean hasPieceJustEntered() {
    return piecein;
  }

  public boolean isPieceInside() {
    return lastState;
  }
}
