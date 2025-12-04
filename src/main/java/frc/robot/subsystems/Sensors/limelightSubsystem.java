package frc.robot.subsystems.Sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class limelightSubsystem {

    private final NetworkTable llFront;
    private boolean aligned = false;

    public limelightSubsystem() {
        llFront = NetworkTableInstance.getDefault().getTable(Constants.LimeLight.limelightFront);
    }

    public double getFrontTX() { return llFront.getEntry("tx").getDouble(0); }
    public double getFrontTY() { return llFront.getEntry("ty").getDouble(0); }
    public double getFrontTA() { return llFront.getEntry("ta").getDouble(0); }
    public int getFrontTargetId() { return (int) llFront.getEntry("tid").getInteger(-1); }
    public boolean frontHasTarget() { return llFront.getEntry("tv").getDouble(0) == 1; }

    public void setAligned(boolean value) {
        aligned = value;
    }

    public boolean isAligned() {
        return aligned;
    }
}
