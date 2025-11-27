package frc.robot.subsystems.Sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class limelightSubsystem {

    private final NetworkTable llFront;
    private final NetworkTable llBack;

    public void LimeLightSubsystem() {
        llFront = NetworkTableInstance.getDefault().getTable(Constants.limelightFront);
        llBack = NetworkTableInstance.getDefault().getTable(Constants.limelightBack);
    }

    public double getFrontTX() { return llFront.getEntry("tx").getDouble(0); }
    public double getFrontTY() { return llFront.getEntry("ty").getDouble(0); }
    public double getFrontTA() { return llFront.getEntry("ta").getDouble(0); }
    public int getFrontTargetId() { return (int) llFront.getEntry("tid").getDouble(-1); }
    public boolean frontHasTarget() { return llFront.getEntry("tv").getDouble(0) == 1; }

    public double getBackTX() { return llBack.getEntry("tx").getDouble(0); }
    public double getBackTY() { return llBack.getEntry("ty").getDouble(0); }
    public double getBackTA() { return llBack.getEntry("ta").getDouble(0); }
    public int getBackTargetId() { return (int) llBack.getEntry("tid").getDouble(-1); }
    public boolean backHasTarget() { return llBack.getEntry("tv").getDouble(0) == 1; }

}