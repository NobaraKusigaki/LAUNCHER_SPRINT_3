package frc.robot.subsystems.Score;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AngularManager extends SubsystemBase {

    public enum ControlState {
        DISABLED, 
        MANUAL, 
        AUTOMATIC
    }

    private final AngularSubsystem angular;
    private final PIDController pid;

    private ControlState state = ControlState.DISABLED;
    private double targetAngle = 0.0;
    private boolean autoActive = false;

    public AngularManager() {
        angular = new AngularSubsystem();

        pid = new PIDController(Constants.ANG_KP, Constants.ANG_KI, Constants.ANG_KD);
        pid.setTolerance(Constants.ANG_TOLERANCE_DEG);

        angular.setEncoderOffset(
            Preferences.getDouble(Constants.ANG_PREF_OFFSET, 0.0));

        targetAngle =
            Preferences.getDouble(Constants.ANG_PREF_TARGET, 0.0);
    }

    public void setManual() {
        state = ControlState.MANUAL;
        autoActive = false;
    }

    public void setManualPower(double power) {
        if (state == ControlState.MANUAL)
            angular.setPower(power);
    }

    public void stopManual() {
        angular.stop();
        state = ControlState.DISABLED;
    }

    public void goToMin() {
        targetAngle =
            Preferences.getDouble(Constants.ANG_PREF_MIN_ANGLE, 0.0);
        state = ControlState.AUTOMATIC;
        autoActive = true;
    }

    public void goToMax() {
        targetAngle =
            Preferences.getDouble(Constants.ANG_PREF_MAX_ANGLE, 90.0);
        state = ControlState.AUTOMATIC;
        autoActive = true;
    }

    
    public void calibrateMinPos() {
        double angle = angular.getAngleDeg();
        Preferences.setDouble(Constants.ANG_PREF_MIN_ANGLE, angle);

        SmartDashboard.putString("Angular/Calib", "Min salvo");
    }

    public void calibrateMaxPos() {
        double angle = angular.getAngleDeg();
        Preferences.setDouble(Constants.ANG_PREF_MAX_ANGLE, angle);

        SmartDashboard.putString("Angular/Calib", "Max salvo");
    }

    @Override
    public void periodic() {

        angular.periodic();

        switch (state) {

            case AUTOMATIC:
                runAutomatic();
                break;

            case MANUAL:
                break;

            case DISABLED:
            default:
                angular.stop();
                break;
        }
    }

    private double lastOutput = 0.0;

    private void runAutomatic() {
        if (!autoActive) {
            state = ControlState.DISABLED;
            return;
        }

        double angle = angular.getAngleDeg();
        double error = targetAngle - angle;
        double pidOut = pid.calculate(angle, targetAngle);

        pidOut = Math.max(
            Math.min(pidOut, Constants.ANG_MAX_OUTPUT),
            -Constants.ANG_MAX_OUTPUT);

        double rampRate = 0.03;
        double output = lastOutput + Math.max(
            Math.min(pidOut - lastOutput, rampRate),
            -rampRate);
        lastOutput = output;

        angular.setPower(output);

        SmartDashboard.putNumber("Angular/Target", targetAngle);
        SmartDashboard.putNumber("Angular/Error", error);
        SmartDashboard.putNumber("Angular/Output", output);

        if (pid.atSetpoint()) {
            angular.stop();
            autoActive = false;
            state = ControlState.DISABLED;
            SmartDashboard.putString("Angular/Status", "Chegou ao alvo");
        }
    }
}
