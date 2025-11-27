package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class ThroughBoreSubsystem extends SubsystemBase {
    private Encoder rightEncoder;

    double wheelDiameterMeters = Constants.Encoder.wheelDiameterMeters;

    public ThroughBoreSubsystem() {
        rightEncoder = new Encoder(Constants.Encoder.portaEncoderA, Constants.Encoder.portaEncoderB);

        double countsPerRev = Constants.Encoder.countsPerRev;
        double gearRatio = Constants.Encoder.gearRatio;
        double wheelCircumference = Math.PI * wheelDiameterMeters;

        double distancePerPulse = wheelCircumference / (countsPerRev * gearRatio);

        rightEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setReverseDirection(Constants.Encoder.ReverseDirection);
    }

    public double getDistanceMeters() {
        return rightEncoder.getDistance();
    }

    public double getSpeedMetersPerSecond() {
        return rightEncoder.getRate();
    }

    public void rate() {
        rightEncoder.getRate();
    }

    public void reset() {
        rightEncoder.reset();
    }

    public double getRPM() {
        double metersPerSecond = rightEncoder.getRate();
        double wheelCircumference = Math.PI * wheelDiameterMeters;
        double revPerSecond = metersPerSecond / wheelCircumference;
        return revPerSecond * 60.0;
    }
    
}