package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderSubsystem extends SubsystemBase {

    private DutyCycleEncoder encoder;

    public void Encoder(int port) {
        encoder = new DutyCycleEncoder(port);
    }

    public double getPosition() {
        return encoder.get();
    }
}