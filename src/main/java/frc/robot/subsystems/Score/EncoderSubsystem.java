package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderSubsystem extends SubsystemBase {

    private final DutyCycleEncoder encoder;

    public Encoder(int port) {
        encoder = new DutyCycleEncoder(port);
    }

    public double getPosition() {
        return encoder.get();
    }
}