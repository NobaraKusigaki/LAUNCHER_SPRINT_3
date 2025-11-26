package frc.robot;

public final class Constants {

// Motor IDs
public static final int COLLECT_MOTOR_ID = 5;
public static final int INPUT_MOTOR_ID = 6;
public static final int BOOST_DOWN_MOTOR_ID = 7;
public static final int BOOST_UP_MOTOR_ID = 8;

// Sensor IDs
public static final int GAME_PIECE_SENSOR_ID = 0;
public static final double INPUT_MAX_OUTPUT = 0.5;

public static final double BOOST_POWER = 0.5;
public static final double RAMP_RATE = 0.05;
public static final int DEBOUNCE_COUNT = 4;

public static final class LimeLight {
  public static final String limelightFront = "limelight-front";
  public static final String limelightBack = "limelight-back";

    public static final double kP_Distance = 0.04;
    public static final double kP_Aim = 0.02;
    public static final double targetArea = 4.0;
    public static final double maxSpeed = 0.25;
    public static final double minSpeed = 0.05;
    public static final double deadbandDistance = 0.3;
    public static final double deadbandAim = 10.0;
    public static final double minTurnCommand = 0.02;
    public static final double initialSpeed = 0.3;
}
}
