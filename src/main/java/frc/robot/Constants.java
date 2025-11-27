package frc.robot;

import edu.wpi.first.wpilibj.DutyCycle;

public final class Constants {

// Motor IDs
public static final int COLLECT_MOTOR_ID = 5;
public static final int RETRACT_MOTOR_ID = 4;
public static final int INPUT_MOTOR_ID = 6;
public static final int BOOST_DOWN_MOTOR_ID = 7;
public static final int BOOST_UP_MOTOR_ID = 8;
public static final int ANGULAR_MOTOR_ID = 9;

// Sensor IDs
public static final int GAME_PIECE_SENSOR_ID = 0; // sensor red
public static final int ANGULAR_ENCODER_ID = 0;
public static final double INPUT_MAX_OUTPUT = 0;

public static final double BOOST_POWER = 0.5;
public static final double ANG_MAX_OUTPUT = 0.3;

public static final double RAMP_RATE = 0.05;
public static final int DEBOUNCE_COUNT = 4;
public static final double ANG_TOLERANCE_DEG = 1;

// Angular PID constants
public static final double ANG_KP = 0.015;
public static final double ANG_KI = 0.0;
public static final double ANG_KD = 0.0008;

// Angular Preferences Keys
public static final String ANG_PREF_OFFSET = "AngularOffsetDeg";
public static final String ANG_PREF_TARGET = "AngularTargetAngle";
public static final String ANG_PREF_MIN_ANGLE = "AngularMinAngle";
public static final String ANG_PREF_MAX_ANGLE = "AngularMaxAngle";
public static final int SYSTEM = 0;

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

        public static final double limeLightHeight = 0.60;
        public static final double tagHeight = 1.22;
        public static final double limeLightAngle = 25.0;
    }

    public static final class Drivetrain {
        public static final int frenteLeft = 1;
        public static final int trasLeft = 2;
        public static final int frenteRight = 3;
        public static final int trasRight = 4;

        public static final boolean leftInvertido = false;
        public static final boolean rightInvertido = true;

        public static final double deadzone = 0.04;
    }

    public static final class driveController {
        public static final int joystickID = 3;

        public static final int eixoX = 0;
        public static final int eixoY = 1;
        public static final int eixoX2 = 4;
        public static final int eixoY2 = 5;

        public static final int botaoA = 1; // X ou A
        public static final int botaoB = 2; // Circulo ou B 
        public static final int botaoX = 3; // Quadrado ou X
        public static final int botaoY = 4; // Triangulo ou Y 

        public static final int L1 = 5;
        public static final int R1 = 6;
        public static final int L2 = 2;
        public static final int R2 = 3;

        public static final double speedA = 0.25;
        public static final double speedB = 0.5;
        public static final double speedX = 1.0;
    }

    public static final class PS5Controller {
        public static final int joystickID = 0;

        public static final int eixoX = 0;
        public static final int eixoY = 1;
        public static final int eixoX2 = 4;
        public static final int eixoY2 = 5;

        public static final int botaoA = 1; // X ou A
        public static final int botaoB = 2; // Circulo ou B 
        public static final int botaoX = 3; // Quadrado ou X
        public static final int botaoY = 4; // Triangulo ou Y 

        public static final int L1 = 5;
        public static final int R1 = 6;
        public static final int L2 = 2;
        public static final int R2 = 3;

        public static final double speedA = 0.25;
        public static final double speedB = 0.5;
        public static final double speedX = 1.0;
    }

    public static final class Angular {
        public static boolean isMovingToTarget = false;
        public static double targetPosition = 0.0;

        public static final double posZero = 0.0;
        public static final double posNoventa = 90.0;
        private static final double toleranciamin = 0.2;
        private static final double maxoutput = 0.7;
    }

    public static final class Encoder {
        public static final int encoderId = 4;
    }
}