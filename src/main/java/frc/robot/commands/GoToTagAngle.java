// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystem.Drivetrain;
// import frc.robot.subsystem.LimeLight;

// public class GoToAprilTagCommand extends Command {

//     private final Drivetrain drivetrain;
//     private final LimeLight limelight;

//     private static final double kP_Distance = 0.04;
//     private static final double kP_Aim = 0.02;

//     private static final double targetArea = 4.0;

//     private static final double maxSpeed = 0.25;

//     private static final double deadbandDistance = 0.3;
//     private static final double deadbandAim = 10.0;

//     private static final double minTurnCommand = 0.02;

//     public GoToAprilTagCommand(Drivetrain drivetrain, LimeLight limelight) {
//         this.drivetrain = drivetrain;
//         this.limelight = limelight;
//         addRequirements(drivetrain, limelight);
//     }

//     @Override
//     public void execute() {
    
//         if (!limelight.hasTarget()) {
//             drivetrain.drive(0.3, -0.3);
//             return;
//         }
    
//         double tx = limelight.getTx();
//         double ta = limelight.getTa();
    
//         double turn = 0;
//         double forward = 0;
    
//         if (Math.abs(tx) > deadbandAim) {
//             turn = kP_Aim * tx;
    
//             if (Math.abs(tx) > 2.0) {
//                 turn += (tx > 0) ? minTurnCommand : -minTurnCommand;
//             }
    
//             turn = clamp(turn, -maxSpeed, maxSpeed);
//         }
    
//         if (Math.abs(tx) <= deadbandAim) {
    
//             double errorDistance = targetArea - ta;
    
//             if (Math.abs(errorDistance) > deadbandDistance) {
    
//                 forward = kP_Distance * errorDistance;
    
//                 forward = clamp(forward, 0.05, maxSpeed);
//             }
//         }
    
//         double leftSpeed = forward + turn;
//         double rightSpeed = forward - turn;
    
//         drivetrain.drive(leftSpeed, rightSpeed);
//     }
    
//     @Override
//     public boolean isFinished() {
//         return limelight.hasTarget() && limelight.getTa() >= targetArea;
//     }    

//     private double clamp(double value, double min, double max) {
//         if (value > max) return max;
//         if (value < min) return min;
//         return value;
//     }
// }