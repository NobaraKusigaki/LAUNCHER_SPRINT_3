package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Locomotion.DriveSubsystem;
import frc.robot.subsystems.Score.AngularManager;
import frc.robot.subsystems.Score.AngularSubsystem;
import frc.robot.subsystems.Score.BoostManager;
import frc.robot.subsystems.Score.BoostSubsystem;
import frc.robot.subsystems.Score.CollectManager;
import frc.robot.subsystems.Score.CollectSubsystem;
import frc.robot.subsystems.Score.InputManager;
import frc.robot.subsystems.Score.InputSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.LimeLight;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GoToPositionCommand;


public class RobotContainer {

  private final PS5Controller systemController = new PS5Controller(Constants.PS5Controller.joystickID);
  private final Joystick driveController = new Joystick(Constants.driveController.joystickID);

  private final CollectSubsystem collectSubsystem = new CollectSubsystem();
  private final BoostSubsystem boostSubsystem = new BoostSubsystem();
  
  private final AngularManager angularManager = new AngularManager();
  private final BoostManager boostManager = new BoostManager(boostSubsystem, collectSubsystem);
  private final CollectManager collectManager = new CollectManager(collectSubsystem);
  private final InputManager inputManager = new InputManager();

  //// VITOR ===============
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ThroughBore rightEncoder = new ThroughBore();

    private final LimelightResults llfront = new LimelightResults();
    private final LimelightResults llback = new LimelightResults();

    
    // private final GoToPositionCommand goToPositionCommand = new GoToPositionCommand(
    //     inputManager.getInputSubsystem(),
    //     angularManager.getEncoder(),
    //     systemController,
    //     angularManager.getAngularSubsystem(),
    //     llfront,
    //     boostManager.getBoostSubsystem()
    // );

   
    public Command getTeleopCommand() {
        return defaultDriveCommand.alongWith(goToPositionCommand);
    }    

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem,driveController ));
        
      configureBindings();
  }

  private void configureBindings() {

    //pos minima 
    new Trigger(() -> systemController.getSquareButton())
    .onTrue(new InstantCommand(() -> angularManager.calibrateMinPos()));

    new Trigger(() -> systemController.getTriangleButton())
    .onTrue(new InstantCommand(() -> angularManager.calibrateMaxPos()));

    //########## MANUAL DO ANGULAR ##########
    new Trigger(() -> systemController.getR2Axis() > 0.04)
    .onTrue(new InstantCommand(() -> angularManager.setManual()))
    .whileTrue(new InstantCommand(() -> angularManager.setManualPower(0.5)))
    .onFalse(new InstantCommand(() -> angularManager.stopManual()));

    new Trigger(() -> systemController.getL2Axis() > 0.04)
    .onTrue(new InstantCommand(() -> angularManager.setManual()))
    .whileTrue(new InstantCommand(() -> angularManager.setManualPower(-0.5)))
    .onFalse(new InstantCommand(() -> angularManager.stopManual()));


    //########## AUTOMÁTICO ANGULAR ##########
    new Trigger(() -> systemController.getL1Button())
    .onTrue(new InstantCommand(() -> angularManager.goToMin()));

    new Trigger(() -> systemController.getR1Button())
    .onTrue(new InstantCommand(() -> angularManager.goToMax()));

  }

  //Fazer os outros botoes do sistema assim que pegar todos os valores necessarios para liberar
  //os que estao sendo usados na calibração e no manual 

  ///// VITOR ==============
  /// 
  /// 
    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public LimeLight getLimelight() {
        return limelight;
    }

}