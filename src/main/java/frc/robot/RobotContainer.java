package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Score.AngularManager;
import frc.robot.subsystems.Score.AngularSubsystem;
import frc.robot.subsystems.Score.BoostManager;
import frc.robot.subsystems.Score.BoostSubsystem;
import frc.robot.subsystems.Score.CollectManager;
import frc.robot.subsystems.Score.CollectSubsystem;
import frc.robot.subsystems.Score.InputManager;
import frc.robot.subsystems.Score.InputSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.GoToAprilTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

  private final PS5Controller systemController = new PS5Controller(Constants.SYSTEM_CONTROLLER_ID);

  private final CollectSubsystem collectSubsystem = new CollectSubsystem();
  private final BoostSubsystem boostSubsystem = new BoostSubsystem();
  
  private final AngularManager angularManager = new AngularManager();
  private final BoostManager boostManager = new BoostManager(boostSubsystem, collectSubsystem);
  private final CollectManager collectManager = new CollectManager(collectSubsystem);
  private final InputManager inputManager = new InputManager();

  //// VITOR ===============
  private final DriveSubsystem driveSubsystem = new Drivetrain();
    private final ThroughBore rightEncoder = new ThroughBore();

    private final LimeLight llfront = new LimeLight();
    private final LimeLight llback = new LimeLight();

    private final Joystick ljoystick = new Joystick(Constants.Joystick.lJoystick);
    private final Joystick sjoystick = new Joystick(Constants.Joystick.sJoystick);

    private final DefaultDriveCommand defaultDriveCommand = new DefaultDriveCommand(driveSubsystem, ljoystick);
    private final SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
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
    public Drivetrain getDriveSubsystem() {
        return driveSubsystem;
    }

    public LimeLight getLimelight() {
        return limelight;
    }

    public Command getTeleopCommand() {
        return null;
    }
}