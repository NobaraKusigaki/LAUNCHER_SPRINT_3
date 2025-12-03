package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Locomotion.DriveSubsystem;
import frc.robot.subsystems.Score.BoostManager;
import frc.robot.subsystems.Score.BoostSubsystem;
import frc.robot.subsystems.Score.CollectManager;
import frc.robot.subsystems.Score.CollectSubsystem;
import frc.robot.subsystems.Score.InputManager;
import frc.robot.subsystems.Score.InputSubsystem;
import frc.robot.subsystems.Sensors.EncoderSubsystem;
import frc.robot.subsystems.Sensors.ThroughBoreSubsystem;
import frc.robot.subsystems.Sensors.limelightSubsystem;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GoToPositionCommand;

public class RobotContainer {

    private final PS5Controller systemController = new PS5Controller(1);
    private final Joystick driveController = new Joystick(0);

    private final CollectSubsystem collectSubsystem = new CollectSubsystem();
    private final BoostSubsystem boostSubsystem = new BoostSubsystem();

    private final BoostManager boostManager =
        new BoostManager(boostSubsystem, collectSubsystem);
    private final CollectManager collectManager =
        new CollectManager(collectSubsystem);
    private final InputManager inputManager = new InputManager();

    private final EncoderSubsystem encoderSubsystem =
        new EncoderSubsystem(Constants.Encoder.encoderID);
    private final limelightSubsystem limelight =
        new limelightSubsystem();
    private final ThroughBoreSubsystem throughBoreSubsystem =
        new ThroughBoreSubsystem();

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(
            new DefaultDriveCommand(driveSubsystem, driveController)
        );

        configureBindings();
        configureAutoGoTo();
    }

    private void configureBindings() {

        new Trigger(() -> systemController.getCrossButton())
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> boostSubsystem.setpower(0.5)),
                new InstantCommand(() -> inputManager.setManualPower(0.5))
            )
        )
        .onFalse(new InstantCommand(() -> {
            boostSubsystem.stopMotors();
            inputManager.stopManual();
        }));    

        new Trigger(() -> systemController.getL2Axis() > 0.04)
            .whileTrue(new InstantCommand(() -> collectManager.pieceIn(0.3)))
            .onFalse(new InstantCommand(() -> collectManager.stopCollect()));

        new Trigger(() -> systemController.getR2Axis() > 0.04)
            .whileTrue(new InstantCommand(() -> collectManager.pieceOut(-0.3)))
            .onFalse(new InstantCommand(() -> collectManager.stopCollect()));

        new Trigger(() -> systemController.getTriangleButton())
            .whileTrue(new InstantCommand(() -> collectManager.retractIn(0.3)))
            .onFalse(new InstantCommand(() -> collectManager.stopRetract()));

        new Trigger(() -> systemController.getCircleButton())
            .whileTrue(new InstantCommand(() -> collectManager.retractOut(0.3)))
            .onFalse(new InstantCommand(() -> collectManager.stopRetract()));

       
    }

    private void configureAutoGoTo() {

        Trigger autoShootTrigger = new Trigger(
            () -> collectManager.isPieceInside() && limelight.frontHasTarget()
        );
    
        autoShootTrigger.whileTrue(
            new GoToPositionCommand(
                driveSubsystem,
                boostSubsystem,
                collectSubsystem,
                collectManager,
                limelight,
                throughBoreSubsystem
            )
        );
    }    

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }
}
