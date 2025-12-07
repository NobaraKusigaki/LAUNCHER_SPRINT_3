package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Locomotion.DriveSubsystem;
import frc.robot.subsystems.Score.BoostManager;
import frc.robot.subsystems.Score.BoostSubsystem;
import frc.robot.subsystems.Score.CollectManager;
import frc.robot.subsystems.Score.CollectSubsystem;
import frc.robot.subsystems.Score.InputManager;
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

        // RETRACT
        new Trigger(() -> systemController.getL1Button())
            .whileTrue(
                new RunCommand(
                    () -> collectManager.retractOut(0.3),
                    collectManager
                )
            )
            .onFalse(new InstantCommand(() -> collectManager.stopRetract()));

        new Trigger(() -> systemController.getR1Button())
            .whileTrue(
                new RunCommand(
                    () -> collectManager.retractIn(-0.3),
                    collectManager
                )
            )
            .onFalse(new InstantCommand(() -> collectManager.stopRetract()));

        // COLLECT
        new Trigger(() -> systemController.getR3Button())
            .whileTrue(
                new RunCommand(
                    () -> collectManager.pieceIn(0.5),
                    collectManager
                )
            )
            .onFalse(new InstantCommand(() -> collectManager.stopCollect()));

        new Trigger(() -> systemController.getL3Button())
            .whileTrue(
                new RunCommand(
                    () -> collectManager.pieceOut(-0.5),
                    collectManager
                )
            )
            .onFalse(new InstantCommand(() -> collectManager.stopCollect()));


        // INPUT
        new Trigger(() -> systemController.getCrossButton())
            .whileTrue(
                new RunCommand(
                    () -> inputManager.setPower(0.3),
                    inputManager
                )
            )
            .onFalse(new InstantCommand(() -> inputManager.stopManual()));

        new Trigger(() -> systemController.getTriangleButton())
            .whileTrue(
                new RunCommand(
                    () -> inputManager.setPower(-0.3),
                    inputManager
                )
            )
            .onFalse(new InstantCommand(() -> inputManager.stopManual()));

        // BOOST
        new Trigger(() -> systemController.getCircleButton())
            .whileTrue(
                new RunCommand(
                    () -> boostSubsystem.setpower(0.7),
                    boostSubsystem
                )
            )
            .onFalse(new InstantCommand(() -> boostSubsystem.stopMotors()));

        new Trigger(() -> systemController.getSquareButton())
            .whileTrue(
                new RunCommand(
                    () -> boostSubsystem.setpower(-0.7),
                    boostSubsystem
                )
            )
            .onFalse(new InstantCommand(() -> boostSubsystem.stopMotors()));
            
        new Trigger(() -> systemController.getL2Axis() > 0.05)
            .whileTrue(
                new RunCommand(
                    () -> boostSubsystem.setpower(systemController.getL2Axis() * 1.0),
                    boostSubsystem
                )
            )
            .onFalse(new InstantCommand(() -> boostSubsystem.stopMotors()));

        new Trigger(() -> systemController.getR2Axis() > 0.05)
            .whileTrue(
                new RunCommand(
                    () -> boostSubsystem.setpower(-systemController.getR2Axis() * 1.0),
                    boostSubsystem
                )
            )
            .onFalse(new InstantCommand(() -> boostSubsystem.stopMotors()));
    }


    private void configureAutoGoTo() {

        Trigger autoShootTrigger = new Trigger(
            () -> collectManager.isPieceInside() && limelight.frontHasTarget()
        );
    
        autoShootTrigger.whileTrue(
            new GoToPositionCommand(
                driveSubsystem,
                boostSubsystem,
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
