package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;
import frc.robot.subsystems.Score.BoostSubsystem;
import frc.robot.subsystems.Score.CollectManager;
import frc.robot.subsystems.Sensors.limelightSubsystem;
import frc.robot.subsystems.Sensors.ThroughBoreSubsystem;
import frc.robot.subsystems.Locomotion.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GoToPositionCommand extends Command {

    private final DriveSubsystem drive;
    private final BoostSubsystem boost;
    private final CollectManager collectManager;
    private final limelightSubsystem lime;
    private final ThroughBoreSubsystem throughBore;

    private enum State { ALIGN, SHOOT, WAIT_END }
    private State state = State.ALIGN;

    private static final double kP_align = Constants.LimeLight.kP_align;
    private static final double maxTurnPower = Constants.LimeLight.maxTurnPower;
    private static final double alignDeadband = Constants.LimeLight.alignDeadband;

    private final Timer shootTimer = new Timer();

    public GoToPositionCommand(
        DriveSubsystem drive,
        BoostSubsystem boost,
        CollectManager collectManager,
        limelightSubsystem lime,
        ThroughBoreSubsystem throughBore
    ) {
        this.drive = drive;
        this.boost = boost;
        this.collectManager = collectManager;
        this.lime = lime;
        this.throughBore = throughBore;

        addRequirements(drive, boost);
    }

    @Override
    public void initialize() {
        state = State.ALIGN;
        shootTimer.stop();
        shootTimer.reset();
        SmartDashboard.putString("GoTo/STATE", "ALIGN");
    }

    @Override
    public void execute() {

        boolean hasPiece = collectManager.isPieceInside();
        boolean hasTarget = lime.frontHasTarget();

        SmartDashboard.putBoolean("GoTo/Piece", hasPiece);
        SmartDashboard.putBoolean("GoTo/Target", hasTarget);

        if (!hasPiece || !hasTarget) {
            boost.stopMotors();
            drive.drive(0, 0);
            state = State.ALIGN;
            SmartDashboard.putString("GoTo/STATE", "WAITING (NO PIECE/TAG)");
            return;
        }

        switch (state) {

            case ALIGN:
                double tx = lime.getFrontTX();
                SmartDashboard.putNumber("GoTo/TX", tx);

                if (Math.abs(tx) <= alignDeadband) {
                    drive.drive(0, 0);
                    state = State.SHOOT;
                    SmartDashboard.putString("GoTo/STATE", "SHOOT");
                    return;
                }

                double turnCmd = MathUtil.clamp(tx * kP_align, -maxTurnPower, maxTurnPower);
                drive.drive(turnCmd, -turnCmd);
            break;

            case SHOOT:

                double dist = calcDistance(lime.getFrontTY());
                double targetRPM = calcShootRPM(dist);
                double motorRPM = throughBore.getRPM();
                double outputRPM = motorRPM / 100.0; 

                double errorRPM = targetRPM - motorRPM;
                double outRPM = MathUtil.clamp(errorRPM * 0.0004, -1, 1);

                boost.setpower(-outRPM);

                SmartDashboard.putNumber("GoTo/RPM Target", targetRPM);
                SmartDashboard.putNumber("GoTo/RPM Current", motorRPM);
                SmartDashboard.putNumber("GoTo/RPM Output", outputRPM);

                if (Math.abs(errorRPM) <= 80) {
                    shootTimer.restart();
                    state = State.WAIT_END;
                    SmartDashboard.putString("GoTo/STATE", "WAIT_END");
                }
            break;

            case WAIT_END:

                if (shootTimer.get() >= 5.0) {
                    boost.stopMotors();
                    collectManager.setPieceInside(false);
                    lime.setAligned(false);
                    drive.drive(0, 0);
                }
            break;
        }
    }

    private double calcDistance(double ty) {
        double h1 = Constants.LimeLight.limeLightHeight;
        double h2 = Constants.LimeLight.tagHeight;
        double a1 = Constants.LimeLight.limeLightAngle;
        return (h2 - h1) / Math.tan(Math.toRadians(a1 + ty));
    }

    private double calcShootRPM(double distance) {
        return 500 * distance + 3000;
    }

    @Override
    public boolean isFinished() {
        return (state == State.WAIT_END && shootTimer.get() >= 5.0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0);
        boost.stopMotors();
        shootTimer.stop();
    }
}