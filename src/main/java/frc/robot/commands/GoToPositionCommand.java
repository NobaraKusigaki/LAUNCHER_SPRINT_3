package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.Score.AngularSubsystem;
import frc.robot.subsystems.Score.BoostSubsystem;
import frc.robot.subsystems.Score.CollectSubsystem;
import frc.robot.subsystems.Score.InputSubsystem;
import frc.robot.subsystems.Sensors.LimelightSubsystem;
import frc.robot.subsystems.Sensors.EncoderSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class GoToPositionCommand extends Command {

    private final AngularSubsystem angular;
    private final BoostSubsystem boost;
    private final CollectSubsystem  collect;
    private final EncoderSubsystem encoder;
    private final InputSubsystem input;
    private final LimelightSubsystem lime;

    private boolean movingToTarget = false;
    private double targetPosition = Constants.Angular.posZero;

    private static final double POS_ZERO = Constants.Angular.posZero;
    private static final double POS_90 = Constants.Angular.posNoventa;

    private static final double TOL = Constants.Angular.toleranciaMin;
    private static final double MAX_OUT = Constants.Angular.maxOutput;


    public GoToPositionCommand(
        AngularSubsystem angular,
        BoostSubsystem boost,
        CollectSubsystem collect,
        EncoderSubsystem encoder,
        InputSubsystem input,
        LimelightSubsystem lime
        )
         {
        this.angular = angular;
        this.boost = boost;
        this.collect = collect;
        this.encoder = encoder;
        this.input = input;
        this.lime = lime;

        addRequirements(angular, boost, collect);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("GoToPosition/Status", "Pronto");
    }

    @Override
    public void execute() {

        double current = encoder.getPosition();
        boolean hasPiece = input.hasPiece();
        boolean hasTarget = lime.frontHasTarget();
        double ty = lime.getFrontTY();

        if (hasPiece && hasTarget) {

            targetPosition = POS_90;
            movingToTarget = true;
            double distance = calcDistance(ty);
            SmartDashboard.putNumber("Shooter/Distance", distance);
            double power = calcShootPower(distance);
            SmartDashboard.putNumber("Shooter/Power", power);
            boost.shoot(power);
            collect.hold();

            moveToTarget();
            return;
        }

        // double L2 = controle.getRawAxis(Constants.Joystick.L2);
        // double R2 = controle.getRawAxis(Constants.Joystick.R2);

        if (L2 > TOL && current > POS_ZERO) {
            angular.move(MANUAL);
            return;
        }

        if (R2 > TOL && current < POS_90) {
            angular.move(-MANUAL);
            return;
        }

        if (movingToTarget) {
            moveToTarget();
        } else {
            angular.stop();
        }
    }

    private void moveToTarget() {

        double current = encoder.getPosition();
        double error = targetPosition - current;

        if (Math.abs(error) <= TOL) {
            angular.stop();
            movingToTarget = false;

            SmartDashboard.putString("GoToPosition/Status", "Alvo atingido");
            return;
        }

        double output = (error < 0) ? MAX_OUT : -MAX_OUT;

        angular.move(output);

        SmartDashboard.putNumber("Angular/Error", error);
        SmartDashboard.putNumber("Angular/Output", output);
    }

    private double calcDistance(double ty) {
        double h1 = Constants.LimeLight.limeLightHeight;
        double h2 = Constants.LimeLight.tagHeight;
        double a1 = Constants.LimeLight.limeLightAngle;

        return (h2 - h1) / Math.tan(Math.toRadians(a1 + ty));
    }

    private double calcShootPower(double distance) {
        double p = 0.05 * distance + 0.3;
        return MathUtil.clamp(p, 0.25, 1.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        angular.stop();
        boost.stop();
        collect.stop();
    }
}
