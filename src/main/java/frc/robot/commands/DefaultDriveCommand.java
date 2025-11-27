package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Locomotion.DriveSubsystem;
import frc.robot.utils.MathUtils;

public class DefaultDriveCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Joystick driveController;

    private double speed = Constants.driveController.speedB;
    private final MathUtils math = new MathUtils();

    public DefaultDriveCommand(DriveSubsystem driveSubsystem, Joystick driveController) {
        this.driveSubsystem = driveSubsystem;
        this.driveController = driveController;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double eixoX1 = driveController.getRawAxis(Constants.driveController.eixoX);
        double eixoY1 = -driveController.getRawAxis(Constants.driveController.eixoY);
        double eixoX2 = driveController.getRawAxis(Constants.driveController.eixoX2);
        double eixoY2 = -driveController.getRawAxis(Constants.driveController.eixoY2);

        int pov = driveController.getPOV();

        boolean botaoA = driveController.getRawButton(Constants.driveController.botaoA);
        boolean botaoB = driveController.getRawButton(Constants.driveController.botaoB);
        boolean botaoX = driveController.getRawButton(Constants.driveController.botaoX);

        if (botaoA) speed = Constants.driveController.speedA;
        else if (botaoB) speed = Constants.driveController.speedB;
        else if (botaoX) speed = Constants.driveController.speedX;

        double velEsq = 0;
        double velDir = 0;

        double L2 = driveController.getRawAxis(Constants.driveController.L2);
        double R2 = driveController.getRawAxis(Constants.driveController.R2);

        if (L2 != 0) {
            velEsq = velDir = math.calcularL2(L2, R2, speed);
        } else if (R2 != 0) {
            velEsq = velDir = -math.calcularR2(L2, R2, speed);
        } else if (pov != -1) {
            double[] povVel = math.calcularPOV(pov, speed);
            velEsq = povVel[0];
            velDir = povVel[1];
        } else {
            double[] magSeno = math.calcularMagESeno(eixoX1, eixoX2, eixoY1, eixoY2);
            double[] velocidades = math.calcularAnalogicos(magSeno, speed, eixoX1, eixoY1, -eixoX2, eixoY2);
            velEsq = velocidades[0];
            velDir = velocidades[1];
        }

        driveSubsystem.drive(velEsq, velDir);

        SmartDashboard.putNumber("Velocidade Esquerda", velEsq);
        SmartDashboard.putNumber("Velocidade Direita", velDir);
        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("POV", pov);
        SmartDashboard.putNumber("L2", L2);
        SmartDashboard.putNumber("R2", R2);
        SmartDashboard.putBoolean("Botao A", botaoA);
        SmartDashboard.putBoolean("Botao B", botaoB);
        SmartDashboard.putBoolean("Botao X", botaoX);
        SmartDashboard.putNumber("Eixo X", eixoX1);
        SmartDashboard.putNumber("Eixo Y", eixoY1);
        SmartDashboard.putNumber("Eixo X2", eixoX2);
        SmartDashboard.putNumber("Eixo Y2", eixoY2);
        SmartDashboard.putNumber("Velocidade Esquerda", velEsq);
        SmartDashboard.putNumber("Velocidade Direita", velDir);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}