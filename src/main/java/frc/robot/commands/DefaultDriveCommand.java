package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.MathUtils;

public class DefaultDriveCommand extends Command {

    private final Drivetrain drivetrain;
    private final Joystick controle1;

    private double speed = Constants.Joystick.speedB;
    private final MathUtils math = new MathUtils();

    public DefaultDriveCommand(Drivetrain drivetrain, Joystick lJoystick) {
        this.drivetrain = drivetrain;
        this.ljoystick = lJoystick;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double eixoX1 = controle1.getRawAxis(Constants.LControl.eixoX);
        double eixoY1 = -controle1.getRawAxis(Constants.LControl.eixoY);
        double eixoX2 = controle1.getRawAxis(Constants.LControl.eixoX2);
        double eixoY2 = -controle1.getRawAxis(Constants.LControl.eixoY2);

        double L2 = controle1.getRawAxis(Constants.LControl.L2);
        double R2 = controle1.getRawAxis(Constants.LControl.R2);
        int pov = controle1.getPOV();

        boolean botaoA = controle1.getRawButton(Constants.LControl.botaoA);
        boolean botaoB = controle1.getRawButton(Constants.LControl.botaoB);
        boolean botaoX = controle1.getRawButton(Constants.LControl.botaoX);

        if (botaoA) speed = Constants.LControl.speedA;
        else if (botaoB) speed = Constants.LControl.speedB;
        else if (botaoX) speed = Constants.LControl.speedX;

        double velEsq = 0;
        double velDir = 0;

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

        drivetrain.drive(velEsq, velDir);

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
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}