package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;
    private Command m_autonomousCommand;
    private Command m_teleopCommand;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        
        try {
            m_robotContainer = new RobotContainer();
            System.out.println("RobotContainer inicializado!");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}