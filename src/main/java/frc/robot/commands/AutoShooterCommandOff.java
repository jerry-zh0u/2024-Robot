package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShooterCommandOff extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final WaitCommand m_test;
    
    public AutoShooterCommandOff(IntakeSubsystem intake){
        m_IntakeSubsystem = intake;
        addRequirements(m_IntakeSubsystem);
        m_test = new WaitCommand(0.5);
    }

    @Override
    public void execute(){
        System.out.println("Shooter Off");
        m_IntakeSubsystem.shooter_Off();
        m_IntakeSubsystem.helper_Off();
        m_IntakeSubsystem.intake_Off();
    }

    @Override
    public boolean isFinished(){
        return m_test.isFinished();
    }
}
