package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShooterHelperOn extends Command {
    private IntakeSubsystem m_IntakeSubsystem;
    private WaitCommand m_test;
    
    public void AutoShooterHelperOn(IntakeSubsystem intake){
        m_IntakeSubsystem = intake;
        addRequirements(m_IntakeSubsystem);
        m_test = new WaitCommand(0.5);
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.helper_On();
        m_IntakeSubsystem.intake_On();
    }

    @Override
    public boolean isFinished(){
        return m_test.isFinished();
    }
}
