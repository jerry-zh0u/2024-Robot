package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class AutoIntakeCommandOn extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final SolenoidSubsystem m_SolenoidSubsystem;
    private final WaitCommand m_test;
    
    public AutoIntakeCommandOn(IntakeSubsystem intake, SolenoidSubsystem solenoid){
        m_IntakeSubsystem = intake;
        m_SolenoidSubsystem = solenoid;
        m_test = new WaitCommand(1);
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_SolenoidSubsystem);
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.intake_On();
        m_SolenoidSubsystem.openSolenoid();
    }

    @Override 
    public boolean isFinished(){
        return m_test.isFinished();
    }
}
