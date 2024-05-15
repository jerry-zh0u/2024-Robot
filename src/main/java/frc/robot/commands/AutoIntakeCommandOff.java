package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class AutoIntakeCommandOff extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final SolenoidSubsystem m_SolenoidSubsystem;   
    private final WaitCommand m_test;                                                                                                                                                                                                 
    
    public AutoIntakeCommandOff(IntakeSubsystem intake, SolenoidSubsystem solenoid){
        m_IntakeSubsystem = intake;
        m_test = new WaitCommand(1);
        m_SolenoidSubsystem = solenoid;
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_SolenoidSubsystem);  
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.intake_Off();
        m_SolenoidSubsystem.closeSolenoid();
    }

    @Override
    public boolean isFinished(){
        return m_test.isFinished();
    }
}
