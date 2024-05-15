package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class IntakeCommandOn extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final SolenoidSubsystem m_SolenoidSubsystem;
    
    public IntakeCommandOn(IntakeSubsystem intake, SolenoidSubsystem solenoid){
        m_IntakeSubsystem = intake;
        m_SolenoidSubsystem = solenoid;
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_SolenoidSubsystem);
    }

    @Override
    public void execute(){
        // System.err.println("")
        m_IntakeSubsystem.intake_On();
        m_SolenoidSubsystem.openSolenoid();
    }
}
