package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class IntakeCommandOff extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final SolenoidSubsystem m_SolenoidSubsystem;                                                                                                                                                                                                    
    
    public IntakeCommandOff(IntakeSubsystem intake, SolenoidSubsystem solenoid){
        m_IntakeSubsystem = intake;
        m_SolenoidSubsystem = solenoid;
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_SolenoidSubsystem);  
    }

    @Override
    public void execute(){
        System.out.println("--------");
        m_IntakeSubsystem.intake_Off();
        m_SolenoidSubsystem.closeSolenoid();
    }
}
