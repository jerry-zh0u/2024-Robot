package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class AutoIntakeCommand extends Command{
    private final IntakeSubsystem m_IntakeSubsystem;
    private final SolenoidSubsystem m_SolenoidSubsystem;
    private final Timer time;
    private double m_intaketime;

    public AutoIntakeCommand(IntakeSubsystem intake, SolenoidSubsystem solenoid){
        m_IntakeSubsystem = intake;
        m_SolenoidSubsystem = solenoid;
        time = new Timer();
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_SolenoidSubsystem);
        m_intaketime = Timer.getFPGATimestamp();
    }
    
    public void initialize(){
        time.reset();
        time.start();
        
    }

    @Override
    public void execute(){
        // time.reset();
        // time.start();
        
  
        
        if((time.get()) < 2){
            m_SolenoidSubsystem.openSolenoid();
            m_IntakeSubsystem.intake_On();

        }
        else{
            m_IntakeSubsystem.intake_Off();
            m_SolenoidSubsystem.closeSolenoid();
            time.stop();
        
        }
        
            
    }

    @Override
    public boolean isFinished(){
    
        return time.get() >= 2.6;
        
    }
}
