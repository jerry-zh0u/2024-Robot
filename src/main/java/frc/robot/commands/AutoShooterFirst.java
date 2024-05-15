package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShooterFirst extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private Timer time; 
    
    public AutoShooterFirst(IntakeSubsystem intake){
        m_IntakeSubsystem = intake;
        time = new Timer();
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize(){
       time.reset();
       time.start();
    }
    
    @Override
    public void execute(){

        m_IntakeSubsystem.shooter_Reverse();

        if (time.get() > 0.5){
            m_IntakeSubsystem.shooter_On();

        }
    
        
        if(time.get() > 2.5){
            // m_IntakeSubsystem.shooter_M
            m_IntakeSubsystem.helper_On();
            m_IntakeSubsystem.intake_On();
        }
        if(time.get() > 2.8){
            m_IntakeSubsystem.shooter_Off();
            m_IntakeSubsystem.helper_Off();
            m_IntakeSubsystem.intake_Off();
        }
        if(time.get() >2.9){
            time.stop();
        }
    }

    

    @Override
    public boolean isFinished(){
        return time.get() >= 2.9;
    }
}
