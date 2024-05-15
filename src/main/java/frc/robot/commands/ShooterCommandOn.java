package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterCommandOn extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private Timer time; 
    
    public ShooterCommandOn(IntakeSubsystem intake){
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
        System.out.println("shooter On");
        m_IntakeSubsystem.shooter_On();
        if(time.get() > 1){
            m_IntakeSubsystem.helper_On();
            m_IntakeSubsystem.intake_On();
        }
    }


}
