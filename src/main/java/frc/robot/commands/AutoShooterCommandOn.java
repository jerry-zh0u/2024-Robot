package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class AutoShooterCommandOn extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
   
    private final WaitCommand m_test;
    
    public AutoShooterCommandOn(IntakeSubsystem intake){
        m_IntakeSubsystem = intake;
        m_test = new WaitCommand(1);
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute(){
        System.out.println("shooter On");
        m_IntakeSubsystem.shooter_On();
        }
    


    @Override 
    public boolean isFinished(){
        return m_test.isFinished();
    }

}
