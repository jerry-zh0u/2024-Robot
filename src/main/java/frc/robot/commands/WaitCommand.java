package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitCommand extends Command {
    private double timeToWait;
    private double startTime;
    
    public WaitCommand(double waitTime){
       timeToWait = waitTime;
    }

    @Override
    public void initialize(){
        System.out.println("Help");
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished(){
        System.out.println("--------");
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }
}
