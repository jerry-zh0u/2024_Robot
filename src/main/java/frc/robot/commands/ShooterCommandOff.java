package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterCommandOff extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    
    public ShooterCommandOff(IntakeSubsystem intake){
        m_IntakeSubsystem = intake;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.shooter_Off();
    }
}
