package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandMax extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    
    public IntakeCommandMax(IntakeSubsystem intake){
        m_IntakeSubsystem = intake;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.intake_Max();
    }
}
