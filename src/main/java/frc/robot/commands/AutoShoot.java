package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class AutoShoot extends Command {

  private IntakeSubsystem m_intake;

  private Timer time;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public AutoShoot(IntakeSubsystem intake) {
    m_intake = intake;
    time = new Timer();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() < 1){
      m_intake.shooter_Off();
      m_intake.intake_Off();
    }
    else if(1 <= time.get() && time.get() < 2.5){
      m_intake.shooter_On();
    }
    else{
      m_intake.intake_On();
    }
    if(time.get() >= 3.5){
      m_intake.shooter_Off();
      m_intake.intake_Off();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){

    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() > 3.5; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}