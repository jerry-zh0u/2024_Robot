package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class AutoDrive extends Command {

  private CommandSwerveDrivetrain m_drive;

  private double drivePower;
  private SwerveRequest.RobotCentric drive;
  private Timer time;
  private double driveTime;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public AutoDrive(CommandSwerveDrivetrain in, SwerveRequest.RobotCentric drive_, double drivePower_, double dtime) {
    m_drive = in;
    drivePower = drivePower_;
    time = new Timer();
    driveTime = dtime;
    addRequirements(m_drive);
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
    if(time.get() < driveTime){
        m_drive.applyRequest(() -> drive.withVelocityX(drivePower));
    }
    else{
        m_drive.applyRequest(() -> drive.withVelocityX(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){

    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() > driveTime; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}