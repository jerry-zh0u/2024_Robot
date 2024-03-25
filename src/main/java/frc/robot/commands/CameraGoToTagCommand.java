package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class CameraGoToTagCommand extends Command {
    private final PhotonCamera m_camera;
    private final CommandSwerveDrivetrain drivetrain; 
    private final SwerveRequest.RobotCentric drive;
    private final PIDController centeringPID;
    private final double joystickInput;

    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    
    public CameraGoToTagCommand(CommandSwerveDrivetrain drivetrain_, double joystickInput_){
        drivetrain = drivetrain_;
        drive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;
        m_camera = new PhotonCamera("April");
        joystickInput = joystickInput_;
        centeringPID = new PIDController(10, 0, 0);
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        // drivetrain.applyRequest(() -> drive.withVelocityX(2).withVelocityY(2)/* .withRotationalRate(rotationSpeed)*/);
        drivetrain.applyRequest(() -> drive.withVelocityX(2));
        // System.err.println("Camera is called");

        // var result = m_camera.getLatestResult();
        // List<PhotonTrackedTarget> targets = result.getTargets();
        // for(PhotonTrackedTarget target : targets){
        //     int targetID = target.getFiducialId();
        //     System.err.println("targeID" + " " + targetID);
        //     if(targetID == 7){
        //         // double targetYaw = result.getBestTarget().getYaw();
        //         double targetYaw = target.getYaw();
        //         double centeringAdjustment = centeringPID.calculate(targetYaw);
        //         double forwardSpeed = TunerConstants.kSpeedAt12VoltsMps * (-joystickInput);
        //         double strafeSpeed = centeringAdjustment;
        //         double rotationSpeed = 0.0;
        //         // System.err.println(targetYaw + " " + centeringAdjustment + " " + forwardSpeed);
        //         SmartDashboard.putNumber("target xYaw ", targetYaw); 
        //         SmartDashboard.putNumber("centering", centeringAdjustment);
        //         SmartDashboard.putNumber("fowardSpeed", forwardSpeed);
        //         drivetrain.applyRequest(() -> drive.withVelocityX(forwardSpeed).withVelocityY(strafeSpeed)/* .withRotationalRate(rotationSpeed)*/);
        //         // System.err.println(drive.VelocityX +"  " + drive.VelocityY);
        //         SmartDashboard.putNumber("velox ", drive.VelocityX); 
        //         SmartDashboard.putNumber("veloy", drive.VelocityY);
        //     }
        //     // else {
        //     //     drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0)/*.withRotationalRate(0)*/);
        //     // }
        // }
    }
}
