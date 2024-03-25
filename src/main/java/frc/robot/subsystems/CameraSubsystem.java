package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase{
    private PhotonCamera camera;

    public CameraSubsystem(){
        camera = new PhotonCamera("photonvision");;
    }
    public boolean hasTargets(){
        return camera.getLatestResult().hasTargets();
    }
}
