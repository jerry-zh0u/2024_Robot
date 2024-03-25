// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class ClimbSubsystem extends SubsystemBase{
//     private CANSparkMax _leftMotor, _rightMotor;

//     public ClimbSubsystem(){
//         _leftMotor = new CANSparkMax(Constants.ClimbConstants.leftMotorID, MotorType.kBrushless);
//         _rightMotor = new CANSparkMax(Constants.ClimbConstants.rightMotorID, MotorType.kBrushless);
//         _leftMotor.restoreFactoryDefaults();
//         _rightMotor.restoreFactoryDefaults();
//         _leftMotor.follow(_rightMotor);
//     }
//     public void retract(){

//     }
// }
