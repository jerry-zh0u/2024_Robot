package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax _intakeMotor, _shooterMotor;
    private TalonFX _retractMotor1, _retractMotor2; 
    private DigitalInput topLimitSwitch, bottomLimitSwitch;
    private SlewRateLimiter filterTurn;
    private double init_Angle1, init_Angle2;
    // private final double dist1 = 0.098145 - (-3.974854);
    // private 
    // private final double dist2 = 51.221191 - (-180.053233);
    

    public IntakeSubsystem(){
        _intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        _shooterMotor = new CANSparkMax(IntakeConstants.shooterMotorID, MotorType.kBrushless);
        _retractMotor1 = new TalonFX(16); //FIXME
        _retractMotor2 = new TalonFX(17);

        // _retractMotor1.getSelectedSenos
        // init_Angle1 = _retractMotor1.getPosition().getValueAsDouble();
        // init_Angle2 = _retractMotor2.getPosition().getValueAsDouble();
        // _retractMotor1.getPOsi
        // _retractMotor1.getPosition().getValueAsDouble();

        // _retractMotor1.configFactoryDefault();
        // _retractMotor1.
        
        // _retractMotor1.follow

        //topLimitSwitch = new DigitalInput(8);
        //bottomLimitSwitch = new DigitalInput(9);
//removed limit switch switch to falcon limit

        // filterTurn = new SlewRateLimiter(2);

        // _retractMotor1.configFactoryDefault();
        
        _intakeMotor.restoreFactoryDefaults();
        _shooterMotor.restoreFactoryDefaults();
    }
    public void intake_On(){
        _intakeMotor.set(0.5);
    }
    public void intake_Off(){
        _intakeMotor.set(0);
    }
    public void intake_Inverse(){
        _intakeMotor.set(-0.2);
    }
    public void intake_Max(){
        _intakeMotor.set(1);
    }
    public void shooter_On(){
        _shooterMotor.set(-1);
    }
    public void shooter_Off(){
        _shooterMotor.set(0);
    }
    public void shooter_Reverse(){
        _shooterMotor.set(0.5);
    }
    public void shooter_Down(){
        // System.err.println("This is called");
        // double speed = 0;
        // double cur_Angle1 = _retractMotor1.getPosition().getValueAsDouble();
        // double cur_Angle2 = _retractMotor2.getPosition().getValueAsDouble();
        // System.err.println()
        // System.err.println(cur_Angle1 - init_Angle1 + " " + (cur_Angle2 - init_Angle2));
        _retractMotor1.set(1); _retractMotor2.set(1);
        // // if(topLimitSwitch.get()){
        //     speed = 0.5;
        // }
        // else{
        //     speed = 0.5;            
        // }
        // _retractMotor1.set(ControlMode.PercentOutput, filterTurn.calculate(speed));
        // _retractMotor1.getPosition().getValueAsDouble();

    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoder1", _retractMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("encoder2", _retractMotor2.getPosition().getValueAsDouble());
        double cur_Angle1 = _retractMotor1.getPosition().getValueAsDouble();
        double cur_Angle2 = _retractMotor2.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Enocder 1 angle dist", cur_Angle1 - init_Angle1);
        SmartDashboard.putNumber("Enocder 2 angle dist", cur_Angle2 - init_Angle2);
    }
    public void zero(){
        init_Angle1 = _retractMotor1.getPosition().getValueAsDouble();
        init_Angle2 = _retractMotor2.getPosition().getValueAsDouble();
    }
    public void shooter_Up(){
        System.err.println("Please work");
        // double speed = 0.0;
        // if(bottomLimitSwitch.get()){
        //     speed = -0.5;
        // }
        // else{
        //     speed = -0.5;
        // }
        // System.err.println()
        // System.err.println(cur_Angle1 - init_Angle1 + " " + (cur_Angle2 - init_Angle2));
        // if(cur_Angle1 - init_Angle  1 <= dist1){
        _retractMotor1.set(-1); _retractMotor2.set(-1);
        // }
        // else{
            // _retractMotor1.set(0); _retractMotor2.set(0);
        // }
        // SmartDashboard.putNumber("encoder", _retractMotor1.get)
        // _retractMotor1.
        
    }
    public void shooterAngleOff(){
        _retractMotor1.set(0);
        _retractMotor2.set(0);
    }
}
