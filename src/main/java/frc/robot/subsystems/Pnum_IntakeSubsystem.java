package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Pnum_IntakeSubsystem extends SubsystemBase{
    private CANSparkMax _intakeMotor, _shooterMotor;
    private TalonSRX _retractMotor;
    private DigitalInput topLimitSwitch, bottomLimitSwitch;
    private SlewRateLimiter filterTurn;
    private DoubleSolenoid m_solonoid;
    

    public Pnum_IntakeSubsystem(){
        m_solonoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);//FIXME IDs
    }
    public void on(){
        m_solonoid.set(DoubleSolenoid.Value.kForward);
    }
    public void off(){
        m_solonoid.set(DoubleSolenoid.Value.kReverse);
    }
}
