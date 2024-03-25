package frc.robot.subsystems;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Telemetry;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private PhotonCamera m_Camera;
    private SwerveDriveState driveState = new SwerveDriveState();
    private PIDController controller;
    private final double pGain = 0.1; private final double dGain = 0;
    public static SendableChooser<Command> autoChooser;
    // private Pose2d pose = super.SwerveDriveState.Pose;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        init();
        setupPathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        init();
        setupPathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void init(){
        m_Camera = new PhotonCamera("MyCamera"); //FIXME change the camera name
        controller = new PIDController(pGain, 0, dGain);
    }
    // @Override
    // public void periodic(){
    //     SmartDashboard.
    // }
    //FIXME Creashing on This Line
    public void setupPathPlanner(){
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::tareEverything, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    public Pose2d getPose(){
        return driveState.Pose;
    }
    public void tareEverything(Pose2d _pose){ //something along this line
        super.seedFieldRelative(_pose);
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(super.m_moduleLocations);
        return kinematics.toChassisSpeeds(driveState.ModuleStates);
    }
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
        /*
         * drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
         */
        // setDefaultCommand(() -> );
        // kinematics.
        SwerveRequest.ApplyChassisSpeeds m_speed = new SwerveRequest.ApplyChassisSpeeds().withSpeeds(chassisSpeeds);
        setControl(m_speed);
        // setControl(SwerveRequest.FieldCentric().withD);
        // setControl(kinematics.toSwerveRe(chassisSpeeds));
    }
    public void driveVisionDistance(){
        var results = m_Camera.getLatestResult();
        double forward;

        if(results.hasTargets()){
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                                Constants.LimeLightConstants.CAMERA_HEIGHT_METERS,
                                Constants.LimeLightConstants.TARGET_HEIGHT_METERS,
                                Constants.LimeLightConstants.CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(results.getBestTarget().getPitch()));
            forward = controller.calculate(range, Constants.LimeLightConstants.GOAL_RANGE_METERS);
        }
        else{
            forward = 0;
        }
       // SwerveRequest.
       //comented just for now to run code
    }
}
