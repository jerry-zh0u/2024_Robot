package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants{
    public static class IntakeConstants{
        public static final int shooterMotorID = 14;
        public static final int intakeMotorID = 15; 
        public static final int retractMotorID = 19;
    }
    public static class ClimbConstants{
        public static final int leftMotorID = 0; //FIXME change IDs
        public static final int rightMotorID = 0; //FIXME change IDs
    }
    public static class DriveConstants{
        public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
    }
    public static class LimeLightConstants{
       // Constants such as camera and target height stored. Change per robot and goal!
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

        // Angle between horizontal and the camera.
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

        // How far from the target we want to be
        public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    }
}