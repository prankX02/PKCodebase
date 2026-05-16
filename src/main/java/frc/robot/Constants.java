package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
        public static enum Mode {
        /** Running on a real robot. */
        REAL,
 
        /** Running a physics simulator. */
        SIM,
 
        /** Replaying from a log file. */
        REPLAY
    }
 
    public static enum RobotType {
        COMPETITION, // The competition robot (with aluminum base)
 
        /** The Sim Bot */
        SIMBOT
    }
 
    // This is the current robot type, change for running on real robot
    private static final RobotType robotType = RobotType.SIMBOT;    
 
    // This is only a fallback! This will not change the robot type.
    private static final RobotType defaultRobotType = RobotType.COMPETITION;
 
    private static final Alert invalidRobotType = new Alert(
        "Invalid RobotType selected. Defaulting to " + defaultRobotType.toString(),
        AlertType.kWarning
    );
 
    public static RobotType getRobot() {
        if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
            invalidRobotType.set(true);
            return defaultRobotType;
        }
 
        return robotType;
    }
 
    public static final Mode getMode() {
        return switch(getRobot()) {
            case SIMBOT -> Mode.SIM;
            default -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        };
    }  
    public static class FieldConstants {
        public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }
    public static class DriveConstants {
        public static final double slowModeJoystickMultiplier = 0.4;
    }
}
