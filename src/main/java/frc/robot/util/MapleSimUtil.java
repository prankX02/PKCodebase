package frc.robot.util;
 
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
 
public class MapleSimUtil {
    private static SwerveDriveSimulation drivebaseSimulation;
 
    private static final Command run =
        Commands.run(MapleSimUtil::periodic)
            .ignoringDisable(true)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
 
    /** Starts the MapleSim simulation, kicks off a periodic method. */
    public static void start() {
        var rebuilt = (Arena2026Rebuilt) SimulatedArena.getInstance();
        rebuilt.resetFieldForAuto();
        CommandScheduler.getInstance().schedule(run);
    }
 
    private static void periodic() {
        var arena = (Arena2026Rebuilt) SimulatedArena.getInstance();       
        arena.simulationPeriodic();
    }
 
    public static SwerveDriveSimulation createSwerve(DriveTrainSimulationConfig config, Pose2d initialPose) {
        drivebaseSimulation = new SwerveDriveSimulation(config, initialPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(drivebaseSimulation);
        return drivebaseSimulation;
    }
 
    public static Pose2d getPosition() {
        return drivebaseSimulation.getSimulatedDriveTrainPose();
    }
 
    public static void placeRobotOnField(Pose2d pose) {
        drivebaseSimulation.setSimulationWorldPose(pose);
    }
 
    public static ChassisSpeeds getFieldChassisSpeeds() {
        return drivebaseSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }
 
    public static GyroSimulation getGyroSimulation() {
        return drivebaseSimulation.getGyroSimulation();
    }
 
    public static SwerveModuleSimulation[] getModuleSimulations() {
        return drivebaseSimulation.getModules();
    }
 }