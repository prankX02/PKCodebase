// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
 
import java.util.Arrays;
 
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOMaple;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOMaple;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight4;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.MapleSimUtil;
 
public class RobotContainer {
    private final CommandXboxController gamepad_ = new CommandXboxController(0);
 
    private Drive drivebase_;
    private AprilTagVision vision_;
 
    public RobotContainer() {
        buildRobot() ;
        createDefaultSubsystems() ;
 
        if (Constants.getRobot() == RobotType.SIMBOT) {
            MapleSimUtil.start();
        }             
 
        configureBindings();    
        configureDriveBindings() ;  
    }
 
    private void configureBindings() {    
    }
 
    private void configureDriveBindings() {    
        DriveCommands.configure(
            drivebase_,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()
        );  
 
       // Default command, normal field-relative drive
        drivebase_.setDefaultCommand(DriveCommands.joystickDrive().withName("JoystickDrive"));
 
        // Slow Mode, during left bumper
        gamepad_.leftBumper().whileTrue(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> -gamepad_.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getRightX() * DriveConstants.slowModeJoystickMultiplier));
 
        // Switch to X pattern / brake while X button is pressed
        gamepad_.x().whileTrue(drivebase_.stopWithXCmd()); 
 
        // Robot Relative
        gamepad_.povUp().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero()));
 
        gamepad_.povDown().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0),
                        RadiansPerSecond.zero()));
 
        gamepad_.povLeft().whileTrue(
                drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero()));
 
        gamepad_.povRight().whileTrue(
                drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(),
                        RadiansPerSecond.zero()));
 
        // Robot relative diagonal
        gamepad_.povUpLeft().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero()));
 
        gamepad_.povUpRight().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero()));
 
        gamepad_.povDownLeft().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero()));
 
        gamepad_.povDownRight().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero()));
 
        // Reset gyro to 0° when Y & B button is pressed
        gamepad_.y().and(gamepad_.b()).onTrue(drivebase_.resetGyroCmd());        
    }
 
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
 
    private void buildRobot() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch(Constants.getRobot()) {
                case SIMBOT:
                    buildSimBot() ;
                    break ;
 
                case COMPETITION:
                    buildComp() ;
                    break ;
            }
        }
        else {
 
        }
    }
 
    private void buildSimBot() {
        DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(COTS.ofMark4(
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                COTS.WHEELS.COLSONS.cof,
                2
            ))
            .withTrackLengthTrackWidth(
                Meters.of(Math.abs(
                    CompTunerConstants.FrontLeft.LocationX -
                    CompTunerConstants.BackLeft.LocationX
                )),
                Meters.of(Math.abs(
                    CompTunerConstants.FrontLeft.LocationY -
                    CompTunerConstants.FrontRight.LocationY
                ))
            )
            .withBumperSize(Inches.of(30.75), Inches.of(37.25));            
        MapleSimUtil.createSwerve(config, new Pose2d(2.0, 2.0, Rotation2d.kZero));
 
        drivebase_ = new Drive(
            new GyroIOMaple(),
            ModuleIOMaple::new,
            CompTunerConstants.FrontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight,
            CompTunerConstants.kCANBus,
            CompTunerConstants.kSpeedAt12Volts
        );        
 
        vision_ = new AprilTagVision(
            drivebase_::addVisionMeasurement,
            new CameraIOPhotonSim("front", VisionConstants.frontTransform, MapleSimUtil::getPosition, true),
            new CameraIOPhotonSim("backleft", VisionConstants.backLeftTransform, MapleSimUtil::getPosition, true),
            new CameraIOPhotonSim("backright", VisionConstants.backRightTransform, MapleSimUtil::getPosition, true)
        );        
    }
 
    private void buildComp() {
        drivebase_ = new Drive(
            new GyroIOPigeon2(CompTunerConstants.DrivetrainConstants.Pigeon2Id, CompTunerConstants.kCANBus),
            ModuleIOTalonFX::new,
            CompTunerConstants.FrontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight,
            CompTunerConstants.kCANBus,
            CompTunerConstants.kSpeedAt12Volts
        );
 
        vision_ = new AprilTagVision(
            drivebase_::addVisionMeasurement,
            new CameraIOLimelight4("limelight-front", drivebase_::getRotation),
            new CameraIOLimelight4("limelight-bl", drivebase_::getRotation),
            new CameraIOLimelight4("limelight-br", drivebase_::getRotation)
        );        
    }
 
    private void createDefaultSubsystems() {
        if (drivebase_ == null) {
 
        }
 
        if (vision_ == null) {
                int numCams = switch (Constants.getRobot()) {
                        default -> 3;
                };
 
                CameraIO[] cams = new CameraIO[numCams];
                Arrays.setAll(cams, i -> new CameraIO() {});
 
                vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        cams
                );
            }    
    }
}