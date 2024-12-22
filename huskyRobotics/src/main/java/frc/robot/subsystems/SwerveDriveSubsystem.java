package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    // Constants
    private static final int PIGEON_CAN_ID = 9;
    private static final double MAX_SPEED = 3.0; // meters per second
    private static final double WHEEL_BASE = 0.5; // meters
    private static final double TRACK_WIDTH = 0.5; // meters

    // Swerve Modules
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    // Pigeon IMU
    private final Pigeon2 pigeon;

    // Kinematics and Odometry
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public SwerveDriveSubsystem() {
        // Initialize swerve modules
        frontLeft = new SwerveModule(1, 2, "Front Left");
        frontRight = new SwerveModule(3, 4, "Front Right");
        backLeft = new SwerveModule(5, 6, "Back Left");
        backRight = new SwerveModule(7, 8, "Back Right");

        // Initialize Pigeon IMU
        pigeon = new Pigeon2(PIGEON_CAN_ID);
        pigeon.setYaw(0); // Reset yaw to 0

        // Define module locations relative to center of robot
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),  // Front Left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Back Left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Back Right
        );

        // Initialize odometry
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        // Set modules to brake mode
        configureBrakeMode(true);
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
        // Get current module states
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, 
                    Rotation2d.fromDegrees(pigeon.getYaw().getValue()))
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed)
        );
    
        // Normalize wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
    
        // Set states to each module
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    @Override
    public void periodic() {
        // Update odometry
        odometry.update(
            Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        // Update dashboard
        updateSmartDashboard();
    }

    private void updateSmartDashboard() {
        Pose2d pose = odometry.getPoseMeters();
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", pigeon.getYaw().getValue());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }

    private void configureBrakeMode(boolean brake) {
        frontLeft.setBrakeMode(brake);
        frontRight.setBrakeMode(brake);
        backLeft.setBrakeMode(brake);
        backRight.setBrakeMode(brake);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}

class SwerveModule {
    // Constants
    private static final double WHEEL_DIAMETER = 0.1016; // 4 inches in meters
    private static final double DRIVE_GEAR_RATIO = 8.14;
    private static final double STEER_GEAR_RATIO = 12.8;
    private static final double MAX_SPEED = 3.0; // meters per second

    // Hardware
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final SparkPIDController steerPIDController;
    private final String moduleName;

    public SwerveModule(int driveMotorId, int steerMotorId, String moduleName) {
        this.moduleName = moduleName;

        // Configure drive motor
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveEncoder = driveMotor.getEncoder();
        
        // Configure steering motor
        steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerEncoder = steerMotor.getEncoder();
        steerPIDController = steerMotor.getPIDController();

        // Configure PID
        steerPIDController.setP(0.5);
        steerPIDController.setI(0);
        steerPIDController.setD(0);
        steerPIDController.setOutputRange(-1, 1);

        // Set conversion factors
        double driveConversionFactor = (Math.PI * WHEEL_DIAMETER) / DRIVE_GEAR_RATIO;
        driveEncoder.setPositionConversionFactor(driveConversionFactor);
        driveEncoder.setVelocityConversionFactor(driveConversionFactor / 60.0);

        double steerConversionFactor = 360.0 / STEER_GEAR_RATIO;
        steerEncoder.setPositionConversionFactor(steerConversionFactor);
        steerEncoder.setVelocityConversionFactor(steerConversionFactor / 60.0);

        // Save configurations
        driveMotor.burnFlash();
        steerMotor.burnFlash();

        // Log configuration
        SmartDashboard.putString(moduleName + " Status", "Initialized");
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the desired state to avoid rotating more than 90 degrees
        SwerveModuleState optimizedState = desiredState.optimize(
            Rotation2d.fromDegrees(steerEncoder.getPosition()));
    
        // Set drive speed
        double normalizedSpeed = optimizedState.speedMetersPerSecond / MAX_SPEED;
        driveMotor.set(normalizedSpeed);
    
        // Set steering angle
        steerPIDController.setReference(optimizedState.angle.getDegrees(), 
            CANSparkMax.ControlType.kPosition);
    
        // Update dashboard
        SmartDashboard.putNumber(moduleName + " Speed", optimizedState.speedMetersPerSecond);
        SmartDashboard.putNumber(moduleName + " Angle", optimizedState.angle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            Rotation2d.fromDegrees(steerEncoder.getPosition())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(steerEncoder.getPosition())
        );
    }

    public void setBrakeMode(boolean brake) {
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        driveMotor.setIdleMode(mode);
        steerMotor.setIdleMode(mode);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}