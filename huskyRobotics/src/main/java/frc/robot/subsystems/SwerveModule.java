import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

 public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity(); // Get velocity
    double angle = steerEncoder.getPosition();    // Get angle
    
    return new SwerveModuleState(
        velocity,
        new Rotation2d(Math.toRadians(angle))
    );
}

public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = desiredState.optimize(
        new Rotation2d(Math.toRadians(steerEncoder.getPosition()))
    );

    driveMotor.set(optimizedState.speedMetersPerSecond / MAX_SPEED);
    steerPIDController.setReference(
        optimizedState.angle.getDegrees(), 
        CANSparkMax.ControlType.kPosition
    );
}
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), // Gets position from drive encoder
            new Rotation2d(Math.toRadians(steerEncoder.getPosition())) // Gets angle from steer encoder
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