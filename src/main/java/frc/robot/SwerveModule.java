package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {
  public int moduleNumber;

  // motor IDs
  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;

  // module variables
  private double targetAngle;
  private boolean inverted;
  private double kNorm;
  private Rotation2d lastAngle;

  // kinematic controllers
  private PIDController controller;
  private SimpleMotorFeedforward driveFeedForward;

  // Construct a new Swerve Module using a preset Configuration
  public SwerveModule(Constants.SwerveModuleConfigurations configs) {
    this.moduleNumber = configs.moduleNumber;

    // Angle Encoder Config
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.magnetOffsetDegrees = configs.angleEncoderOffset;
    this.angleEncoder = new CANCoder(configs.angleEncoderId);

    this.angleEncoder.configAllSettings(canCoderConfiguration);

    // TalonFXConfiguration falconConfiguration = new TalonFXConfiguration();

    // Angle Motor Config
    this.angleMotor = new TalonFX(configs.angleMotorId);
    this.angleMotor.setNeutralMode(NeutralMode.Coast);

    // Drive Motor Config
    this.driveMotor = new TalonFX(configs.driveMotorId);
    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    this.driveMotor.setSelectedSensorPosition(0);

    this.driveMotor.config_kP(0, Constants.driveKP);
    this.driveMotor.config_kI(0, Constants.driveKI);
    this.driveMotor.config_kD(0, Constants.driveKD);
    this.driveMotor.config_kF(0, Constants.driveKF);

    this.controller = new PIDController(configs.kP, configs.kI, configs.kD);
    this.controller.setIntegratorRange(-0.3d, 0.3d);
    this.driveFeedForward = new SimpleMotorFeedforward(1.0d, configs.kF);
    this.kNorm = configs.kNorm;

    this.inverted = false;

    targetAngle = 0;
    // lastAngle = getState().angle;
    lastAngle = Rotation2d.fromDegrees(targetAngle);
  }

  // Debug swerve module information to SmartDashboard
  public void debug() {
    SmartDashboard.putNumber(moduleNumber + " CANCoder", angleEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean(moduleNumber + " Inverted", inverted);
    double kP = Utils.serializeNumber(moduleNumber + " P", controller.getP());
    double kI = Utils.serializeNumber(moduleNumber + " I", controller.getI());
    double kD = Utils.serializeNumber(moduleNumber + " D", controller.getD());
    kNorm = Utils.serializeNumber(moduleNumber + " Norm", kNorm);
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  // Sets the current target angle of the swerve module
  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public void moveToTargetAngle() {
    // returns motor angle between [0, 360]
    double currentAngle = angleEncoder.getAbsolutePosition();

    double delta = targetAngle - currentAngle;

    if (delta > 180) {
      delta -= 360;
    } else if (delta < -180) {
      delta += 360;
    }
    if (delta > 90) {
      delta -= 180;
    } else if (delta < -90) {
      delta += 180;
    }

    inverted = Math.abs(targetAngle - currentAngle) > 90;

    double output = controller.calculate(0, delta);
    angleMotor.set(ControlMode.PercentOutput, output / kNorm);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Utils.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
            Constants.wheelCircumference,
            Constants.driveGearRatio),
        getCanCoder());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Utils.falconToMPS(
            driveMotor.getSelectedSensorPosition(),
            Constants.wheelCircumference,
            Constants.driveGearRatio),
        getCanCoder());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */
    // desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01)) ? lastAngle
        : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    setTargetAngle(angle.getDegrees());
    moveToTargetAngle();
    // driveMotor.set(ControlMode.Position,
    // Utils.degreesToFalcon(angle.getDegrees(), Constants.angleGearRatio));
    lastAngle = angle;
    /*
     * if(isOpenLoop){
     * double percentOutput = desiredState.speedMetersPerSecond /
     * Constants.maxSpeed;
     * driveMotor.set(ControlMode.PercentOutput, percentOutput);
     * }
     */
    /*
     * else {
     * double velocity = Utils.MPSToFalcon(desiredState.speedMetersPerSecond,
     * Constants.wheelCircumference, Constants.driveGearRatio);
     * driveMotor.set(ControlMode.Velocity, velocity,
     * DemandType.ArbitraryFeedForward,
     * driveFeedForward.calculate(desiredState.speedMetersPerSecond));
     * }
     */
    double percentOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
    driveMotor.set(ControlMode.PercentOutput, percentOutput);
  }
}