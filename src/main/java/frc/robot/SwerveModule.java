package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {
    public int moduleNumber;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    private boolean atTargetAngle = false;

    private double deadBand = 5d;

    private double kP = 0.105d;
    private double kI = 0.092d;
    private double kD = 0.001d;

    private double kDiv = 12;

    PIDController controller = new PIDController(kP, kI, kD);

    // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, int cancoderID, int angleMotorID, int driveMotorID, double offsetDegrees){
        this.moduleNumber = moduleNumber;
        
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        // TalonFXConfiguration falconConfiguration = new TalonFXConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.magnetOffsetDegrees = offsetDegrees;
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(cancoderID);
        angleEncoder.configFactoryDefault();

        controller.setIntegratorRange(-.3, .3);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(angleMotorID);
        // mAngleMotor.config_kP(0, 0.2);
        mAngleMotor.setNeutralMode(NeutralMode.Coast);
        //mAngleMotor.configSelectedFeedbackSensor(null)

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(driveMotorID);
        // resetToAbsolute();
    }
    public void resetToAbsolute(){
        double absolutePosition = Constants.degreesToFalcon(getCanCoder().getDegrees(), Constants.kGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void dumpToSmartDashBoard() {
    }

    public void setPercentOutput(double percentOutput) {
        mAngleMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void moveToTargetAngle(double targetAngle) {
        // returns motor angle between [0, 360]
        double currentAngle = angleEncoder.getAbsolutePosition();
        // get angle error between expected and actual angles
        double error = targetAngle - currentAngle;

        // TODO: add I-zones!!!

        SmartDashboard.putNumber(moduleNumber + " CANCoder", getCanCoder().getDegrees());
        SmartDashboard.putBoolean(moduleNumber + " At Target Angle", atTargetAngle);
        SmartDashboard.setDefaultNumber(moduleNumber + " P", kP);
        SmartDashboard.setDefaultNumber(moduleNumber + " I", kI);
        SmartDashboard.setDefaultNumber(moduleNumber + " D", kD);
        SmartDashboard.setDefaultNumber(moduleNumber + " Div", kDiv);
        kP = SmartDashboard.getNumber(moduleNumber + " P", 0.0);
        kI = SmartDashboard.getNumber(moduleNumber + " I", 0.0);
        kD = SmartDashboard.getNumber(moduleNumber + " D", 0.0);
        kDiv = SmartDashboard.getNumber(moduleNumber + " Div", 0.0);
        controller.setP(kP);    
        controller.setI(kI);
        controller.setD(kD);

        atTargetAngle = Math.abs(error) < deadBand;
        
        double voltage = controller.calculate(currentAngle, targetAngle);
        mAngleMotor.set(ControlMode.PercentOutput, voltage/kDiv);
    }
}