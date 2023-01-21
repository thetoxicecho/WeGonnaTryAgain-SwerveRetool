package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

	// kinematic controllers
    private PIDController controller;
	private SimpleMotorFeedforward driveFeedForward;

	public SwerveModule(Constants.SwerveModuleConfigurations configs) {
		this.moduleNumber = configs.moduleNumber;
        
        // Angle Encoder Config
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.magnetOffsetDegrees = configs.angleEncoderOffset;
        this.angleEncoder = new CANCoder(configs.angleEncoderId);
        // this.angleEncoder.configFactoryDefault();
        this.angleEncoder.configAllSettings(canCoderConfiguration);
        
		// TalonFXConfiguration falconConfiguration = new TalonFXConfiguration();
		
        /* Angle Motor Config */
        this.angleMotor = new TalonFX(configs.angleMotorId);
        this.angleMotor.setNeutralMode(NeutralMode.Coast);
		
        /* Drive Motor Config */
        this.driveMotor = new TalonFX(configs.driveMotorId);

		this.controller = new PIDController(configs.kP, configs.kI, configs.kD);
		this.controller.setIntegratorRange(-0.3d, 0.3d);
		this.driveFeedForward = new SimpleMotorFeedforward(1.0d, configs.kF);
		this.kNorm = configs.kNorm;

		this.inverted = false;
	}

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

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void moveToTargetAngle() {
		debug();
        // returns motor angle between [0, 360]
        double currentAngle = angleEncoder.getAbsolutePosition();

        double delta = targetAngle - currentAngle;

        if (delta > 180) {
            delta -= 360;
        } 
        else if (delta < -180) {
            delta += 360;
        }
        if (delta > 90) {
            delta -= 180;
        }
        else if (delta < -90) {
            delta += 180;
        }

        inverted = Math.abs(targetAngle - currentAngle) > 90;
        
        double output = controller.calculate(0, delta);
        angleMotor.set(ControlMode.PercentOutput, output / kNorm);
    }
}