package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Utils {
	
	/*
	 * public static double CANcoderToDegrees(double positionCounts, double
	 * gearRatio) {
	 * return positionCounts * (360.0 / (gearRatio * 4096.0));
	 * }
	 */
	public static double falconToDegrees(double positionCounts, double gearRatio) {
		return positionCounts * (360.0 / (gearRatio * 2048.0));
	}

	public static double degreesToFalcon(double degrees, double gearRatio) {
		return degrees / (360.0 / (gearRatio * 2048.0));
	}

	public static double serializeNumber(String key, double val) {
		SmartDashboard.setDefaultNumber(key, val);
		return SmartDashboard.getNumber(key, 0.0);
	}

	public static boolean serializeBoolean(String key, boolean val) {
		SmartDashboard.setDefaultBoolean(key, val);
		return SmartDashboard.getBoolean(key, false);
	}
}
