package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class OuttakeSystem {

	private final DcMotorEx motor;
	private VoltageSensor voltageSensor = null;

	public static double TICKS_PER_REV = 28;

	// ===== TUNING =====
	public static double kV = 1.0 / 4700.0;
	public static double kS = 0.03;

	public static double kP = 0.0018;
	public static double kI = 0.0;
	public static double kD = 0.0004;

	public static double RPM_SLEW = 20000;

	public static double MAX_POWER = 1.0;
	public static double MIN_POWER = 0.0;

	// ===== Kick =====
	public static double KICK_PWR = 0.22;
	public static double KICK_TIME_SEC = 0.22;
	public static double KICK_ERR_RPM = 120;
	public static double KICK_DROP_RPM_PER_S = 2000;
	public static double KICK_MIN_CMD_RPM = 1800;

	private double cmdRpm = 0;
	private double integral = 0;
	private double lastErr = 0;
	private long lastNs = 0;

	private double lastRpm = 0;
	private long kickUntilNs = 0;

	public OuttakeSystem(DcMotorEx motor, DcMotorSimple.Direction direction) {
		this.motor = motor;
		this.motor.setDirection(direction);
	}

	public void setVoltageSensor(VoltageSensor sensor) {
		this.voltageSensor = sensor;
	}

	public void init() {
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		cmdRpm = 0;
		integral = 0;
		lastErr = 0;
		lastRpm = 0;
		lastNs = System.nanoTime();
		kickUntilNs = 0;

		motor.setPower(0);
	}

	public void setRpm(double targetRpm) {
		if (targetRpm < 0) targetRpm = 0;

		long now = System.nanoTime();
		double dt = (now - lastNs) / 1e9;
		if (dt <= 0) dt = 0.02;
		lastNs = now;

		double maxDelta = RPM_SLEW * dt;
		double delta = targetRpm - cmdRpm;
		delta = Math.max(-maxDelta, Math.min(maxDelta, delta));
		cmdRpm += delta;

		double rpm = getRpm();
		double err = cmdRpm - rpm;

		double rpmRate = (rpm - lastRpm) / dt;
		lastRpm = rpm;

		double deriv = (err - lastErr) / dt;
		lastErr = err;

		integral += err * dt;

		double vbat = 12.0;
		if (voltageSensor != null) {
			double v = voltageSensor.getVoltage();
			vbat = Math.max(11.0, Math.min(13.5, v));
		}
		double vComp = 12.0 / vbat;

		double ff = (kS + kV * cmdRpm) * vComp;
		double pid = kP * err + kI * integral + kD * deriv;

		double power = ff + pid;

		boolean wantKick =
				cmdRpm >= KICK_MIN_CMD_RPM &&
						(err > KICK_ERR_RPM || rpmRate < -KICK_DROP_RPM_PER_S);

		if (wantKick) {
			kickUntilNs = Math.max(kickUntilNs,
					now + (long)(KICK_TIME_SEC * 1e9));
		}

		if (now < kickUntilNs) {
			power += KICK_PWR;
		}

		if (err < -150) {
			power = Math.min(power, ff * 0.6);
		}

		power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
		motor.setPower(power);
	}

	public double getRpm() {
		double ticksPerSec = motor.getVelocity();
		return ticksPerSec * 60.0 / TICKS_PER_REV;
	}

	public void triggerKick() {
		long now = System.nanoTime();
		kickUntilNs = now + (long)(KICK_TIME_SEC * 1e9);
	}

	public void stop() {
		cmdRpm = 0;
		integral = 0;
		lastErr = 0;
		kickUntilNs = 0;
		motor.setPower(0);
	}

	public DcMotorEx raw() { return motor; }
}