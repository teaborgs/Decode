package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class OuttakeSystem {

	private final DcMotorEx motor;
	private VoltageSensor voltageSensor = null;

	public static double TICKS_PER_REV = 28;

	public static double kV = 1.0 / 5400.0;
	public static double kS = 0.0;

	public static double kP = 0.0015; //0.0024 //0.0010
	public static double kI = 0.0;
	public static double kD = 0.0; //0.007

	private double filteredRpm = 0;

	public static double RPM_SLEW = 6000; //50000

	public static double MAX_POWER = 1.0;
	public static double MIN_POWER = 0.0;

	public static double KICK_PWR = 0.28;
	public static double KICK_TIME_SEC = 0.28;
	public static double KICK_ERR_RPM = 90;
	public static double KICK_DROP_RPM_PER_S = 1400;
	public static double KICK_MIN_CMD_RPM = 1800;

	public static double OVERSHOOT_START_RPM = 35;
	public static double OVERSHOOT_FULL_RPM = 120;
	public static double OVERSHOOT_MIN_FF_SCALE = 0.80;

	public static double FILTER_OLD_WEIGHT = 0.72;
	public static double FILTER_NEW_WEIGHT = 0.28;

	public static double INTEGRAL_MAX = 8000;

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
		filteredRpm = 0;

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

		double rawRpm = getRpm();
		if (filteredRpm == 0 && rawRpm > 0) filteredRpm = rawRpm;
		filteredRpm = 0.4 * filteredRpm + 0.6 * rawRpm;
		double rpm = filteredRpm;
		/*double rawRpm = getRpm();
		if (filteredRpm == 0 && rawRpm > 0) filteredRpm = rawRpm;
		filteredRpm = FILTER_OLD_WEIGHT * filteredRpm + FILTER_NEW_WEIGHT * rawRpm;
		double rpm = filteredRpm;*/

		double err = cmdRpm - rpm;

		double rpmRate = (rpm - lastRpm) / dt;
		lastRpm = rpm;

		double dMeas = rpmRate;

		double vbat = 12.0;
		if (voltageSensor != null) {
			double v = voltageSensor.getVoltage();
			vbat = Math.max(11.0, Math.min(13.5, v));
		}
		double vComp = 12.0 / vbat;

		double ff = (kS + kV * cmdRpm) * vComp;

		double prelimPid = kP * err - kD * dMeas;
		double prelimPower = ff + prelimPid;

		boolean canIntegrate = err > 0 && prelimPower < MAX_POWER - 1e-6;
		if (canIntegrate) {
			integral += err * dt;
			if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
			if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;
		}

		double pid = kP * err + kI * integral - kD * dMeas;
		double power = ff + pid;

		/*boolean wantKick =
				cmdRpm >= KICK_MIN_CMD_RPM &&
						(err > KICK_ERR_RPM || rpmRate < -KICK_DROP_RPM_PER_S);

		if (wantKick) {
			kickUntilNs = Math.max(kickUntilNs, now + (long) (KICK_TIME_SEC * 1e9));
		}

		if (now < kickUntilNs) {
			power += KICK_PWR;
		}*/

		/*if (err < 0) {
			double overshoot = -err;
			if (overshoot > OVERSHOOT_START_RPM) {
				double t = (overshoot - OVERSHOOT_START_RPM) / Math.max(1.0, (OVERSHOOT_FULL_RPM - OVERSHOOT_START_RPM));
				if (t > 1.0) t = 1.0;
				double ffScale = 1.0 - t * (1.0 - OVERSHOOT_MIN_FF_SCALE);
				double maxAllowed = ff * ffScale;
				if (power > maxAllowed) power = maxAllowed;
			}
		}*/

		power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
		motor.setPower(power);

		lastErr = err;
	}

	public double getRpm() {
		double ticksPerSec = motor.getVelocity();
		return ticksPerSec * 60.0 / TICKS_PER_REV;
	}

	public void triggerKick() {
		long now = System.nanoTime();
		kickUntilNs = now + (long) (KICK_TIME_SEC * 1e9);
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