package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OuttakeSystem {
	private final DcMotorEx motor;

	// goBILDA 6000RPM encoder intern
	public static double TICKS_PER_REV = 28;

	// ===== TUNING (începe cu astea) =====
	// kV = feedforward (aprox power per rpm). Dacă max real e ~5500 rpm la power=1,
	// kV ~ 1/5500 = 0.0001818
	public static double kV = 1.0 / 5000.0;

	// PID pe eroare RPM (astea controlează cât de “agresiv” corectează)
	public static double kP = 0.0010;
	public static double kI = 0.0;
	public static double kD = 0.0003;

	// rampă pentru target ca să nu “șuteze” flywheel-ul
	public static double RPM_SLEW = 6000; // rpm/sec

	// protecții
	public static double MAX_POWER = 1.0;
	public static double MIN_POWER = 0.0; // shooter de obicei nu vrei reverse aici

	// state
	private double cmdRpm = 0;
	private double integral = 0;
	private double lastErr = 0;
	private long lastNs = 0;

	public OuttakeSystem(DcMotorEx motor, DcMotorSimple.Direction direction) {
		this.motor = motor;
		this.motor.setDirection(direction);
	}

	public void init() {
		// BRAKE ajută enorm să nu mai “plutească” peste target la inerție
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		cmdRpm = 0;
		integral = 0;
		lastErr = 0;
		lastNs = System.nanoTime();
		motor.setPower(0);
	}

	/** Setează RPM țintă (custom PID pe RPM + FF). */
	public void setRpm(double targetRpm) {
		if (targetRpm < 0) targetRpm = 0;

		long now = System.nanoTime();
		double dt = (now - lastNs) / 1e9;
		if (dt <= 0) dt = 0.02;
		lastNs = now;

		// rampă pe target
		double maxDelta = RPM_SLEW * dt;
		double delta = targetRpm - cmdRpm;
		if (delta > maxDelta) delta = maxDelta;
		if (delta < -maxDelta) delta = -maxDelta;
		cmdRpm += delta;

		double rpm = getRpm();
		double err = cmdRpm - rpm;

		// anti-windup simplu: integrăm doar când nu suntem blocați la limită
		integral += err * dt;

		double deriv = (err - lastErr) / dt;
		lastErr = err;

		// Feedforward (power aproximativ pentru rpm) + PID
		double ff = kV * cmdRpm;
		double pid = kP * err + kI * integral + kD * deriv;

		double power = ff + pid;

		// IMPORTANT: dacă ești peste target (err negativ mare), taie power repede (nu-l lăsa să “plutească”)
		if (err < -150) { // poți regla pragul (150-300 rpm)
			power = Math.min(power, ff * 0.6); // frânează mai tare decât doar FF
		}

		// clamp
		if (power > MAX_POWER) power = MAX_POWER;
		if (power < MIN_POWER) power = MIN_POWER;

		motor.setPower(power);
	}

	/** RPM real (din encoder velocity). */
	public double getRpm() {
		double ticksPerSec = motor.getVelocity();
		return ticksPerSec * 60.0 / TICKS_PER_REV;
	}

	/** Oprire clean. */
	public void stop() {
		cmdRpm = 0;
		integral = 0;
		lastErr = 0;
		motor.setPower(0);
	}

	/** Debug manual. */
	public void setPower(double p) {
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setPower(p);
	}

	public void enableVelocityControl() {
		// păstrăm numele metodei ca să nu-ți pice alte coduri, dar aici rămânem pe custom power PID
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		cmdRpm = 0;
		integral = 0;
		lastErr = 0;
		lastNs = System.nanoTime();
		motor.setPower(0);
	}

	public DcMotorEx raw() { return motor; }
}
