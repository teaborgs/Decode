package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TurretSystem extends AbstractSystem {
	private final DcMotorEx turretMotor;

	// Dacă vrei control open-loop (manual), folosește enum-ul ăsta.
	public enum TurretDirection {
		CW,         // clockwise
		CCW,        // counter-clockwise
		SLOW_CW,
		SLOW_CCW,
		STOP
	}

	public TurretSystem(DcMotorEx turretMotor) {
		this.turretMotor = turretMotor;

		// Ajustează dacă e invers la tine
		this.turretMotor.setDirection(DcMotor.Direction.REVERSE);

		this.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		stop();
	}

	@Override
	public void init() {
		stop();
	}

	private void stop() {
		turretMotor.setPower(0);
	}

	public DcMotorEx getMotor() {
		return turretMotor;
	}

	// ========= Open-loop manual control (opțional) =========

	public void setTurretDirection(TurretDirection direction) {
		switch (direction) {
			case CW:
				turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				turretMotor.setPower(1.0);
				break;
			case CCW:
				turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				turretMotor.setPower(-1.0);
				break;
			case SLOW_CW:
				turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				turretMotor.setPower(0.25);
				break;
			case SLOW_CCW:
				turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				turretMotor.setPower(-0.25);
				break;
			case STOP:
			default:
				stop();
				break;
		}
	}

	public Action setDirectionAction(TurretDirection direction) {
		return new InstantAction(() -> setTurretDirection(direction));
	}

	// ========= RoadRunner Actions =========

	/**
	 * Duce tureta la target absolut (ticks) folosind RUN_TO_POSITION.
	 *
	 * @param targetTicks target absolut encoder
	 * @param power       puterea de deplasare (0..1)
	 * @param tolTicks    toleranță (ex: 5-15)
	 * @param timeoutMs   timeout (ex: 1000-2000)
	 * @param holdPower   puterea cu care ține poziția după ce ajunge (ex: 0.05-0.15)
	 */
	public Action goToTicksAction(int targetTicks, double power, int tolTicks, long timeoutMs, double holdPower) {
		return new Action() {
			boolean init = false;
			long startMs = 0;

			@Override
			public boolean run(TelemetryPacket packet) {
				if (!init) {
					init = true;
					startMs = System.currentTimeMillis();

					turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
					turretMotor.setTargetPosition(targetTicks);
					turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
					turretMotor.setPower(Math.abs(power));
				}

				int pos = turretMotor.getCurrentPosition();
				int err = targetTicks - pos;

				packet.put("turret_target", targetTicks);
				packet.put("turret_pos", pos);
				packet.put("turret_err", err);
				packet.put("turret_busy", turretMotor.isBusy());

				boolean atTarget = Math.abs(err) <= tolTicks;
				boolean timedOut = (System.currentTimeMillis() - startMs) >= timeoutMs;

				if (atTarget || timedOut) {
					// ține pe target
					turretMotor.setTargetPosition(targetTicks);
					turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
					turretMotor.setPower(holdPower);

					packet.put("turret_done", atTarget ? "OK" : "TIMEOUT");
					return false;
				}

				return true;
			}
		};
	}

	/**
	 * Duce tureta la 0 (encoder), apoi face STOP_AND_RESET_ENCODER ca să pornească TeleOp cu 0 fix.
	 *
	 * Atenție: asta resetează encoderul "din software" — dacă 0 mecanic nu e consistent, ai nevoie de limit switch/hardstop.
	 */
	public Action goToZeroAndResetAction(double power, int tolTicks, long timeoutMs, double holdPower) {
		return new Action() {
			boolean init = false;
			long startMs = 0;

			@Override
			public boolean run(TelemetryPacket packet) {
				if (!init) {
					init = true;
					startMs = System.currentTimeMillis();

					turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
					turretMotor.setTargetPosition(0);
					turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
					turretMotor.setPower(Math.abs(power));
				}

				int pos = turretMotor.getCurrentPosition();

				packet.put("turret_zero_pos", pos);
				packet.put("turret_busy", turretMotor.isBusy());

				boolean atZero = Math.abs(pos) <= tolTicks;
				boolean timedOut = (System.currentTimeMillis() - startMs) >= timeoutMs;

				if (atZero || timedOut) {
					turretMotor.setPower(0);

					// reset encoder -> teleop pornește de la 0 fix
					turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

					// opțional: ține 0 (dacă vrei să nu “cadă”/alunece)
					turretMotor.setTargetPosition(0);
					turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
					turretMotor.setPower(holdPower);

					packet.put("turret_zero_done", atZero ? "OK_RESET" : "TIMEOUT_RESET");
					return false;
				}

				return true;
			}
		};
	}
}