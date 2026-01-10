package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "decodeaza-mi-l v2", group = "TeleOp")
public final class decodev2 extends BaseOpMode {

	private InputSystem driveInput, armInput;
	private RobotHardware robot;

	private static class Keybindings {
		public static class Drive {
			public static final InputSystem.Key SHOOTER_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key INTAKE_REVERSE_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key INTAKE_KEY = new InputSystem.Key("right_bumper");

			public static final InputSystem.Axis TURRET_ANGLE = new InputSystem.Axis("right_stick_x");
			public static final InputSystem.Axis SHOOTER_ANGLE = new InputSystem.Axis("right_stick_y");

			public static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
			public static final InputSystem.Axis DRIVE_X = new InputSystem.Axis("left_stick_x");
			public static final InputSystem.Axis DRIVE_Y = new InputSystem.Axis("left_stick_y");
			public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
			public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");

			public static final InputSystem.Key LL_AIM = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key ODO_LOCK = new InputSystem.Key("dpad_right");

			public static final InputSystem.Key AUTO_SHOOTER_ENABLE_KEY = new InputSystem.Key("start");
			public static final InputSystem.Key AUTO_SHOOTER_DISABLE_KEY = new InputSystem.Key("back");
		}
	}

	// ================== DRIVE / SHOOTER ==================
	private static final double TURRET_DEADZONE = 0.12;
	private static final double SHOOTER_DEADZONE = 0.15;

	private boolean transfer_servo = false;
	private double shooterPosition = 0.5;

	private boolean autoShooterAngle = true;
	private boolean prevAutoShooterKeyPressed = false;
	private boolean prevAutoAngleKeyPressed = false;

	// ================== TURRET MODES ==================
	private enum TurretAimMode { MANUAL, LL_ONLY, WORLD_TARGET }
	private TurretAimMode turretMode = TurretAimMode.LL_ONLY;

	private boolean prevLlAimPressed = false;
	private boolean prevOdoLockPressed = false;
	private boolean turretAligned = false;

	// ================== TURRET LIMITS & GAINS ==================
	private static final int TURRET_MIN_TICKS = -335;
	private static final int TURRET_MAX_TICKS =  307;

	private static final double TURRET_TICKS_PER_RAD = 380.0;

	private static final double LL_DEADZONE_START = 1.6;
	private static final double LL_DEADZONE_STOP  = 0.9;

	private static final double LL_KP_FAST  = 0.020;
	private static final double LL_MAX_FAST = 0.55;

	private static final double HOLD_KP  = 0.007;
	private static final double HOLD_MAX = 0.45;

	// ================== WORLD POINT LOCK ==================
	private boolean worldLocked = false;
	private boolean worldJustEntered = false;

	private double targetWorldX = 0.0;
	private double targetWorldY = 0.0;

	private int lastWorldTargetTicks = 0;

	private static final double WORLD_CAPTURE_TX_MAX_DEG = 4.0;
	private static final int WORLD_MAX_STEP_TICKS = 10;

	// ================== WORLD + LL MICRO CORRECTION ==================
	private static final double LL_WORLD_MICRO_ACTIVE_TX_MAX_DEG = 8.0;  // era 12
	private static final double LL_WORLD_MICRO_DEADZONE_DEG = 1.0;       // era 0.6

	private static final double LL_WORLD_NUDGE_TICKS_PER_DEG = 2.0;      // era 4.0
	private static final double LL_WORLD_NUDGE_GAIN = 0.18;              // era 0.35
	private static final int    LL_WORLD_MAX_NUDGE_TICKS = 10;           // era 18


	private static final double WORLD_RELOCK_TX_DEG = 0.8;
	private static final double WORLD_RELOCK_ALPHA = 0.04;

	// ================== LIMELIGHT DISTANCE CONSTANTS ==================
	private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 85.0;
	private static final double LIMELIGHT_LENS_HEIGHT_CM  = 40.0;
	private static final double GOAL_HEIGHT_CM            = 105.0;

	@Override
	protected void OnInitialize() {
		driveInput = input1;
		armInput = input2;
	}

	@Override
	protected void jOnInitialize() {
		driveInput = input1;
		armInput = input2;
	}

	@Override
	protected void OnStart() {
		robot = new RobotHardware(hardwareMap);
		robot.init();
		robot.limelight.pipelineSwitch(0);

		shooterPosition = robot.turretTumbler.getPosition();
		robot.turret.getMotor().setZeroPowerBehavior(
				com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
		);
	}

	@Override
	protected void OnRun() {
		if (robot != null) robot.drivetrain.updatePoseEstimate();

		// Toggle auto shooter angle
		boolean autoShooterKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_ENABLE_KEY);
		if (autoShooterKey && !prevAutoShooterKeyPressed) autoShooterAngle = !autoShooterAngle;
		prevAutoShooterKeyPressed = autoShooterKey;

		boolean autoAngleKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_DISABLE_KEY);
		if (autoAngleKey && !prevAutoAngleKeyPressed) autoShooterAngle = !autoShooterAngle;
		prevAutoAngleKeyPressed = autoAngleKey;

		// DPAD_UP: MANUAL <-> LL_ONLY
		boolean llAimPressed = driveInput.isPressed(Keybindings.Drive.LL_AIM);
		if (llAimPressed && !prevLlAimPressed) {
			turretMode = (turretMode == TurretAimMode.MANUAL) ? TurretAimMode.LL_ONLY : TurretAimMode.MANUAL;
			turretAligned = false;
			worldLocked = false;
			worldJustEntered = false;
		}
		prevLlAimPressed = llAimPressed;

		// DPAD_RIGHT: LL_ONLY <-> WORLD_TARGET
		boolean worldPressed = driveInput.isPressed(Keybindings.Drive.ODO_LOCK);
		if (worldPressed && !prevOdoLockPressed) {
			if (turretMode != TurretAimMode.WORLD_TARGET) {
				turretMode = TurretAimMode.WORLD_TARGET;
				worldLocked = false;
				worldJustEntered = true;
				lastWorldTargetTicks = robot.turret.getMotor().getCurrentPosition();
			} else {
				turretMode = TurretAimMode.LL_ONLY;
				turretAligned = false;
				worldLocked = false;
				worldJustEntered = false;
			}
		}
		prevOdoLockPressed = worldPressed;

		Drive();
		Shooter();
		Turret();
		ShooterAngle();
		Intake();
		UpdateLimelight();
	}

	private void UpdateLimelight() {
		// placeholder (dacă ai nevoie)
	}

	private void Drive() {
		float speed = 1f;
		if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;

		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(
						new Vector2d(
								driveInput.getValue(Keybindings.Drive.DRIVE_Y),
								driveInput.getValue(Keybindings.Drive.DRIVE_X)
						).times(-speed),
						(driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L)
								- driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
				)
		);
	}

	private void Shooter() {
		if (driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY)) {
			robot.outtake1.setIntakeDirection(IntakeDirection.FORWARD);
			robot.outtake2.setIntakeDirection(IntakeDirection.FORWARD);
			transfer_servo = true;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		} else {
			robot.outtake1.setIntakeDirection(IntakeDirection.STOP);
			robot.outtake2.setIntakeDirection(IntakeDirection.STOP);
			transfer_servo = false;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
		}
	}

	private void Intake() {
		if (driveInput.isPressed(Keybindings.Drive.INTAKE_KEY)) {
			if (transfer_servo) robot.transfer.start();
			robot.intake.setIntakeDirection(IntakeDirection.FORWARD);
		} else if (driveInput.isPressed(Keybindings.Drive.INTAKE_REVERSE_KEY)) {
			robot.intake.setIntakeDirection(IntakeDirection.REVERSE);
		} else {
			robot.transfer.stop();
			robot.intake.setIntakeDirection(IntakeDirection.STOP);
		}
	}

	// ================== SHOOTER ANGLE ==================
	private double shooterAngleFromDistanceCm(double distanceCm) {
		// păstrat fix cum aveai (poți retuna)
		double dNear = -1.0;
		double posNear = 0.6;

		double dFar = 15.0;
		double posFar = 0.75;

		if (distanceCm <= 15.0 && distanceCm >= 8.0) return shooterPosition = posNear;
		else if (distanceCm <= 8.0 && distanceCm >= 2.0) return shooterPosition = 0.7;
		else if (distanceCm <= 2.0 && distanceCm >= -1) return shooterPosition = 0.7;
		else if (distanceCm >= dFar) return shooterPosition = posFar;
		else return shooterPosition = 0.0;
	}

	private void ShooterAngle() {
		if (autoShooterAngle) {
			if (robot == null || robot.limelight == null) return;

			LLResult result = robot.limelight.getLatestResult();
			if (result != null && result.isValid()) {
				double ty = result.getTy();

				double angleDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
				double angleRad = Math.toRadians(angleDeg);

				Double distanceCm = null;
				if (Math.abs(angleRad) > 0.001) {
					distanceCm = (GOAL_HEIGHT_CM - LIMELIGHT_LENS_HEIGHT_CM) / Math.tan(angleRad);
				}

				if (distanceCm != null) {
					shooterPosition = shooterAngleFromDistanceCm(distanceCm);
					shooterPosition = clamp(shooterPosition, 0.0, 1.0);
					robot.turretTumbler.setPosition(shooterPosition);
				}

				telemetry.addData("AUTO ty", ty);
				telemetry.addData("AUTO dist(cm)", distanceCm);
				telemetry.addData("AUTO shooterPos", shooterPosition);
			} else {
				telemetry.addLine("AUTO: no valid LL result");
			}
			return;
		}

		// manual
		double input = -driveInput.getValue(Keybindings.Drive.SHOOTER_ANGLE);
		if (Math.abs(input) < SHOOTER_DEADZONE) return;

		shooterPosition += input * 0.01;
		shooterPosition = clamp(shooterPosition, 0.0, 1.0);
		robot.turretTumbler.setPosition(shooterPosition);
	}

	// ================== LIMELIGHT DISTANCE (INCHES) ==================
	private Double getDistanceInchesFromLimelight(LLResult result) {
		if (result == null || !result.isValid()) return null;

		double ty = result.getTy();
		double angleDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
		double angleRad = Math.toRadians(angleDeg);
		if (Math.abs(angleRad) < 1e-3) return null;

		double distCm = (GOAL_HEIGHT_CM - LIMELIGHT_LENS_HEIGHT_CM) / Math.tan(angleRad);
		return distCm / 2.54; // inches (RR)
	}

	// ================== TURRET ==================
	private void Turret() {
		double stick = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);
		int ticks = robot.turret.getMotor().getCurrentPosition();

		// limits in manual
		if (ticks <= TURRET_MIN_TICKS && stick < 0) stick = 0;
		if (ticks >= TURRET_MAX_TICKS && stick > 0) stick = 0;

		LLResult result = (robot.limelight != null) ? robot.limelight.getLatestResult() : null;
		boolean hasTarget = (result != null && result.isValid());
		double txDeg = hasTarget ? result.getTx() : 0.0;

		Pose2d pose = robot.drivetrain.pose;
		double robotX = pose.position.x;
		double robotY = pose.position.y;
		double robotHeading = pose.heading.log();

		// ===== MANUAL =====
		if (turretMode == TurretAimMode.MANUAL) {
			if (Math.abs(stick) < TURRET_DEADZONE) {
				robot.turret.getMotor().setPower(0);
				return;
			}
			robot.turret.getMotor().setPower(clamp(stick * 0.35, -0.35, 0.35));
			return;
		}

		// ===== LL_ONLY =====
		if (turretMode == TurretAimMode.LL_ONLY) {
			if (!hasTarget) {
				robot.turret.getMotor().setPower(0);
				return;
			}

			// hysteresis
			if (turretAligned) {
				if (Math.abs(txDeg) > LL_DEADZONE_START) turretAligned = false;
				else { robot.turret.getMotor().setPower(0); return; }
			} else {
				if (Math.abs(txDeg) < LL_DEADZONE_STOP) {
					turretAligned = true;
					robot.turret.getMotor().setPower(0);
					return;
				}
			}

			double p = clamp(LL_KP_FAST * txDeg, -LL_MAX_FAST, LL_MAX_FAST);

			if (p > 0 && ticks >= TURRET_MAX_TICKS) p = 0;
			if (p < 0 && ticks <= TURRET_MIN_TICKS) p = 0;

			robot.turret.getMotor().setPower(p);
			return;
		}

		// ===== WORLD_TARGET (WORLD POINT LOCK + LL micro correction) =====
		if (turretMode == TurretAimMode.WORLD_TARGET) {

			// 1) capture world point
			if (worldJustEntered) {
				if (!hasTarget || Math.abs(txDeg) > WORLD_CAPTURE_TX_MAX_DEG) {
					robot.turret.getMotor().setPower(0);
					return;
				}

				Double distIn = getDistanceInchesFromLimelight(result);
				if (distIn == null || distIn < 1.0) {
					robot.turret.getMotor().setPower(0);
					return;
				}

				double txRad = Math.toRadians(txDeg);

				// dacă merge invers, schimbă - cu +
				double worldBearing = robotHeading - txRad;

				targetWorldX = robotX + Math.cos(worldBearing) * distIn;
				targetWorldY = robotY + Math.sin(worldBearing) * distIn;

				worldLocked = true;
				worldJustEntered = false;
				lastWorldTargetTicks = ticks;
			}

			if (!worldLocked) {
				robot.turret.getMotor().setPower(0);
				return;
			}

			// 2) ODO target ticks
			double dx = targetWorldX - robotX;
			double dy = targetWorldY - robotY;

			double angleWorld = Math.atan2(dy, dx);
			double desiredRel = wrapRad(angleWorld - robotHeading);

			int targetTicks = (int) Math.round(desiredRel * TURRET_TICKS_PER_RAD);
			targetTicks = clampInt(targetTicks, TURRET_MIN_TICKS, TURRET_MAX_TICKS);

			// 2.5) LL micro correction if tag visible
			if (hasTarget && Math.abs(txDeg) <= LL_WORLD_MICRO_ACTIVE_TX_MAX_DEG) {

				if (Math.abs(txDeg) > LL_WORLD_MICRO_DEADZONE_DEG) {
					int nudge = (int) Math.round(txDeg * LL_WORLD_NUDGE_TICKS_PER_DEG * LL_WORLD_NUDGE_GAIN);
					nudge = clampInt(nudge, -LL_WORLD_MAX_NUDGE_TICKS, LL_WORLD_MAX_NUDGE_TICKS);
					nudge = clampInt(nudge, -3, 3);


					// dacă îți corectează invers: schimbă += cu -=
					targetTicks -= nudge;
					targetTicks = clampInt(targetTicks, TURRET_MIN_TICKS, TURRET_MAX_TICKS);
				}

				// relock lent când e aproape centrat
				if (Math.abs(txDeg) < WORLD_RELOCK_TX_DEG) {
					Double distIn = getDistanceInchesFromLimelight(result);
					if (distIn != null && distIn > 1.0) {
						double txRad = Math.toRadians(txDeg);

						double worldBearing = robotHeading - txRad;

						double newX = robotX + Math.cos(worldBearing) * distIn;
						double newY = robotY + Math.sin(worldBearing) * distIn;

						targetWorldX = targetWorldX * (1.0 - WORLD_RELOCK_ALPHA) + newX * WORLD_RELOCK_ALPHA;
						targetWorldY = targetWorldY * (1.0 - WORLD_RELOCK_ALPHA) + newY * WORLD_RELOCK_ALPHA;
					}
				}
			}

			// 3) rate limit (anti-jump)
			int step = clampInt(targetTicks - lastWorldTargetTicks, -WORLD_MAX_STEP_TICKS, WORLD_MAX_STEP_TICKS);
			targetTicks = lastWorldTargetTicks + step;
			lastWorldTargetTicks = targetTicks;

			// 4) hold P
			int err = targetTicks - ticks;
			double pHold = clamp(HOLD_KP * err, -HOLD_MAX, HOLD_MAX);

			if (pHold > 0 && ticks >= TURRET_MAX_TICKS) pHold = 0;
			if (pHold < 0 && ticks <= TURRET_MIN_TICKS) pHold = 0;

			robot.turret.getMotor().setPower(pHold);
			return;
		}

		// fallback
		robot.turret.getMotor().setPower(0);
	}

	// ================== HELPERS ==================
	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}

	private static int clampInt(int v, int lo, int hi) {
		return Math.max(lo, Math.min(hi, v));
	}

	private static double wrapRad(double a) {
		while (a > Math.PI) a -= 2.0 * Math.PI;
		while (a < -Math.PI) a += 2.0 * Math.PI;
		return a;
	}

	// ================== TELEMETRY ==================
	@Override
	protected void OnTelemetry(Telemetry telemetry) {
		super.OnTelemetry(telemetry);

		if (robot != null && robot.drivetrain != null) {
			Pose2d p = robot.drivetrain.pose;
			telemetry.addData("ODO X (in)", p.position.x);
			telemetry.addData("ODO Y (in)", p.position.y);
			telemetry.addData("ODO H (deg)", Math.toDegrees(p.heading.log()));
		}

		if (robot == null || robot.limelight == null) {
			telemetry.addLine("Robot sau Limelight NULL");
			return;
		}

		LLResult result = robot.limelight.getLatestResult();
		boolean hasTarget = (result != null && result.isValid());

		double tx = hasTarget ? result.getTx() : 999;
		double ty = hasTarget ? result.getTy() : 999;
		double ta = hasTarget ? result.getTa() : 0;

		Double distIn = hasTarget ? getDistanceInchesFromLimelight(result) : null;

		telemetry.addData("LL tx", tx);
		telemetry.addData("LL ty", ty);
		telemetry.addData("LL ta", ta);
		telemetry.addData("LL Last Update", robot.limelight.getTimeSinceLastUpdate());
		telemetry.addData("LL Dist (in)", distIn);

		telemetry.addData("Auto Shooter Angle", autoShooterAngle ? "ON" : "OFF");
		telemetry.addData("TurretMode", turretMode);
		telemetry.addData("TurretTicks", robot.turret.getMotor().getCurrentPosition());
		telemetry.addData("Shooter Pos", shooterPosition);

		telemetry.addData("WorldLocked", worldLocked);
		telemetry.addData("WorldX", targetWorldX);
		telemetry.addData("WorldY", targetWorldY);
	}
}
