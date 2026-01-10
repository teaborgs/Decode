/*package org.firstinspires.ftc.teamcode.teleop;

import android.util.JsonReader;
import android.util.JsonToken;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.onbotjava.JavaSourceFile;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;
import org.firstinspires.ftc.teamcode.systems.TransferSystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "\uD80C\uDC1B\uD83C\uDF46decodeaza-mi-l albastru\uD80C\uDC1B\uD83C\uDF46", group = "TeleOp")
public final class decode_patched extends BaseOpMode {
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
			public static final InputSystem.Key EXTENDO_ZERO_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key GRAB_TRANSFER_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key GRAB_HOLD_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key GRAB_WALL_KEY = new InputSystem.Key("x");
			public static final InputSystem.Key GRAB_RESET_KEY = new InputSystem.Key("y");

			public static final InputSystem.Key AUTO_SHOOTER_ENABLE_KEY = new InputSystem.Key("start");
			public static final InputSystem.Key AUTO_SHOOTER_DISABLE_KEY = new InputSystem.Key("back");
		}
	}

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
	}

	@Override
	protected void OnRun() {

		if (robot != null) {
			robot.drivetrain.updatePoseEstimate();
		}

		// === TOGGLE autoShooterAngle with START ===
		boolean autoShooterKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_ENABLE_KEY);

		if (autoShooterKey && !prevAutoShooterKeyPressed) {
			autoShooterAngle = !autoShooterAngle;  // toggle
		}
		prevAutoShooterKeyPressed = autoShooterKey;

		// === Turret mode toggles ===
// DPAD_UP: MANUAL <-> LL_ONLY
boolean llAimPressed = driveInput.isPressed(Keybindings.Drive.LL_AIM);
if (llAimPressed && !prevLlAimPressed) {
	if (turretMode == TurretAimMode.MANUAL) turretMode = TurretAimMode.LL_ONLY;
	else turretMode = TurretAimMode.MANUAL;

	odoLocked = false;
	turretAligned = false;
}
prevLlAimPressed = llAimPressed;

// DPAD_RIGHT: LL_ONLY <-> ODO_LOCK
boolean odoLockPressed = driveInput.isPressed(Keybindings.Drive.ODO_LOCK);
if (odoLockPressed && !prevOdoLockPressed) {
	if (turretMode != TurretAimMode.ODO_LOCK) {
		turretMode = TurretAimMode.ODO_LOCK;
	} else {
		turretMode = TurretAimMode.LL_ONLY;
	}
	odoLocked = false;
	turretAligned = false;
}
prevOdoLockPressed = odoLockPressed;

boolean autoAngleKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_DISABLE_KEY);

		if(autoAngleKey && !prevAutoAngleKeyPressed){
			autoShooterAngle = !autoShooterAngle;
		}
		prevAutoAngleKeyPressed = autoAngleKey;

		Drive();
		Shooter();
		Turret();
		ShooterAngle();
		Intake();
		UpdateLimelight();
	}

	private void Drive() {
		float speed = 1f;
		if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;
		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(new Vector2d(driveInput.getValue(Keybindings.Drive.DRIVE_Y),
						driveInput.getValue(Keybindings.Drive.DRIVE_X)).times(-speed),
						(driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
				)
		);
	}

	private static final double TURRET_DEADZONE = 0.12;
	private static final double SHOOTER_DEADZONE = 0.15;
	private boolean transfer_servo = false;
	private enum TurretAimMode { MANUAL, LL_ONLY, ODO_LOCK }
private TurretAimMode turretMode = TurretAimMode.LL_ONLY; // start like before (auto aim on)

// toggle edge-detect
private boolean prevLlAimPressed = false;
private boolean prevOdoLockPressed = false;

// ODO lock state
private boolean odoLocked = false;
private double lockHeadingRad = 0.0;
private int lockTurretTicks = 0;

private double shooterPosition = 0.5;

	private static final int TURRET_MIN_TICKS = -335;
	private static final int TURRET_MAX_TICKS =  307;

// ODO conversion (start value; can tune later)
	private static final double TURRET_TICKS_PER_RAD = 380.0;

// Limelight aiming
	private static final double LL_DEADZONE_START = 1.6; // deg (hysteresis start moving)
	private static final double LL_DEADZONE_STOP  = 0.9; // deg (hysteresis stop moving)

	private static final double LL_KP_FAST  = 0.020; // power per deg (LL_ONLY)
	private static final double LL_KP_SLOW  = 0.010; // power per deg (ODO micro)
	private static final double LL_MAX_FAST = 0.55;
	private static final double LL_MAX_SLOW = 0.25;

// Hold (ticks) controller for ODO lock
	private static final double HOLD_KP  = 0.005;
	private static final double HOLD_MAX = 0.35;

	private boolean turretAligned = false; // hysteresis state for LL aim
	private boolean prevAutoShooterKeyPressed = false;
	private boolean prevAutoAngleKeyPressed = false;

private boolean autoShooterAngle = true;

	private void UpdateLimelight() {}

	private void Shooter() {
		if (driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY)) {
			robot.outtake1.setIntakeDirection(IntakeDirection.FORWARD);
			robot.outtake2.setIntakeDirection(IntakeDirection.FORWARD);
			transfer_servo = true;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

		}
		else {
			robot.outtake1.setIntakeDirection(IntakeDirection.STOP);
			robot.outtake2.setIntakeDirection(IntakeDirection.STOP);
			transfer_servo = false;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
		}
	}

	// === helper: calculează distanța din Limelight (în cm) ===
	private Double getDistanceCmFromLimelight() {
		if (robot == null || robot.limelight == null) return null;

		LLResult result = robot.limelight.getLatestResult();
		if (result == null || !result.isValid()) return null;

		double ty = result.getTy();

		double limelightMountAngleDegrees = 85.0;
		double limelightLensHeightCm = 55.88;   // 22 inch
		double goalHeightCm = 152.4;            // 60 inch

		double angleToGoalDegrees = limelightMountAngleDegrees + ty;
		double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

		if (Math.abs(angleToGoalRadians) < 0.001) return null;

		double distanceFromLimelightToGoalCm =
				(goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);

		return distanceFromLimelightToGoalCm;
	}

	// === helper: mapare distanță -> poziție servo în CM ===
	private double shooterAngleFromDistanceCm(double distanceCm) {
		double dNear = -1.0;
		double posNear=0.6;


		double dFar = 15.0;
		double posFar = 0.75;

		if (distanceCm <= 15.0 && distanceCm>=8.0) return shooterPosition = posNear;
		else if(distanceCm<=8.0 && distanceCm>=2.0) return shooterPosition=0.7;
		else if(distanceCm<=2.0 && distanceCm>=-1) return shooterPosition=0.7;
		else if (distanceCm >= dFar) return shooterPosition=posFar;
		else return shooterPosition=0.0;


		//	return posNear + (distanceCm - dNear) * (posFar - posNear) / (dFar - dNear);
	}

	private void ShooterAngle() {

		// ==== MOD AUTO (după Limelight / ty) ====
		if (autoShooterAngle) {

			if (robot == null || robot.limelight == null) return;
			LLResult result = robot.limelight.getLatestResult();

			if (result != null && result.isValid()) {
				double ty = result.getTy();

				// calc distanță (FOLOSESC ACELEAȘI VALORI CA ÎN TELEMETRY, să nu mai fie două formule diferite)
				double limelightMountAngleDegrees = 85.0;
				double limelightLensHeightCm = 40.0;   // la fel ca în OnTelemetry
				double goalHeightCm = 105.0;          // la fel ca în OnTelemetry

				double angleToGoalDegrees = limelightMountAngleDegrees + ty;
				double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

				Double distanceCm = null;
				if (Math.abs(angleToGoalRadians) > 0.001) {
					distanceCm = (goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);
				}

				if (distanceCm != null) {
					// mapezi distanța -> poziție servo
					shooterPosition = shooterAngleFromDistanceCm(distanceCm);

					// clamp 0..1
					if (shooterPosition < 0.0) shooterPosition = 0.0;
					if (shooterPosition > 1.0) shooterPosition = 1.0;

					robot.turretTumbler.setPosition(shooterPosition);
				}

				// DEBUG: vezi exact ce se întâmplă
				telemetry.addData("AUTO ty", ty);
				telemetry.addData("AUTO dist", distanceCm);
				telemetry.addData("AUTO shooterPos", shooterPosition);
			} else {
				telemetry.addLine("AUTO: no valid LL result");
			}

			return; // IMPORTANT: nu mai intri în manual dacă e pe auto
		}

		// ==== MOD MANUAL (stick dreapta Y) ====
		double input = -driveInput.getValue(Keybindings.Drive.SHOOTER_ANGLE);

		if (Math.abs(input) < SHOOTER_DEADZONE) return;

		shooterPosition += input * 0.01;
		if (shooterPosition < 0.0) shooterPosition = 0.0;
		if (shooterPosition > 1.0) shooterPosition = 1.0;

		robot.turretTumbler.setPosition(shooterPosition);
	}


	private void Turret() {
	// Manual input (right stick X)
	double stick = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);
	int ticks = robot.turret.getMotor().getCurrentPosition();

	// Safety: cut power if we try to drive into a hard limit
	if (ticks <= TURRET_MIN_TICKS && stick < 0) stick = 0;
	if (ticks >= TURRET_MAX_TICKS && stick > 0) stick = 0;

	// === MANUAL ===
	if (turretMode == TurretAimMode.MANUAL) {
		if (Math.abs(stick) < TURRET_DEADZONE) {
			robot.turret.getMotor().setPower(0);
			return;
		}
		robot.turret.getMotor().setPower(clamp(stick * 0.35, -0.35, 0.35));
		return;
	}

	// Read Limelight
	LLResult result = (robot.limelight != null) ? robot.limelight.getLatestResult() : null;
	boolean hasTarget = (result != null && result.isValid());
	double txDeg = hasTarget ? result.getTx() : 0.0;

	// === LL_ONLY ===
	if (turretMode == TurretAimMode.LL_ONLY) {
		if (!hasTarget) {
			robot.turret.getMotor().setPower(0);
			return;
		}

		// hysteresis on tx
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

		// soft limits
		if (p > 0 && ticks >= TURRET_MAX_TICKS) p = 0;
		if (p < 0 && ticks <= TURRET_MIN_TICKS) p = 0;

		robot.turret.getMotor().setPower(p);
		return;
	}

	// === ODO_LOCK (+ LL micro) ===
	// Because turret has a small range, this mode works best if the driver keeps the robot roughly facing the target.
	if (!odoLocked) {
		lockHeadingRad = robot.drivetrain.pose.heading.log(); // radians
		lockTurretTicks = ticks;
		odoLocked = true;
		turretAligned = false;
	}

	double headingNow = robot.drivetrain.pose.heading.log();
	double dHeading = wrapRad(headingNow - lockHeadingRad);

	int targetTicks = (int) Math.round(lockTurretTicks + dHeading * TURRET_TICKS_PER_RAD);

	// micro-correction from Limelight (slow + gentle)
	if (hasTarget) {
		// small bias in ticks, avoids jitter
		targetTicks += (int) Math.round((-txDeg) * 2.0); // ~2 ticks/deg (tune if needed)
	}

	targetTicks = clampInt(targetTicks, TURRET_MIN_TICKS, TURRET_MAX_TICKS);

	int err = targetTicks - ticks;
	double pHold = clamp(HOLD_KP * err, -HOLD_MAX, HOLD_MAX);

	// soft limits
	if (pHold > 0 && ticks >= TURRET_MAX_TICKS) pHold = 0;
	if (pHold < 0 && ticks <= TURRET_MIN_TICKS) pHold = 0;

	robot.turret.getMotor().setPower(pHold);
}

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




	private void Intake() {
		if (driveInput.isPressed(Keybindings.Drive.INTAKE_KEY)) {
			if(transfer_servo)
				robot.transfer.start();
			robot.intake.setIntakeDirection(IntakeDirection.FORWARD);
		}
		else if (driveInput.isPressed(Keybindings.Drive.INTAKE_REVERSE_KEY)) {
			robot.intake.setIntakeDirection(IntakeDirection.REVERSE);
		}
		else {
			robot.transfer.stop();
			robot.intake.setIntakeDirection(IntakeDirection.STOP);
		}
	}

	double lastTx = -1, lastTy = -1, lastTa = -1;

	@Override
	protected void OnTelemetry(Telemetry telemetry)
	{
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

		double tx = -1, ty = -1, ta = -1;
		double distanceCm = -1;
		boolean hasTarget = false;

		if (result != null && result.isValid())
		{
			tx = result.getTx();
			ty = result.getTy();
			ta = result.getTa();
			hasTarget = true;

			double limelightMountAngleDegrees = 85.0;
			double limelightLensHeightCm = 40.0;
			double goalHeightCm = 105.0;

			double angleToGoalDegrees = limelightMountAngleDegrees + ty;
			double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

			if (Math.abs(angleToGoalRadians) > 0.001)
			{
				distanceCm =
						(goalHeightCm - limelightLensHeightCm) /
								Math.tan(angleToGoalRadians);
			}
			else
			{
				distanceCm = -1;
				hasTarget = false;
			}
		}

		double lastdistanceCm=distanceCm;

		telemetry.addData("LL tx", tx);
		telemetry.addData("LL ty", ty);
		telemetry.addData("LL ta", ta);
		telemetry.addData("LL Last Update", robot.limelight.getTimeSinceLastUpdate());

		if (hasTarget)
			telemetry.addData("LL Distance (cm)", distanceCm);
		else
			telemetry.addData("LL Distance", "NO TARGET");

		telemetry.addData("Auto Shooter Angle", autoShooterAngle ? "ON" : "OFF");
		telemetry.addData("Last LL Distance", lastdistanceCm);

		telemetry.addData("Shooter Pos", shooterPosition);
		telemetry.addData("Turret pos", robot.turret.getMotor().getCurrentPosition());
	}
}*/