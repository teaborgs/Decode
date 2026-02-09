package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@TeleOp(name = "🔴🔴decodeaza-mi-l.🔴🔴", group = "TeleOp")
public final class decodeRED extends BaseOpMode {
	private InputSystem driveInput, armInput;

	private RobotHardware robot;

	private static class Keybindings {
		public static class Drive {
			public static final InputSystem.Key SHOOTER_KEY = new InputSystem.Key("a"); // hold = shoot
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

		// shooter rpm config
		OuttakeSystem.TICKS_PER_REV = 28;

		// limelight
		robot.limelight.pipelineSwitch(1);

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
			autoShooterAngle = !autoShooterAngle;
		}
		prevAutoShooterKeyPressed = autoShooterKey;

		// === TOGGLE auto_aim with DPAD_UP ===
		boolean llAimPressed = driveInput.isPressed(Keybindings.Drive.LL_AIM);
		if (llAimPressed && !prevLlAimPressed) {
			auto_aim = !auto_aim;
			turretAligned = false;
		}
		prevLlAimPressed = llAimPressed;

		boolean autoAngleKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_DISABLE_KEY);
		if (autoAngleKey && !prevAutoAngleKeyPressed) {
			autoShooterAngle = !autoShooterAngle;
		}
		prevAutoAngleKeyPressed = autoAngleKey;

		Drive();
		Shooter();       // RPM bazat pe distanță/poziție
		Turret();
		ShooterAngle();  // aici se calculează distanța și se setează și rpm-ul
		Intake();
		UpdateLimelight();
	}

	// ================= DRIVE =================
	private void Drive() {
		float speed = 1f;
		if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;

		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(
						new Vector2d(
								driveInput.getValue(Keybindings.Drive.DRIVE_Y),
								driveInput.getValue(Keybindings.Drive.DRIVE_X)
						).times(-speed),
						(driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
				)
		);
	}

	// ================= SHOOTER / OUTTAKE RPM =================
	private boolean transfer_servo = false;

	// target din distanță (default)
	private double shooterTargetRpm = 4500;

	// cerința ta:
	private final double shooterTargetRpmNear = 4000; // posNear
	private final double shooterTargetRpmFar  = 4800; // posFar

	private void Shooter() {
		// HOLD A = shoot
		if (driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY)) {
			// NU mai forțăm aici un preset. shooterTargetRpm e setat de ShooterAngle() (auto).
			robot.outtake1.setRpm(shooterTargetRpm);
			robot.outtake2.setRpm(shooterTargetRpm);

			transfer_servo = true;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		} else {
			robot.outtake1.stop();
			robot.outtake2.stop();

			transfer_servo = false;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
		}
	}

	// ================= TURRET =================
	private static final double TURRET_DEADZONE = 0.12;
	private boolean auto_aim = true;

	private static final double TURRET_LL_DEADZONE_STOP  = 2.0;
	private static final double TURRET_LL_DEADZONE_START = 3.0;

	private boolean turretAligned = false;
	private boolean prevLlAimPressed = false;

	private void Turret() {
		double x = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);

		// MANUAL MODE
		if (!auto_aim) {
			if (Math.abs(x) < TURRET_DEADZONE) {
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
				return;
			}
			robot.turret.setIntakeDirection(x > 0 ? IntakeDirection.SLOW_FORWARD : IntakeDirection.SLOW_REVERSE);
			return;
		}

		// AUTO AIM MODE
		LLResult result = robot.limelight.getLatestResult();
		if (result == null || !result.isValid()) {
			robot.turret.setIntakeDirection(IntakeDirection.STOP);
			return;
		}

		double tx = result.getTx();

		// hysteresis
		if (turretAligned) {
			if (Math.abs(tx) > TURRET_LL_DEADZONE_START) {
				turretAligned = false;
			} else {
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
				return;
			}
		} else {
			if (Math.abs(tx) < TURRET_LL_DEADZONE_STOP) {
				turretAligned = true;
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
				return;
			}
		}

		if (tx > 1) robot.turret.setIntakeDirection(IntakeDirection.SLOW_FORWARD);
		else if (tx < 1) robot.turret.setIntakeDirection(IntakeDirection.SLOW_REVERSE);
	}

	// ================= SHOOTER ANGLE =================
	private static final double SHOOTER_DEADZONE = 0.15;
	private double shooterPosition = 0.5;

	private boolean autoShooterAngle = true;
	private boolean prevAutoShooterKeyPressed = false;
	private boolean prevAutoAngleKeyPressed = false;

	private void UpdateLimelight() {}

	/**
	 * MAP distanță -> poziție servo + RPM:
	 *  - posNear -> 4000 rpm
	 *  - posFar  -> 4800 rpm
	 *
	 * NOTĂ: condițiile tale se suprapun (>=12 e și în intervalul 8..14).
	 * Eu păstrez ordinea ta: dacă e în 8..14 => near, altfel dacă e >=12 => far.
	 */
	private double shooterAngleFromDistanceCm(double distanceCm) {
		double posNear = 0.54;
		double posFar  = 0.47;

		// FAR: 12+ (prima condiție, ca să nu fie mâncată de near)
		if (distanceCm >= 12.0) {
			shooterTargetRpm = shooterTargetRpmFar; // 4800
			return shooterPosition = posFar;
		}

		// NEAR: 8..12
		if (distanceCm >= 7.0) {
			shooterTargetRpm = shooterTargetRpmNear; // 4000
			return shooterPosition = posNear;
		}

		// restul cazurilor tale (aproape de hub / foarte aproape)
		if (distanceCm >= 4.0) {
			shooterTargetRpm = shooterTargetRpmNear;
			return shooterPosition = 0.54;
		}

		if (distanceCm >= -1.0) {
			shooterTargetRpm = shooterTargetRpmNear;
			return shooterPosition = 0.54;
		}

		shooterTargetRpm = 4500; // fallback
		return shooterPosition = 0.75;
	}


	private void ShooterAngle() {
		if (autoShooterAngle) {
			LLResult result = robot.limelight.getLatestResult();
			if (result == null || !result.isValid()) return;

			double ty = result.getTy();

			double limelightMountAngleDegrees = 85.0;
			double limelightLensHeightCm = 40.0;
			double goalHeightCm = 105.0;

			double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + ty);
			if (Math.abs(angleToGoalRadians) < 0.001) return;

			double distanceCm = (goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);

			// setează și poziție și shooterTargetRpm
			shooterPosition = shooterAngleFromDistanceCm(distanceCm);

			if (shooterPosition < 0.0) shooterPosition = 0.0;
			if (shooterPosition > 1.0) shooterPosition = 1.0;

			robot.turretTumbler.setPosition(shooterPosition);
			return;
		}

		// manual
		double input = -driveInput.getValue(Keybindings.Drive.SHOOTER_ANGLE);
		if (Math.abs(input) < SHOOTER_DEADZONE) return;

		shooterPosition += input * 0.01;
		if (shooterPosition < 0.0) shooterPosition = 0.0;
		if (shooterPosition > 1.0) shooterPosition = 1.0;

		robot.turretTumbler.setPosition(shooterPosition);
	}

	// ================= INTAKE =================
	private void Intake() {

		if (driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY)
				&& driveInput.isPressed(Keybindings.Drive.INTAKE_KEY)) {

			if (transfer_servo) robot.transfer.start();
			robot.intake.setIntakeDirection(IntakeDirection.FORWARD_SHOOT);
		}
		else if (driveInput.isPressed(Keybindings.Drive.INTAKE_KEY)) {
			if (transfer_servo) robot.transfer.start();
			robot.intake.setIntakeDirection(IntakeDirection.FORWARD);

		} else if (driveInput.isPressed(Keybindings.Drive.INTAKE_REVERSE_KEY)) {
			robot.intake.setIntakeDirection(IntakeDirection.REVERSE);
		} else {
			robot.transfer.stop();
			robot.intake.setIntakeDirection(IntakeDirection.STOP);
		}
	}

	// ================= TELEMETRY =================
	@Override
	protected void OnTelemetry(Telemetry telemetry) {
		super.OnTelemetry(telemetry);

		if (robot != null && robot.drivetrain != null) {
			Pose2d p = robot.drivetrain.pose;
			telemetry.addData("ODO X (in)", p.position.x);
			telemetry.addData("ODO Y (in)", p.position.y);
			telemetry.addData("ODO H (deg)", Math.toDegrees(p.heading.log()));
		}

		// outtake rpm
		if (robot != null) {
			double r1 = robot.outtake1.getRpm();
			double r2 = robot.outtake2.getRpm();
			telemetry.addData("Shooter TargetRPM", (int) shooterTargetRpm);
			telemetry.addData("OT1 RPM", (int) r1);
			telemetry.addData("OT2 RPM", (int) r2);
			telemetry.addData("AVG RPM", (int) ((r1 + r2) / 2.0));
		}

		if (robot == null || robot.limelight == null) {
			telemetry.addLine("Robot sau Limelight NULL");
			return;
		}

		LLResult result = robot.limelight.getLatestResult();

		double tx = -1, ty = -1, ta = -1;
		double distanceCm = -1;
		boolean hasTarget = false;

		if (result != null && result.isValid()) {
			tx = result.getTx();
			ty = result.getTy();
			ta = result.getTa();
			hasTarget = true;

			double limelightMountAngleDegrees = 85.0;
			double limelightLensHeightCm = 40.0;
			double goalHeightCm = 105.0;

			double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + ty);
			if (Math.abs(angleToGoalRadians) > 0.001) {
				distanceCm = (goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);
			} else {
				hasTarget = false;
			}
		}

		telemetry.addData("LL tx", tx);
		telemetry.addData("LL ty", ty);
		telemetry.addData("LL ta", ta);
		telemetry.addData("LL Last Update", robot.limelight.getTimeSinceLastUpdate());

		telemetry.addData("LL Distance", hasTarget ? distanceCm : "NO TARGET");

		telemetry.addData("Auto Shooter Angle", autoShooterAngle ? "ON" : "OFF");
		telemetry.addData("Turret Mode", auto_aim ? "AUTO" : "MANUAL");
		telemetry.addData("Shooter Pos", shooterPosition);
	}
}
