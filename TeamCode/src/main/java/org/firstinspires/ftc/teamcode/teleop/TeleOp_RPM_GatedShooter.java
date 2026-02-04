package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

/**
 * IMPORTANT CHANGE:
 * - NU mai folosim "RPM real" (ca la tine raporteaza ~57 si nu e util).
 * - Folosim direct unitatile native de velocity ale SDK (ticks/sec) si setVelocity(ticks/sec).
 *
 * Ce face:
 * - cand confirmi SHOOTER_KEY (A): porneste flywheel la un target (din distanta), in ticks/sec
 * - intakeStopper e MEREU deschis (TRANSFER) cat timp shooter e pornit
 * - transfer + intake (ground intake) NU merg daca shooter e pornit si nu esti "ready"
 * - intake merge normal cand shooter NU e pornit (ca sa iei bile de pe jos)
 */
@TeleOp(name = "TeleOp_RPM_GatedShooter", group = "TeleOp")
public final class TeleOp_RPM_GatedShooter extends BaseOpMode {

	private InputSystem driveInput, armInput;
	private RobotHardware robot;

	// ======== Keybinds (ca la tine) ========
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
			public static final InputSystem.Key AUTO_SHOOTER_ENABLE_KEY = new InputSystem.Key("start");
			public static final InputSystem.Key AUTO_SHOOTER_DISABLE_KEY = new InputSystem.Key("back");
		}
	}

	// ======== Flywheels ========
	private DcMotorEx fly1, fly2;

	// Max velocity (ticks/sec) estimat de SDK (achievable). Il folosim ca referinta.
	private double maxTps = 0;

	// READY window (in procente din maxTps) + histerezis
	private static final double READY_IN_PCT  = 0.015; // 1.5% sub target -> devii ready
	private static final double READY_OUT_PCT = 0.030; // 3.0% sub target -> cazi din ready

	private static final long MIN_MS_BETWEEN_FEEDS = 150;

	private boolean shooterReady = false;
	private long lastFeedMs = 0;

	// ======== Shooter angle (servo) ========
	private static final double SHOOTER_DEADZONE = 0.15;
	private double shooterPosition = 0.5;
	private boolean autoShooterAngle = true;
	private boolean prevAutoShooterKeyPressed = false;
	private boolean prevAutoAngleKeyPressed = false;

	// ======== Turret aim ========
	private static final double TURRET_DEADZONE = 0.12;
	private boolean auto_aim = true;
	private boolean turretAligned = false;
	private boolean prevLlAimPressed = false;

	private static final double TURRET_LL_DEADZONE_STOP  = 2.0;
	private static final double TURRET_LL_DEADZONE_START = 3.0;

	// ======== Limelight distance (UNIFICATA) ========
	private static final double LL_MOUNT_ANGLE_DEG = 85.0;
	private static final double LL_LENS_HEIGHT_CM  = 40.0;
	private static final double GOAL_HEIGHT_CM     = 105.0;

	// ======== Target din distanta: procent din maxTps ========
	// AICI e cheia: nu mai e 3000 rpm etc. E procent (0..1).
	// Ajustezi 2-3 valori si iti va bate imediat.
	private static class ShotPoint {
		final double dCm;
		final double pct; // 0..1 din maxTps
		ShotPoint(double dCm, double pct) { this.dCm = dCm; this.pct = pct; }
	}

	private static final ShotPoint[] VEL_MAP = new ShotPoint[] {
			new ShotPoint(180, 0.55),
			new ShotPoint(230, 0.58),
			new ShotPoint(280, 0.62),
			new ShotPoint(330, 0.66),
			new ShotPoint(380, 0.70),
			new ShotPoint(430, 0.74),
	};

	private static double pctFromDistance(double dCm) {
		if (dCm <= VEL_MAP[0].dCm) return VEL_MAP[0].pct;
		for (int i = 0; i < VEL_MAP.length - 1; i++) {
			ShotPoint a = VEL_MAP[i], b = VEL_MAP[i + 1];
			if (dCm <= b.dCm) {
				double t = (dCm - a.dCm) / (b.dCm - a.dCm);
				return a.pct + t * (b.pct - a.pct);
			}
		}
		return VEL_MAP[VEL_MAP.length - 1].pct;
	}

	// ======== Lifecycle ========
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

		fly1 = robot.outtake1.getMotor();
		fly2 = robot.outtake2.getMotor();

		fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// ia max ticks/sec din SDK; folosim minimul ca referinta comuna
		double m1 = fly1.getMotorType().getAchieveableMaxTicksPerSecond();
		double m2 = fly2.getMotorType().getAchieveableMaxTicksPerSecond();
		maxTps = Math.max(1.0, Math.min(m1, m2));
	}

	@Override
	protected void OnRun() {
		if (robot != null) robot.drivetrain.updatePoseEstimate();

		// toggle auto shooter angle (START/BACK)
		boolean autoShooterKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_ENABLE_KEY);
		if (autoShooterKey && !prevAutoShooterKeyPressed) autoShooterAngle = !autoShooterAngle;
		prevAutoShooterKeyPressed = autoShooterKey;

		boolean autoAngleKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_DISABLE_KEY);
		if (autoAngleKey && !prevAutoAngleKeyPressed) autoShooterAngle = !autoShooterAngle;
		prevAutoAngleKeyPressed = autoAngleKey;

		// toggle auto aim
		boolean llAimPressed = driveInput.isPressed(Keybindings.Drive.LL_AIM);
		if (llAimPressed && !prevLlAimPressed) {
			auto_aim = !auto_aim;
			turretAligned = false;
		}
		prevLlAimPressed = llAimPressed;

		Drive();
		ShooterVelocityAndFeed(); // aici e logica noua
		Turret();
		ShooterAngle();
		Intake(); // intake e gated aici in functie de shooterReady
	}

	// ======== Drive ========
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

	// ======== Limelight distance (cm) ========
	private Double getDistanceCmFromLimelight() {
		if (robot == null || robot.limelight == null) return null;
		LLResult result = robot.limelight.getLatestResult();
		if (result == null || !result.isValid()) return null;

		double ty = result.getTy();
		double angleDeg = LL_MOUNT_ANGLE_DEG + ty;
		double angleRad = Math.toRadians(angleDeg);
		if (Math.abs(angleRad) < 1e-3) return null;

		return (GOAL_HEIGHT_CM - LL_LENS_HEIGHT_CM) / Math.tan(angleRad);
	}

	// ======== Velocity helpers (ticks/sec) ========
	private double getTps(DcMotorEx m) {
		// raw ticks/sec
		return m.getVelocity();
	}

	private void setTps(DcMotorEx m, double tps) {
		if (tps <= 0) {
			m.setPower(0);
			return;
		}
		m.setVelocity(tps);
	}

	private boolean computeReady(double tpsNow, double tpsTarget) {
		if (tpsTarget < 1) {
			shooterReady = false;
			return false;
		}

		double in  = tpsTarget - (READY_IN_PCT  * maxTps);
		double out = tpsTarget - (READY_OUT_PCT * maxTps);

		if (shooterReady) {
			if (tpsNow < out) shooterReady = false;
		} else {
			if (tpsNow >= in) shooterReady = true;
		}
		return shooterReady;
	}

	// ======== Shooter: seteaza viteza + feed gating ========
	private boolean shooterOn = false;
	private double targetTps = 0;

	private void ShooterVelocityAndFeed() {
		shooterOn = driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY);

		if (!shooterOn) {
			targetTps = 0;
			setTps(fly1, 0);
			setTps(fly2, 0);

			shooterReady = false;
			robot.transfer.stop();
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
			return;
		}

		// intakeStopper MEREU deschis cand shooter e pornit
		robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

		Double d = getDistanceCmFromLimelight();
		if (d == null) d = 300.0;
		d = Range.clip(d, 120.0, 520.0);

		double pct = pctFromDistance(d);
		pct = Range.clip(pct, 0.0, 1.0);

		targetTps = pct * maxTps;
		// siguranta: daca maxTps raportat e dubios, tot nu lasam target mic
		targetTps = Math.max(1.0, targetTps);

		setTps(fly1, targetTps);
		setTps(fly2, targetTps);

		double nowTps = (getTps(fly1) + getTps(fly2)) * 0.5;
		boolean ready = computeReady(nowTps, targetTps);

		long nowMs = System.currentTimeMillis();
		boolean cadenceOk = (nowMs - lastFeedMs) >= MIN_MS_BETWEEN_FEEDS;

		// Transferul NU merge daca nu esti ready
		if (ready && cadenceOk) {
			robot.transfer.start();
			lastFeedMs = nowMs;
		} else {
			robot.transfer.stop();
		}

		telemetry.addData("LL dist(cm)", d);
		telemetry.addData("maxTps(sdk)", maxTps);
		telemetry.addData("targetPct", pct);
		telemetry.addData("targetTps", targetTps);
		telemetry.addData("nowTps(avg)", nowTps);
		telemetry.addData("ready", ready);
	}

	// ======== Shooter angle ========
	private double shooterAngleFromDistanceCm(double distanceCm) {
		if (distanceCm <= 180) return shooterPosition = 0.90;
		if (distanceCm <= 240) return shooterPosition = 0.80;
		if (distanceCm <= 320) return shooterPosition = 0.72;
		if (distanceCm <= 400) return shooterPosition = 0.66;
		return shooterPosition = 0.60;
	}

	private void ShooterAngle() {
		if (autoShooterAngle) {
			Double d = getDistanceCmFromLimelight();
			if (d == null) return;

			d = Range.clip(d, 120.0, 520.0);
			shooterPosition = shooterAngleFromDistanceCm(d);
			shooterPosition = Range.clip(shooterPosition, 0.0, 1.0);
			robot.turretTumbler.setPosition(shooterPosition);
			return;
		}

		double input = -driveInput.getValue(Keybindings.Drive.SHOOTER_ANGLE);
		if (Math.abs(input) < SHOOTER_DEADZONE) return;

		shooterPosition += input * 0.01;
		shooterPosition = Range.clip(shooterPosition, 0.0, 1.0);
		robot.turretTumbler.setPosition(shooterPosition);
	}

	// ======== Turret ========
	private void Turret() {
		double x = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);

		if (!auto_aim) {
			if (Math.abs(x) < TURRET_DEADZONE) {
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
				return;
			}
			robot.turret.setIntakeDirection(x > 0 ? IntakeDirection.SLOW_FORWARD : IntakeDirection.SLOW_REVERSE);
			return;
		}

		LLResult result = robot.limelight.getLatestResult();
		if (result == null || !result.isValid()) {
			robot.turret.setIntakeDirection(IntakeDirection.STOP);
			return;
		}

		double tx = result.getTx();

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

		if (tx > 0) robot.turret.setIntakeDirection(IntakeDirection.SLOW_FORWARD);
		else robot.turret.setIntakeDirection(IntakeDirection.SLOW_REVERSE);
	}

	// ======== Intake ========
	// Cerinta ta:
	// - daca shooter NU e pornit: intake merge normal (iei bile)
	// - daca shooter e pornit: intake NU merge daca nu esti ready (ca sa nu bage bila cand flywheel nu e la target)
	private void Intake() {
		boolean intakeFwd = driveInput.isPressed(Keybindings.Drive.INTAKE_KEY);
		boolean intakeRev = driveInput.isPressed(Keybindings.Drive.INTAKE_REVERSE_KEY);

		if (!shooterOn) {
			if (intakeFwd) robot.intake.setIntakeDirection(IntakeDirection.FORWARD);
			else if (intakeRev) robot.intake.setIntakeDirection(IntakeDirection.REVERSE);
			else robot.intake.setIntakeDirection(IntakeDirection.STOP);
			return;
		}

		// shooter e ON
		if (!shooterReady) {
			robot.intake.setIntakeDirection(IntakeDirection.STOP);
			return;
		}

		// shooter ON + READY => permit intake
		if (intakeFwd) robot.intake.setIntakeDirection(IntakeDirection.FORWARD);
		else if (intakeRev) robot.intake.setIntakeDirection(IntakeDirection.REVERSE);
		else robot.intake.setIntakeDirection(IntakeDirection.STOP);
	}

	// ======== Telemetry ========
	@Override
	protected void OnTelemetry(Telemetry telemetry) {
		super.OnTelemetry(telemetry);

		if (robot != null && robot.drivetrain != null) {
			Pose2d p = robot.drivetrain.pose;
			telemetry.addData("ODO X (in)", p.position.x);
			telemetry.addData("ODO Y (in)", p.position.y);
			telemetry.addData("ODO H (deg)", Math.toDegrees(p.heading.log()));
		}

		Double d = getDistanceCmFromLimelight();
		telemetry.addData("LL dist(cm)", d == null ? "NO TARGET" : d);

		double tps1 = fly1 != null ? getTps(fly1) : 0;
		double tps2 = fly2 != null ? getTps(fly2) : 0;

		telemetry.addData("Fly1 tps", tps1);
		telemetry.addData("Fly2 tps", tps2);
		telemetry.addData("Target tps", targetTps);
		telemetry.addData("maxTps(sdk)", maxTps);
		telemetry.addData("ShooterOn", shooterOn);
		telemetry.addData("Ready", shooterReady);

		telemetry.addData("Auto Shooter Angle", autoShooterAngle ? "ON" : "OFF");
		telemetry.addData("Shooter Pos", shooterPosition);
		telemetry.addData("Turret Mode", auto_aim ? "AUTO" : "MANUAL");
		telemetry.addData("Turret enc", robot.turret.getMotor().getCurrentPosition());
	}
}