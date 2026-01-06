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
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@TeleOp(name = "\uD80C\uDC1B\uD83C\uDF46decodeaza-mi-l rosu\uD80C\uDC1B\uD83C\uDF46", group = "TeleOp")
public final class decodeRED extends BaseOpMode {
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
			public static final InputSystem.Key EXTENDO_HALF_KEY = new InputSystem.Key("dpad_left");
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
			autoShooterAngle = !autoShooterAngle;  // toggle
		}
		prevAutoShooterKeyPressed = autoShooterKey;

		// === TOGGLE auto_aim with DPAD_UP ===
		boolean llAimPressed = driveInput.isPressed(Keybindings.Drive.LL_AIM);

		if (llAimPressed && !prevLlAimPressed) {
			auto_aim = !auto_aim;  // toggle
			turretAligned = false; // when switching modes, reset lock state
		}
		prevLlAimPressed = llAimPressed;

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
	private boolean auto_aim = true;
	private double shooterPosition = 0.5;

	private static final double TURRET_LL_DEADZONE_STOP  = 2.0; // when to STOP moving
	private static final double TURRET_LL_DEADZONE_START = 3.0; // when to START moving again

	private boolean turretAligned = false; // are we currently "locked on"?
	private boolean prevLlAimPressed = false;
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
		double x = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);

		// === MANUAL MODE ===
		if (!auto_aim) {
			if (Math.abs(x) < TURRET_DEADZONE) {
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
				return;
			}
			robot.turret.setIntakeDirection(x > 0 ? IntakeDirection.SLOW_FORWARD : IntakeDirection.SLOW_REVERSE);
			return;
		}

		// === AUTO AIM MODE ===
		LLResult result = robot.limelight.getLatestResult();
		if (result == null || !result.isValid()) {
			robot.turret.setIntakeDirection(IntakeDirection.STOP);
			return;
		}

		double tx = result.getTx(); // offset orizontal fata de target

		// --- HYSTERESIS LOGIC ---
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

		// --- MOVE TURRET ---
		if (tx > 0) {
			robot.turret.setIntakeDirection(IntakeDirection.SLOW_FORWARD);
		} else if (tx <  0){
			robot.turret.setIntakeDirection(IntakeDirection.SLOW_REVERSE);
		}
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
		telemetry.addData("Turret Mode", auto_aim ? "AUTO" : "MANUAL");
		telemetry.addData("Shooter Pos", shooterPosition);
		telemetry.addData("Turret pos", robot.turret.getMotor().getCurrentPosition());
	}
}