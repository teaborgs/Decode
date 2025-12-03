package org.firstinspires.ftc.teamcode.teleop;

import android.util.JsonReader;
import android.util.JsonToken;

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
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "\uD80C\uDC1B\uD83C\uDF46decodeaza-mi-l\uD80C\uDC1B\uD83C\uDF46", group = "TeleOp")
public final class decode extends BaseOpMode {
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
		shooterPosition = robot.turretTumbler.getPosition();
		robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		turretTargetDeg = 0.0;

		shooterPosition = robot.turretTumbler.getPosition();
	}

	@Override
	protected void OnRun() {
		boolean llAimPressed = driveInput.isPressed(Keybindings.Drive.LL_AIM);
		if (llAimPressed && !lastLLAimPressed) {
			auto_aim = !auto_aim;
		}
		lastLLAimPressed = llAimPressed;

		if (!autoShooterAngle && driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_ENABLE_KEY)) {
			autoShooterAngle = true;
		}

		if (!autoShooterAngle && driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_ENABLE_KEY)) {
			autoShooterAngle = true;
		}

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
	private boolean lastLLAimPressed = false;
	private static final double TURRET_TICKS_PER_DEG = 0.288;
	private static final double TURRET_MIN_DEG = -120.0;
	private static final double TURRET_MAX_DEG =  120.0;
	private double turretTargetDeg = 0.0;

	private static final double TURRET_LL_DEADZONE = 1.5;

	private boolean autoShooterAngle = false;

	private void UpdateLimelight() {}

	private double getTurretAngleDeg() {
		return robot.turretMotor.getCurrentPosition() / TURRET_TICKS_PER_DEG;
	}

	private void setTurretTargetDeg(double angleDeg) {
		angleDeg = Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, angleDeg));
		turretTargetDeg = angleDeg;

		int targetTicks = (int) Math.round(angleDeg * TURRET_TICKS_PER_DEG);
		robot.turretMotor.setTargetPosition(targetTicks);
		robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.turretMotor.setPower(0.4); // viteza
	}

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
		double dNear = 250.0;
		double posNear=0.5;


		double dFar = 251.0;
		double posFar = 0.60;

		if (distanceCm <= 250.0 && distanceCm>=160.0) return shooterPosition=posNear;
		if(distanceCm<=160.0 && distanceCm>=100.0) return shooterPosition=0.4;
		if(distanceCm<=100.0 && distanceCm>=60) return shooterPosition=0.2;
		if(distanceCm<=60.0) return shooterPosition=0.0;
		if (distanceCm >= dFar) return shooterPosition=posFar;

		return posNear + (distanceCm - dNear) * (posFar - posNear) / (dFar - dNear);
	}

	private void ShooterAngle() {

		if (autoShooterAngle) {
			Double distanceCm = getDistanceCmFromLimelight();
			if (distanceCm == null) return;

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

	private static final double TURRET_STICK_DEG_PER_LOOP = 2.0;
	private static final double TURRET_KP_TX = 1.0;

	private void Turret() {
		double x = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);

		// === MANUAL MODE ===
		if (!auto_aim) {
			double currentDeg = getTurretAngleDeg();

			if (Math.abs(x) < TURRET_DEADZONE) {
				setTurretTargetDeg(currentDeg);
				return;
			}

			double stepDeg = x * TURRET_STICK_DEG_PER_LOOP;
			setTurretTargetDeg(currentDeg + stepDeg);
			return;
		}

		// === AUTO AIM ===
		LLResult result = robot.limelight.getLatestResult();
		if (result == null || !result.isValid()) {
			// no target
			setTurretTargetDeg(getTurretAngleDeg());
			return;
		}

		double tx = result.getTx();

		if (Math.abs(tx) < TURRET_LL_DEADZONE) {
			// margin of error
			setTurretTargetDeg(getTurretAngleDeg());
			return;
		}

		double currentAngle = getTurretAngleDeg();
		double desiredAngle = currentAngle + TURRET_KP_TX * tx; // TODO:VERIFICA DACA - SAU +

		setTurretTargetDeg(desiredAngle);
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
			double limelightLensHeightCm = 45.0;
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

		telemetry.addData("LL tx", tx);
		telemetry.addData("LL ty", ty);
		telemetry.addData("LL ta", ta);
		telemetry.addData("LL Last Update", robot.limelight.getTimeSinceLastUpdate());

		if (hasTarget)
			telemetry.addData("LL Distance (cm)", distanceCm);
		else
			telemetry.addData("LL Distance", "NO TARGET");

		telemetry.addData("Auto Shooter Angle", autoShooterAngle ? "ON" : "OFF");
		telemetry.addData("Shooter Pos", shooterPosition);
	}
}
