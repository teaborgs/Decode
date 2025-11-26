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

@TeleOp(name = "\uD80C\uDC1B\uD83C\uDF46decodeaza-mi-l\uD80C\uDC1B\uD83C\uDF46", group = "TeleOp")
public final class decode extends BaseOpMode {
	private InputSystem driveInput, armInput;

	private RobotHardware robot;

	private static class Keybindings {
		//ramane de vazut cum facem cu driver2 :)
		/*public static class Arm
		{
			public static final InputSystem.Key BASKET_LOW_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key BASKET_HIGH_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key CHAMBER_HIGH_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key PRIMARY_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key SPECIMEN_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key SUSPEND_KEY = new InputSystem.Key("y");
			public static final InputSystem.Key CANCEL_SUSPEND_KEY = new InputSystem.Key("x");
		}*/

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
	}

	@Override
	protected void OnRun() {
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
	private double shooterPosition = 0.5; // trb tunat
	private static final double TURRET_LL_DEADZONE = 1.5;
	;
	private void UpdateLimelight() {

	}

	private void Shooter() {
		if (driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY)) {
			robot.outtake.setIntakeDirection(IntakeDirection.FORWARD);
			transfer_servo = true;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

		}
		else {
			robot.outtake.setIntakeDirection(IntakeDirection.STOP);
			transfer_servo = false;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
		}
	}

	private void ShooterAngle() {
		double input = -driveInput.getValue(Keybindings.Drive.SHOOTER_ANGLE); // invert?

		if (Math.abs(input) < SHOOTER_DEADZONE) return;

		shooterPosition += input * 0.01;              // sensitivity
		shooterPosition = Math.max(0, Math.min(1, shooterPosition));

		robot.turretTumbler.setPosition(shooterPosition);
		LLResult result = robot.limelight.getLatestResult();
		double ty = result.getTy(); // offset pe orizontala
		double targetOffsetAngle_Vertical = result.getTy();


		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = 10.0;

		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightInches = 22.0;

		// distance from the target to the floor
		double goalHeightInches = 60.0;

		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

		//calculate distance
		double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

	}

	private void Turret() {
		double x = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);



		if(!auto_aim){
		if (Math.abs(x) < TURRET_DEADZONE) {
			robot.turret.setIntakeDirection(IntakeDirection.STOP);
			return;
		}
		if (x > 0) {
			robot.turret.setIntakeDirection(IntakeDirection.SLOW_FORWARD);
		}
		else {
			robot.turret.setIntakeDirection(IntakeDirection.SLOW_REVERSE);
		}}
		else if(auto_aim){

			LLResult result = robot.limelight.getLatestResult();
			double tx = result.getTx(); // offset pe orizontala

			if (result == null || !result.isValid()) {
				robot.turret.setIntakeDirection(IntakeDirection.STOP); //opreste tureta daca nu vede apriltag...teoretic
				return;
			}

			if (Math.abs(tx) < TURRET_LL_DEADZONE) {
				// centrat( cu spatiu de eroare )
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
			} else if (tx > 0) {
				// se intoarce spre dreapta
				robot.turret.setIntakeDirection(IntakeDirection.SLOW_FORWARD);
			} else if (tx < 0){
				// se intoarce spre stanga
				robot.turret.setIntakeDirection(IntakeDirection.SLOW_REVERSE);
			}

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

		// --- Preluare date Limelight ---
		LLResult result = robot.limelight.getLatestResult();

		double tx = -1, ty = -1, ta = -1;
		double distanceInches = -1;
		boolean hasTarget = false;

		if (result != null && result.isValid())
		{

			tx = result.getTx();
			ty = result.getTy();
			ta = result.getTa();
			hasTarget = true;

			// --- CALCUL DISTANȚĂ AICI, ÎN TELEMETRY ---
			double limelightMountAngleDegrees = 10.0;
			double limelightLensHeightInches = 22.0;
			double goalHeightInches = 60.0;

			// unghi total spre target
			double angleToGoalDegrees = limelightMountAngleDegrees + ty;
			double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

			// protecție: dacă unghiul e prea mic, nu calculăm
			if (Math.abs(angleToGoalRadians) > 0.001)
			{
				distanceInches =
						(goalHeightInches - limelightLensHeightInches) /
								Math.tan(angleToGoalRadians);
			}
			else
			{
				distanceInches = -1;
				hasTarget = false;
			}
		}

		// --- TELEMETRY OUTPUT ---
		telemetry.addData("LL tx", tx);
		telemetry.addData("LL ty", ty);
		telemetry.addData("LL ta", ta);
		telemetry.addData("LL Last Update", robot.limelight.getTimeSinceLastUpdate());

		if (hasTarget)
			telemetry.addData("LL Distance (in)", distanceInches);
		else
			telemetry.addData("LL Distance", "NO TARGET");
	}}