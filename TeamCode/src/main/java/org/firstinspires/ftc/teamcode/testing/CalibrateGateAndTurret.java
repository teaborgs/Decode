package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

@TeleOp(name = "🧪Calibrate Gate + Turret🧪", group = "TeleOp")
public final class CalibrateGateAndTurret extends BaseOpMode {

	private InputSystem driveInput, armInput;
	private RobotHardware robot;

	// ===== Buttons =====
	private static final InputSystem.Key KEY_RESET_GATE   = new InputSystem.Key("dpad_left");
	private static final InputSystem.Key KEY_SAVE_BASKET   = new InputSystem.Key("dpad_right");
	private static final InputSystem.Key KEY_SET_TMIN      = new InputSystem.Key("x");
	private static final InputSystem.Key KEY_SET_TMAX      = new InputSystem.Key("y");
	private static final InputSystem.Key KEY_RESET_TURRET  = new InputSystem.Key("dpad_down");
	private static final InputSystem.Key KEY_WRITE_FILE    = new InputSystem.Key("start");

	// Manual turret move for calibration
	private static final InputSystem.Axis AX_TURRET_MANUAL = new InputSystem.Axis("right_stick_x");

	// Drive
	private static final InputSystem.Axis AX_DRIVE_X = new InputSystem.Axis("left_stick_x");
	private static final InputSystem.Axis AX_DRIVE_Y = new InputSystem.Axis("left_stick_y");
	private static final InputSystem.Axis AX_ROT_L   = new InputSystem.Axis("left_trigger");
	private static final InputSystem.Axis AX_ROT_R   = new InputSystem.Axis("right_trigger");
	private static final InputSystem.Key KEY_SUPPRESS = new InputSystem.Key("left_bumper");

	// ===== Gate reset pose (definești tu 0,0,0 la gate) =====
	private static final double GATE_X_IN = 0.0;
	private static final double GATE_Y_IN = 0.0;
	private static final double GATE_H_DEG = 0.0;

	// ===== Turret assumptions =====
	// tureta are ~±170° => ~340° total
	private static final double TURRET_TOTAL_RANGE_DEG = 340.0;

	// ===== Saved calibration values =====
	private double basketX = 60.0;  // fallback până salvezi
	private double basketY = 30.0;

	private int turretMinTicks = -5000; // fallback până calibrezi
	private int turretMaxTicks =  5000;

	private double degPerTick = 0.05;  // fallback până calibrezi (va fi recalculat)

	private boolean prevResetGate = false;
	private boolean prevSaveBasket = false;
	private boolean prevSetMin = false;
	private boolean prevSetMax = false;
	private boolean prevResetTurret = false;
	private boolean prevWriteFile = false;

	private static final String CALIB_PATH = "/sdcard/FIRST/turret_aim_calib.txt";

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

		// load existing calib if exists
		loadCalibIfExists();

		// Optional: dacă pui tureta fizic în HOME înainte de start
		resetTurretEncoder();

		// start from gate as default
		setPoseInches(GATE_X_IN, GATE_Y_IN, GATE_H_DEG);
	}

	@Override
	protected void OnRun() {
		robot.drivetrain.updatePoseEstimate();

		// ===== drive =====
		Drive();

		// ===== manual turret move (calib) =====
		manualTurretMove();

		// ===== buttons =====
		boolean resetGate = driveInput.isPressed(KEY_RESET_GATE);
		if (resetGate && !prevResetGate) {
			setPoseInches(GATE_X_IN, GATE_Y_IN, GATE_H_DEG);
		}
		prevResetGate = resetGate;

		boolean saveBasket = driveInput.isPressed(KEY_SAVE_BASKET);
		if (saveBasket && !prevSaveBasket) {
			Pose2d p = robot.drivetrain.pose;
			basketX = p.position.x;
			basketY = p.position.y;
		}
		prevSaveBasket = saveBasket;

		boolean setMin = driveInput.isPressed(KEY_SET_TMIN);
		if (setMin && !prevSetMin) {
			turretMinTicks = robot.turret.getMotor().getCurrentPosition();
			recomputeDegPerTick();
		}
		prevSetMin = setMin;

		boolean setMax = driveInput.isPressed(KEY_SET_TMAX);
		if (setMax && !prevSetMax) {
			turretMaxTicks = robot.turret.getMotor().getCurrentPosition();
			recomputeDegPerTick();
		}
		prevSetMax = setMax;

		boolean resetTurret = driveInput.isPressed(KEY_RESET_TURRET);
		if (resetTurret && !prevResetTurret) {
			resetTurretEncoder();
		}
		prevResetTurret = resetTurret;

		boolean writeFile = driveInput.isPressed(KEY_WRITE_FILE);
		if (writeFile && !prevWriteFile) {
			writeCalibToFile();
		}
		prevWriteFile = writeFile;
	}

	private void Drive() {
		float speed = driveInput.isPressed(KEY_SUPPRESS) ? 0.4f : 1f;

		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(
						new Vector2d(
								driveInput.getValue(AX_DRIVE_Y),
								driveInput.getValue(AX_DRIVE_X)
						).times(-speed),
						(driveInput.getValue(AX_ROT_L) - driveInput.getValue(AX_ROT_R)) * speed
				)
		);
	}

	private void manualTurretMove() {
		double x = driveInput.getValue(AX_TURRET_MANUAL);
		double dead = 0.10;
		if (Math.abs(x) < dead) {
			robot.turret.getMotor().setPower(0);
			return;
		}
		// mișcare încetă pentru calibrare
		robot.turret.getMotor().setPower(-x * 0.25);
	}

	private void setPoseInches(double xIn, double yIn, double headingDeg) {
		robot.drivetrain.pose = new Pose2d(new Vector2d(xIn, yIn), Math.toRadians(headingDeg));
	}

	private void resetTurretEncoder() {
		DcMotor m = robot.turret.getMotor();
		m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	private void recomputeDegPerTick() {
		int span = Math.abs(turretMaxTicks - turretMinTicks);
		if (span < 100) return; // prea mic => calib incomplet
		degPerTick = TURRET_TOTAL_RANGE_DEG / (double) span;
	}

	private void writeCalibToFile() {
		try {
			File f = new File(CALIB_PATH);
			f.getParentFile().mkdirs();

			FileWriter w = new FileWriter(f, false);
			// format simplu key=value
			w.write("basketX=" + basketX + "\n");
			w.write("basketY=" + basketY + "\n");
			w.write("turretMinTicks=" + turretMinTicks + "\n");
			w.write("turretMaxTicks=" + turretMaxTicks + "\n");
			w.write("degPerTick=" + degPerTick + "\n");
			w.flush();
			w.close();
		} catch (Exception ignored) {}
	}

	private void loadCalibIfExists() {
		try {
			File f = new File(CALIB_PATH);
			if (!f.exists()) return;

			BufferedReader br = new BufferedReader(new FileReader(f));
			String line;
			while ((line = br.readLine()) != null) {
				String[] parts = line.split("=");
				if (parts.length != 2) continue;
				String k = parts[0].trim();
				String v = parts[1].trim();

				if (k.equals("basketX")) basketX = Double.parseDouble(v);
				else if (k.equals("basketY")) basketY = Double.parseDouble(v);
				else if (k.equals("turretMinTicks")) turretMinTicks = Integer.parseInt(v);
				else if (k.equals("turretMaxTicks")) turretMaxTicks = Integer.parseInt(v);
				else if (k.equals("degPerTick")) degPerTick = Double.parseDouble(v);
			}
			br.close();
		} catch (Exception ignored) {}
	}

	@Override
	protected void OnTelemetry(Telemetry telemetry) {
		super.OnTelemetry(telemetry);

		Pose2d p = robot.drivetrain.pose;
		telemetry.addLine("=== GATE CALIB ===");
		telemetry.addData("ResetGate", "DPAD_LEFT  -> setPose(0,0,0)");
		telemetry.addData("SaveBasket", "DPAD_RIGHT -> save current pose as basket");
		telemetry.addData("Set Turret MIN", "X (at left limit)");
		telemetry.addData("Set Turret MAX", "Y (at right limit)");
		telemetry.addData("Reset Turret Enc", "DPAD_DOWN (when in HOME)");
		telemetry.addData("Write file", "START -> " + CALIB_PATH);

		telemetry.addLine(" ");
		telemetry.addData("ODO X(in)", p.position.x);
		telemetry.addData("ODO Y(in)", p.position.y);
		telemetry.addData("ODO H(deg)", Math.toDegrees(p.heading.log()));

		telemetry.addLine(" ");
		telemetry.addData("Saved basketX", basketX);
		telemetry.addData("Saved basketY", basketY);

		int ticks = robot.turret.getMotor().getCurrentPosition();
		telemetry.addLine(" ");
		telemetry.addData("Turret ticks", ticks);
		telemetry.addData("TurretMinTicks", turretMinTicks);
		telemetry.addData("TurretMaxTicks", turretMaxTicks);
		telemetry.addData("degPerTick", degPerTick);
	}
}
