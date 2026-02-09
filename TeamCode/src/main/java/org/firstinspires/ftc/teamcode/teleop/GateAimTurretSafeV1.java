package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "🎯Gate Aim Turret (Safe)🎯", group = "TeleOp")
public final class GateAimTurretSafeV1 extends BaseOpMode {

	private InputSystem driveInput, armInput;
	private RobotHardware robot;

	// ===== Buttons =====
	private static final InputSystem.Key KEY_RESET_GATE = new InputSystem.Key("dpad_left");
	private static final InputSystem.Key KEY_AIM_TOGGLE = new InputSystem.Key("dpad_up");
	private static final InputSystem.Key KEY_SUPPRESS   = new InputSystem.Key("left_bumper");

	// Drive
	private static final InputSystem.Axis AX_DRIVE_X = new InputSystem.Axis("left_stick_x");
	private static final InputSystem.Axis AX_DRIVE_Y = new InputSystem.Axis("left_stick_y");
	private static final InputSystem.Axis AX_ROT_L   = new InputSystem.Axis("left_trigger");
	private static final InputSystem.Axis AX_ROT_R   = new InputSystem.Axis("right_trigger");

	// ===== Gate pose =====
	private static final double GATE_X_IN = 0.0;
	private static final double GATE_Y_IN = 0.0;
	private static final double GATE_H_DEG = 0.0;

	// ===== Turret mechanical limits (deg intention) =====
	private static final double TURRET_LIMIT_DEG = 170.0;
	private static final double TURRET_RETURN_OK_DEG = 165.0; // hysteresis

	// ===== Ticks safety =====
	private static final int TICKS_BUFFER = 10;

	// ===== LL micro correction =====
	private static final double LL_TX_DEADZONE = 1.5;
	private static final double LL_GAIN = 1.0;
	private static final double LL_MAX_CORR_DEG = 8.0;
	private static final double LL_DISABLE_NEAR_LIMIT_DEG = 160.0;

	// ===== PID in ticks =====
	private static final double kP = 0.0040;
	private static final double kD = 0.0008;
	private static final double MAX_POWER = 0.45;
	private static final int ON_TARGET_TICKS = 6;

	// ===== YOUR CALIB VALUES (hard-coded) =====
	// “anchor” = punctul de shot pe care l-ai salvat
	private static final double anchorX = -4.205;
	private static final double anchorY = -47.2062;

	// limite turetă (ticks) calibrate de tine
	private static final int turretMinTicks = -472;
	private static final int turretMaxTicks =  438;

	// degPerTick corect: 340 deg / (438 - (-472)) = 340 / 910
	private static final double degPerTick = 340.0 / 910.0; // ≈ 0.373626

	// IMPORTANT: la tine semnul e invers
	private static final double MOTOR_SIGN = -1.0;

	// ===== State =====
	private boolean autoAim = false;
	private boolean aimInvalid = false;

	private boolean prevResetGate = false;
	private boolean prevAimToggle = false;

	private int targetTicks = 0;
	private double errPrev = 0.0;
	private final ElapsedTime dtTimer = new ElapsedTime();

	// debug
	private double turretDesiredDeg_dbg = 0.0;
	private double turretCmdDeg_dbg = 0.0;
	private double targetAngleFieldDeg_dbg = 0.0;

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

		// pipeline (schimbă dacă trebuie)
		robot.limelight.pipelineSwitch(0);

		// IMPORTANT: reset encoder turetă (altfel pornește de la mii de ticks)
		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// reset pose la gate
		setPoseInches(GATE_X_IN, GATE_Y_IN, GATE_H_DEG);

		dtTimer.reset();
	}

	@Override
	protected void OnRun() {
		robot.drivetrain.updatePoseEstimate();

		boolean resetGate = driveInput.isPressed(KEY_RESET_GATE);
		if (resetGate && !prevResetGate) {
			setPoseInches(GATE_X_IN, GATE_Y_IN, GATE_H_DEG);
		}
		prevResetGate = resetGate;

		boolean aimToggle = driveInput.isPressed(KEY_AIM_TOGGLE);
		if (aimToggle && !prevAimToggle) {
			autoAim = !autoAim;
			aimInvalid = false;
			errPrev = 0.0;
			dtTimer.reset();
		}
		prevAimToggle = aimToggle;

		Drive();
		TurretAimSafe();
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

	private void TurretAimSafe() {
		if (!autoAim) {
			robot.turret.getMotor().setPower(0);
			return;
		}

		Pose2d p = robot.drivetrain.pose;
		double robotX = p.position.x;
		double robotY = p.position.y;
		double robotHeadingDeg = Math.toDegrees(p.heading.log());

		// 1) unghi spre anchor (field)
		double dx = anchorX - robotX;
		double dy = -(anchorY - robotY);
		double targetAngleFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
		targetAngleFieldDeg_dbg = targetAngleFieldDeg;

		// 2) unghi relativ la robot
		double turretDesiredDeg = wrapDeg(targetAngleFieldDeg - robotHeadingDeg);
		turretDesiredDeg_dbg = turretDesiredDeg;

		// 3) invalid detection (în spate)
		if (!aimInvalid) {
			if (Math.abs(turretDesiredDeg) > TURRET_LIMIT_DEG) aimInvalid = true;
		} else {
			if (Math.abs(turretDesiredDeg) < TURRET_RETURN_OK_DEG) aimInvalid = false;
		}

		if (aimInvalid) {
			robot.turret.getMotor().setPower(0);
			return;
		}

		// 4) micro LL tx
		double corrDeg = 0.0;
		if (Math.abs(turretDesiredDeg) < LL_DISABLE_NEAR_LIMIT_DEG) {
			LLResult r = robot.limelight.getLatestResult();
			if (r != null && r.isValid()) {
				double tx = r.getTx();
				if (Math.abs(tx) > LL_TX_DEADZONE) {
					corrDeg = clamp(tx * LL_GAIN, -LL_MAX_CORR_DEG, LL_MAX_CORR_DEG);
				}
			}
		}

		double turretCmdDeg = clamp(turretDesiredDeg + corrDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);
		turretCmdDeg_dbg = turretCmdDeg;

		// 5) deg -> ticks
		int unclamped = (int) Math.round(turretCmdDeg / degPerTick);
		targetTicks = clampTicks(unclamped);

		// 6) PID pe ticks + hard guard
		int nowTicks = robot.turret.getMotor().getCurrentPosition();
		int errTicks = targetTicks - nowTicks;

		double dt = Math.max(0.01, dtTimer.seconds());
		dtTimer.reset();

		double err = errTicks;
		double derr = (err - errPrev) / dt;
		errPrev = err;

		if (Math.abs(errTicks) < ON_TARGET_TICKS) {
			robot.turret.getMotor().setPower(0);
			return;
		}

		double power = kP * err + kD * derr;
		power = clamp(power, -MAX_POWER, MAX_POWER);

		power = guardPowerByLimits(power, nowTicks);

		// semn invers (la tine)
		robot.turret.getMotor().setPower(MOTOR_SIGN * power);
	}

	private int clampTicks(int t) {
		int lo = turretMinTicks + TICKS_BUFFER;
		int hi = turretMaxTicks - TICKS_BUFFER;
		return Math.max(lo, Math.min(hi, t));
	}

	private double guardPowerByLimits(double pwr, int nowTicks) {
		int lo = turretMinTicks + TICKS_BUFFER;
		int hi = turretMaxTicks - TICKS_BUFFER;

		// pwr pozitiv = vrei spre max ticks
		if (nowTicks >= hi && pwr > 0) return 0.0;
		if (nowTicks <= lo && pwr < 0) return 0.0;

		return pwr;
	}

	private void setPoseInches(double xIn, double yIn, double headingDeg) {
		robot.drivetrain.pose = new Pose2d(new Vector2d(xIn, yIn), Math.toRadians(headingDeg));
	}

	private static double wrapDeg(double a) {
		a = a % 360.0;
		if (a <= -180.0) a += 360.0;
		if (a > 180.0) a -= 360.0;
		return a;
	}

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}

	@Override
	protected void OnTelemetry(Telemetry telemetry) {
		super.OnTelemetry(telemetry);

		Pose2d p = robot.drivetrain.pose;

		telemetry.addData("Aim", autoAim ? "ON" : "OFF");
		telemetry.addData("AimInvalid", aimInvalid ? "YES (TURN ROBOT)" : "NO");

		telemetry.addLine(" ");
		telemetry.addData("ODO X(in)", p.position.x);
		telemetry.addData("ODO Y(in)", p.position.y);
		telemetry.addData("ODO H(deg)", Math.toDegrees(p.heading.log()));

		telemetry.addLine(" ");
		telemetry.addData("AnchorX", anchorX);
		telemetry.addData("AnchorY", anchorY);

		telemetry.addLine(" ");
		telemetry.addData("fieldAngleDeg", targetAngleFieldDeg_dbg);
		telemetry.addData("turretDesiredDeg", turretDesiredDeg_dbg);
		telemetry.addData("turretCmdDeg", turretCmdDeg_dbg);

		int nowTicks = robot.turret.getMotor().getCurrentPosition();
		telemetry.addLine(" ");
		telemetry.addData("TurretTicks", nowTicks);
		telemetry.addData("TargetTicks", targetTicks);
		telemetry.addData("MinTicks", turretMinTicks);
		telemetry.addData("MaxTicks", turretMaxTicks);
		telemetry.addData("degPerTick", degPerTick);
		telemetry.addData("BUFFER", TICKS_BUFFER);
		telemetry.addData("MOTOR_SIGN", MOTOR_SIGN);

		LLResult r = robot.limelight.getLatestResult();
		telemetry.addLine(" ");
		telemetry.addData("LL valid", (r != null && r.isValid()));
		if (r != null && r.isValid()) telemetry.addData("LL tx", r.getTx());
	}
}
