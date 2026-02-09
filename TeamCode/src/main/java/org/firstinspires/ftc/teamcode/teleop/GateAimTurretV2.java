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

@TeleOp(name = "🎯Gate Aim Turret (V2 FIXED + LL)🎯", group = "TeleOp")
public final class GateAimTurretV2 extends BaseOpMode {

	private InputSystem driveInput;
	private RobotHardware robot;

	// ================= BUTTONS =================
	private static final InputSystem.Key KEY_RESET_GATE = new InputSystem.Key("dpad_left");
	private static final InputSystem.Key KEY_AIM_TOGGLE = new InputSystem.Key("dpad_up");
	private static final InputSystem.Key KEY_SUPPRESS   = new InputSystem.Key("left_bumper");

	// Drive
	private static final InputSystem.Axis AX_DRIVE_X = new InputSystem.Axis("left_stick_x");
	private static final InputSystem.Axis AX_DRIVE_Y = new InputSystem.Axis("left_stick_y");
	private static final InputSystem.Axis AX_ROT_L   = new InputSystem.Axis("left_trigger");
	private static final InputSystem.Axis AX_ROT_R   = new InputSystem.Axis("right_trigger");

	// ================= GATE POSE =================
	private static final double GATE_X_IN = 0.0;
	private static final double GATE_Y_IN = 0.0;
	private static final double GATE_H_DEG = 0.0;

	// ================= TURRET LIMITS =================
	private static final double TURRET_LIMIT_DEG = 170.0;
	private static final double TURRET_RETURN_OK_DEG = 165.0;
	private static final int TICKS_BUFFER = 10;

	// ================= PID =================
	private static final double kP = 0.0020;
	private static final double kD = 0.0003;
	private static final double MAX_POWER = 0.40;
	private static final int ON_TARGET_TICKS = 6;

	// ================= CALIB (ANCHOR) =================
	private static final double anchorX = -4.205;
	private static final double anchorY = -47.2062;

	private static final int turretMinTicks = -472;
	private static final int turretMaxTicks =  438;

	private static final double degPerTick = 340.0 / 910.0;

	// IMPORTANT: după fixul cu heading inversat, motorul NU mai e inversat
	private static final double MOTOR_SIGN = 1.0;

	// ================= ANTI JITTER =================
	private static final int TARGET_HOLD_TICKS = 15;
	private static final int MAX_TARGET_STEP_TICKS = 10;

	private static final double CMD_LPF_ALPHA = 0.20;
	private double turretCmdDegFiltered = 0.0;
	private boolean filterInit = false;

	// ================= LIMELIGHT =================
	private static final double LL_TX_DEADZONE = 1.5;
	private static final double LL_GAIN = 0.8;
	private static final double LL_MAX_CORR_DEG = 6.0;
	private static final double LL_DISABLE_WHEN_TURNING_DEG_S = 80.0;
	private static final double LL_STRONG_ZONE_DEG = 10.0;
	private static final double LL_STRONG_GAIN = 1.6;

	// ================= ROTATION FREEZE =================
	private static final double MAX_ROBOT_ROT_DEG_S = 120.0;
	private double prevHeadingDeg = 0.0;

	// ================= STATE =================
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
	private double omegaDbg = 0.0;
	private int errTicksDbg = 0;
	private double corrDegDbg = 0.0;

	// ================= INIT =================
	@Override
	protected void OnInitialize() {
		driveInput = input1;
	}

	@Override
	protected void jOnInitialize() {
		driveInput = input1;
	}

	@Override
	protected void OnStart() {
		robot = new RobotHardware(hardwareMap);
		robot.init();

		robot.limelight.pipelineSwitch(0);

		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		setPoseInches(GATE_X_IN, GATE_Y_IN, GATE_H_DEG);

		prevHeadingDeg = 0.0;
		filterInit = false;
		errPrev = 0.0;
		dtTimer.reset();
	}

	// ================= LOOP =================
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
			filterInit = false;
			dtTimer.reset();
		}
		prevAimToggle = aimToggle;

		Drive();
		TurretAim();
	}

	// ================= DRIVE =================
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

	// ================= TURRET AIM =================
	private void TurretAim() {
		if (!autoAim) {
			robot.turret.getMotor().setPower(0);
			return;
		}

		Pose2d p = robot.drivetrain.pose;
		double robotX = p.position.x;
		double robotY = p.position.y;

		double headingDeg = Math.toDegrees(p.heading.log());
		double dt = Math.max(0.01, dtTimer.seconds());

		double omega = wrapDeg(headingDeg - prevHeadingDeg) / dt;
		prevHeadingDeg = headingDeg;
		omegaDbg = omega;

		if (Math.abs(omega) > MAX_ROBOT_ROT_DEG_S) {
			robot.turret.getMotor().setPower(0);
			dtTimer.reset();
			return;
		}

		// ===== ODOM AIM (field -> robot) =====
		double dx = anchorX - robotX;
		double dy = anchorY - robotY;

		// heading inversat (frame fix) — păstrăm asta
		double headingRad = -p.heading.log();

		double fieldAngleRad = Math.atan2(dy, dx);
		double turretDesiredDeg = Math.toDegrees(fieldAngleRad - headingRad);
		turretDesiredDeg = wrapDeg(turretDesiredDeg);
		turretDesiredDeg_dbg = turretDesiredDeg;

		// invalid detection
		if (!aimInvalid) {
			if (Math.abs(turretDesiredDeg) > TURRET_LIMIT_DEG) aimInvalid = true;
		} else {
			if (Math.abs(turretDesiredDeg) < TURRET_RETURN_OK_DEG) aimInvalid = false;
		}

		if (aimInvalid) {
			robot.turret.getMotor().setPower(0);
			dtTimer.reset();
			return;
		}

		// ===== LIMELIGHT MICRO CORRECTION (FIXED SIGN) =====
		double corrDeg = 0.0;
		LLResult r = robot.limelight.getLatestResult();

		if (r != null && r.isValid() && Math.abs(omega) < LL_DISABLE_WHEN_TURNING_DEG_S) {
			double tx = r.getTx();

			// IMPORTANT: semn inversat aici (la tine tx era "pe dos")
			if (Math.abs(tx) < LL_STRONG_ZONE_DEG) {
				corrDeg = clamp(-tx * LL_STRONG_GAIN, -12.0, 12.0);
			} else if (Math.abs(tx) > LL_TX_DEADZONE) {
				corrDeg = clamp(-tx * LL_GAIN, -LL_MAX_CORR_DEG, LL_MAX_CORR_DEG);
			}
		}
		corrDegDbg = corrDeg;

		double turretCmdDeg = clamp(turretDesiredDeg + corrDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);
		turretCmdDeg_dbg = turretCmdDeg;

		// ===== FILTER (dynamic alpha) =====
		if (!filterInit) {
			turretCmdDegFiltered = turretCmdDeg;
			filterInit = true;
		} else {
			double alpha = (r != null && r.isValid()) ? 0.45 : CMD_LPF_ALPHA;
			turretCmdDegFiltered += alpha * (turretCmdDeg - turretCmdDegFiltered);
		}

		// ===== deg -> ticks =====
		int unclamped = (int) Math.round(turretCmdDegFiltered / degPerTick);
		int newTarget = clampTicks(unclamped);

		// hold
		if (Math.abs(newTarget - targetTicks) <= TARGET_HOLD_TICKS) {
			newTarget = targetTicks;
		}

		// slew
		int delta = newTarget - targetTicks;
		if (Math.abs(delta) > MAX_TARGET_STEP_TICKS) {
			targetTicks += Math.signum(delta) * MAX_TARGET_STEP_TICKS;
		} else {
			targetTicks = newTarget;
		}

		// ===== PID =====
		int nowTicks = robot.turret.getMotor().getCurrentPosition();
		int errTicks = targetTicks - nowTicks;
		errTicksDbg = errTicks;

		double derr = (errTicks - errPrev) / dt;
		errPrev = errTicks;

		if (Math.abs(errTicks) < ON_TARGET_TICKS) {
			robot.turret.getMotor().setPower(0);
			dtTimer.reset();
			return;
		}

		double power = kP * errTicks + kD * derr;
		power = clamp(power, -MAX_POWER, MAX_POWER);
		power = guardPower(power, nowTicks);

		robot.turret.getMotor().setPower(MOTOR_SIGN * power);
		dtTimer.reset();
	}

	// ================= HELPERS =================
	private int clampTicks(int t) {
		int lo = turretMinTicks + TICKS_BUFFER;
		int hi = turretMaxTicks - TICKS_BUFFER;
		return Math.max(lo, Math.min(hi, t));
	}

	private double guardPower(double pwr, int nowTicks) {
		int lo = turretMinTicks + TICKS_BUFFER;
		int hi = turretMaxTicks - TICKS_BUFFER;
		if (nowTicks >= hi && pwr > 0) return 0.0;
		if (nowTicks <= lo && pwr < 0) return 0.0;
		return pwr;
	}

	private void setPoseInches(double x, double y, double hDeg) {
		robot.drivetrain.pose = new Pose2d(new Vector2d(x, y), Math.toRadians(hDeg));
	}

	private static double wrapDeg(double a) {
		a %= 360.0;
		if (a <= -180.0) a += 360.0;
		if (a > 180.0) a -= 360.0;
		return a;
	}

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}

	// ================= TELEMETRY =================
	@Override
	protected void OnTelemetry(Telemetry t) {
		super.OnTelemetry(t);

		Pose2d p = robot.drivetrain.pose;

		// ODOM back
		t.addData("ODO X", p.position.x);
		t.addData("ODO Y", p.position.y);
		t.addData("ODO H(deg)", Math.toDegrees(p.heading.log()));

		t.addLine(" ");
		t.addData("Aim", autoAim);
		t.addData("AimInvalid", aimInvalid);

		t.addLine(" ");
		t.addData("omega(deg/s)", omegaDbg);
		t.addData("corrDeg", corrDegDbg);
		t.addData("turretDesiredDeg", turretDesiredDeg_dbg);
		t.addData("turretCmdDeg", turretCmdDeg_dbg);

		t.addLine(" ");
		t.addData("ErrTicks", errTicksDbg);
		t.addData("TargetTicks", targetTicks);
		t.addData("TurretTicks", robot.turret.getMotor().getCurrentPosition());

		LLResult r = robot.limelight.getLatestResult();
		t.addLine(" ");
		t.addData("LL valid", (r != null && r.isValid()));
		if (r != null && r.isValid()) t.addData("LL tx", r.getTx());
	}
}
