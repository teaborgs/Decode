package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "🔵TurretAim_ResetPoint_PID🔵", group = "TeleOp")
public final class TurretAim_ResetPoint_PID extends BaseOpMode {

	private InputSystem driveInput, armInput;
	private RobotHardware robot;

	// ========== INPUTS ==========
	private static final InputSystem.Key KEY_AIM_TOGGLE = new InputSystem.Key("dpad_up");
	private static final InputSystem.Key KEY_RESET_POSE = new InputSystem.Key("dpad_left");
	private static final InputSystem.Key KEY_RESET_TURRET_ENCODER = new InputSystem.Key("dpad_down");
	private static final InputSystem.Key KEY_SUPPRESS = new InputSystem.Key("left_bumper");

	private static final InputSystem.Axis AX_DRIVE_X = new InputSystem.Axis("left_stick_x");
	private static final InputSystem.Axis AX_DRIVE_Y = new InputSystem.Axis("left_stick_y");
	private static final InputSystem.Axis AX_ROT_L   = new InputSystem.Axis("left_trigger");
	private static final InputSystem.Axis AX_ROT_R   = new InputSystem.Axis("right_trigger");

	// ========== FIELD (inch) ==========
	// Astea le setezi tu după “reset point”-ul tău
	private static final double RESET_X_IN = 0.0;
	private static final double RESET_Y_IN = 0.0;
	private static final double RESET_H_DEG = 0.0;

	// Coordonatele basketului în același sistem ca reset point
	private static final double BASKET_X_IN = 60.0;
	private static final double BASKET_Y_IN = 30.0;

	// ========== TURRET LIMITS ==========
	private static final double TURRET_LIMIT_DEG = 170.0;
	private static final double TURRET_RETURN_OK_DEG = 165.0; // hysteresis

	// ========== LIMELIGHT MICRO ==========
	private static final double LL_TX_DEADZONE = 1.5;
	private static final double LL_GAIN = 1.0;
	private static final double LL_MAX_CORR_DEG = 8.0;
	private static final double LL_DISABLE_NEAR_LIMIT_DEG = 160.0;

	// ========== TURRET ENCODER MODEL ==========
	// Pune valorile reale când știi (altfel tuningul în grade e off).
	// Pentru start, dacă nu știi, lasă 1440 și o să ajustezi.
	private static final double TURRET_TICKS_PER_REV = 1440.0;
	private static final double TURRET_GEAR_RATIO = 1.0;
	private static final double DEG_PER_TICK = 360.0 / (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO);

	// dacă 0 ticks nu e “înainte”, corectezi aici
	private static final double TURRET_ZERO_OFFSET_DEG = 0.0;

	// motorul turetei e setat Direction.REVERSE în IntakeSystem
	// dacă tureta se mișcă invers, schimbă între +1 și -1
	private static final double TURRET_POWER_SIGN = 1.0;

	// ========== PID ==========
	private static final double kP = 0.020;
	private static final double kD = 0.001;
	private static final double MAX_POWER = 0.45;
	private static final double ON_TARGET_DEG = 2.0;

	private boolean autoAim = false;
	private boolean aimInvalid = false;

	private boolean prevAimToggle = false;
	private boolean prevResetPose = false;
	private boolean prevResetTurretEnc = false;

	private double turretTargetDeg = 0.0;
	private double errPrev = 0.0;

	private final ElapsedTime dtTimer = new ElapsedTime();

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

		// pipeline LL (schimbă indexul dacă trebuie)
		robot.limelight.pipelineSwitch(0);

		// Reset pose (anchor)
		setPoseInches(RESET_X_IN, RESET_Y_IN, RESET_H_DEG);

		// IMPORTANT: tureta trebuie pusă fizic în “home” înainte de start
		resetTurretEncoder();

		dtTimer.reset();
	}

	@Override
	protected void OnRun() {
		robot.drivetrain.updatePoseEstimate();

		boolean aimToggle = driveInput.isPressed(KEY_AIM_TOGGLE);
		if (aimToggle && !prevAimToggle) {
			autoAim = !autoAim;
			aimInvalid = false;
		}
		prevAimToggle = aimToggle;

		boolean resetPose = driveInput.isPressed(KEY_RESET_POSE);
		if (resetPose && !prevResetPose) {
			setPoseInches(RESET_X_IN, RESET_Y_IN, RESET_H_DEG);
		}
		prevResetPose = resetPose;

		boolean resetTurretEnc = driveInput.isPressed(KEY_RESET_TURRET_ENCODER);
		if (resetTurretEnc && !prevResetTurretEnc) {
			resetTurretEncoder();
		}
		prevResetTurretEnc = resetTurretEnc;

		Drive();
		TurretAimPID();
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

	private void TurretAimPID() {
		if (!autoAim) {
			setTurretPower(0.0);
			return;
		}

		Pose2d p = robot.drivetrain.pose;
		double robotX = p.position.x;
		double robotY = p.position.y;
		double robotHeadingDeg = Math.toDegrees(p.heading.log());

		// unghi spre basket (field)
		double dx = BASKET_X_IN - robotX;
		double dy = BASKET_Y_IN - robotY;
		double targetAngleFieldDeg = Math.toDegrees(Math.atan2(dy, dx));

		// setpoint tureta relativ la robot
		double turretDesiredDeg = wrapDeg(targetAngleFieldDeg - robotHeadingDeg - TURRET_ZERO_OFFSET_DEG);

		// limits + hysteresis
		if (!aimInvalid) {
			if (Math.abs(turretDesiredDeg) > TURRET_LIMIT_DEG) aimInvalid = true;
		} else {
			if (Math.abs(turretDesiredDeg) < TURRET_RETURN_OK_DEG) aimInvalid = false;
		}

		if (aimInvalid) {
			setTurretPower(0.0);
			return;
		}

		// LL micro correction (tx)
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

		turretTargetDeg = clamp(turretDesiredDeg + corrDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);

		// PID pe encoder (ticks -> deg)
		double turretNowDeg = getTurretDeg();
		double err = wrapDeg(turretTargetDeg - turretNowDeg);

		double dt = Math.max(0.01, dtTimer.seconds());
		dtTimer.reset();

		double derr = (err - errPrev) / dt;
		errPrev = err;

		if (Math.abs(err) < ON_TARGET_DEG) {
			setTurretPower(0.0);
			return;
		}

		double power = kP * err + kD * derr;
		power = clamp(power, -MAX_POWER, MAX_POWER);

		setTurretPower(power);
	}

	// ======== TURRET IO (ACUM E REAL, NU TODO) ========
	private void setTurretPower(double pwr) {
		if (robot == null || robot.turret == null) return;
		robot.turret.getMotor().setPower(TURRET_POWER_SIGN * pwr);
	}

	private double getTurretDeg() {
		if (robot == null || robot.turret == null) return 0.0;
		int ticks = robot.turret.getMotor().getCurrentPosition();
		return ticks * DEG_PER_TICK;
	}

	private void resetTurretEncoder() {
		if (robot == null || robot.turret == null) return;
		DcMotor m = robot.turret.getMotor();
		m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	// ======== POSE SET ========
	private void setPoseInches(double xIn, double yIn, double headingDeg) {
		robot.drivetrain.pose = new Pose2d(new Vector2d(xIn, yIn), Math.toRadians(headingDeg));
	}

	// ======== MATH ========
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
		telemetry.addData("ODO X(in)", p.position.x);
		telemetry.addData("ODO Y(in)", p.position.y);
		telemetry.addData("ODO H(deg)", Math.toDegrees(p.heading.log()));

		telemetry.addData("Aim", autoAim ? "ON" : "OFF");
		telemetry.addData("AimInvalid", aimInvalid ? "YES (TURN ROBOT)" : "NO");

		telemetry.addData("TurretTarget(deg)", turretTargetDeg);
		telemetry.addData("TurretTicks", robot.turret.getMotor().getCurrentPosition());
		telemetry.addData("TurretDegNow", getTurretDeg());

		LLResult r = robot.limelight.getLatestResult();
		telemetry.addData("LL valid", (r != null && r.isValid()));
		if (r != null && r.isValid()) telemetry.addData("LL tx", r.getTx());
	}
}
