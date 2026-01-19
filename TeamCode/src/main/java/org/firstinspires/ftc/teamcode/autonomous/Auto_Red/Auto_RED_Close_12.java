package org.firstinspires.ftc.teamcode.autonomous.Auto_Red;

import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_RED_CLOSE;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "Autonom_Red_Close12", group = "Auto")
public class Auto_RED_Close_12 extends BaseOpMode {
	private RobotHardware robot;

	// === TUNE THESE ===
	private static final int AUTON_TURRET_TICKS = 20;      // tune this (unused right now)
	private static final double AUTON_SHOOTER_POS = 0.72;  // tune this
	private static final double AUTON_SHOOTER_POS_SECOND = 0.74;
	private static final double AUTON_SHOOTER_POS_FINAL = 0.75;
	private static final double TURRET_HOLD_POWER = 0.1;  // tune 0.05–0.20

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();
		robot.limelight.pipelineSwitch(1);

		// Zero turret encoder at known starting angle
		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// hold turret immediately
		turretHoldCurrent(TURRET_HOLD_POWER);
	}

	@Override
	protected void jOnInitialize() {}

	/** Actively holds turret at its current encoder position (prevents drift). */
	private void turretHoldCurrent(double holdPower) {
		if (robot == null || robot.turret == null) return;
		DcMotorEx turret = robot.turret.getMotor();
		int pos = turret.getCurrentPosition();
		turret.setTargetPosition(pos);
		turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		turret.setPower(holdPower);
	}

	/** Lets turret be driven open-loop (used during limelight aiming). */
	private void turretOpenLoopBrake() {
		DcMotorEx turret = robot.turret.getMotor();
		turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	/** Create a fresh LL aim action each time. */
	private Action newAimTurretLL() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.045,   // kP
				0.12,    // minPower
				0.40,    // maxPower
				0.45,    // lockThreshold degrees
				750,     // timeout ms
				+1.0,    // directionSign
				TURRET_HOLD_POWER
		);
	}

	/** Slightly more tolerant aim for later cycles. */
	private Action newAimTurretLLFinal() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.055,   // kP
				0.12,    // minPower
				0.45,    // maxPower
				0.70,    // lockThreshold degrees
				750,     // timeout ms
				+1.0,    // directionSign
				TURRET_HOLD_POWER
		);
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_RED_Close_12 op;
		private final RobotHardware robot;

		private final double kP;
		private final double minPower;
		private final double maxPower;
		private final double lockThresholdDeg;
		private final long timeoutMs;
		private final double directionSign;
		private final double holdPower;

		private boolean initialized = false;
		private long startTimeMs = 0;

		AimTurretWithLimelightAction(
				Auto_RED_Close_12 op,
				RobotHardware robot,
				double kP,
				double minPower,
				double maxPower,
				double lockThresholdDeg,
				long timeoutMs,
				double directionSign,
				double holdPower
		) {
			this.op = op;
			this.robot = robot;
			this.kP = kP;
			this.minPower = minPower;
			this.maxPower = maxPower;
			this.lockThresholdDeg = lockThresholdDeg;
			this.timeoutMs = timeoutMs;
			this.directionSign = directionSign;
			this.holdPower = holdPower;
		}

		@Override
		public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
			if (!initialized) {
				initialized = true;
				startTimeMs = System.currentTimeMillis();
				op.turretOpenLoopBrake();
				robot.turret.getMotor().setPower(0);
			}

			long elapsed = System.currentTimeMillis() - startTimeMs;
			DcMotorEx turretMotor = robot.turret.getMotor();

			if (elapsed >= timeoutMs) {
				turretMotor.setPower(0);
				op.turretHoldCurrent(holdPower);
				packet.put("LL Aim", "TIMEOUT->HOLD");
				return false;
			}

			if (robot.limelight == null) {
				turretMotor.setPower(0);
				op.turretHoldCurrent(holdPower);
				packet.put("LL Aim", "NO LIMELIGHT->HOLD");
				return false;
			}

			LLResult result = robot.limelight.getLatestResult();
			packet.put("LL valid", (result != null && result.isValid()));
			packet.put("Turret mode", turretMotor.getMode().toString());

			if (result == null || !result.isValid()) {
				turretMotor.setPower(0);
				packet.put("LL Aim", "NO TARGET");
				return true;
			}

			double tx = result.getTx();
			double absTx = Math.abs(tx);

			packet.put("LL tx", tx);
			packet.put("LL absTx", absTx);

			if (absTx <= lockThresholdDeg) {
				turretMotor.setPower(0);
				op.turretHoldCurrent(holdPower);
				packet.put("LL Aim", "ALIGNED->HOLD");
				return false;
			}

			double power = directionSign * (kP * tx);

			if (power > 0) power = Math.max(power, minPower);
			else           power = Math.min(power, -minPower);

			if (power >  maxPower) power =  maxPower;
			if (power < -maxPower) power = -maxPower;

			turretMotor.setPower(power);
			packet.put("Turret power", power);

			return true;
		}
	}

	@Override
	protected void OnRun() {
		robot.drivetrain.updatePoseEstimate();

		// Common headings
		final double hShoot = WAYPOINTS_RED_CLOSE.SHOOT.heading.toDouble(); // constant heading you want
		final double tanForward = hShoot;
		final double tanBack = hShoot + Math.PI;

		/// === PATH ACTIONS ===

		Action goToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.START)
						.splineTo(
								new Vector2d(WAYPOINTS_RED_CLOSE.SHOOT.position.x, WAYPOINTS_RED_CLOSE.SHOOT.position.y),
								WAYPOINTS_RED_CLOSE.SHOOT.heading.toDouble()
						)
						.build();

		Action finishline =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.SHOOT)
						.splineTo(
								new Vector2d(WAYPOINTS_RED_CLOSE.FINISHLINE.position.x, WAYPOINTS_RED_CLOSE.FINISHLINE.position.y),
								WAYPOINTS_RED_CLOSE.FINISHLINE.heading.toDouble()
						)
						.build();

		// SHOOT -> PICKUPL1
		Action goToPickup =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.SHOOT)
						.setTangent(tanForward)
						.splineToConstantHeading(
								new Vector2d(WAYPOINTS_RED_CLOSE.PICKUPL1.position.x, WAYPOINTS_RED_CLOSE.PICKUPL1.position.y),
								tanForward
						)
						.build();

		// PICKUPL1 -> SHOOT (backwards translation, same heading)
		Action backToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUPL1)
						.setTangent(tanBack)
						.splineToConstantHeading(
								new Vector2d(WAYPOINTS_RED_CLOSE.SHOOT.position.x, WAYPOINTS_RED_CLOSE.SHOOT.position.y),
								tanBack
						)
						.build();

		// SHOOT -> PICKUP2 (left strafe-ish)
		double tanLeft = hShoot + Math.toRadians(90);
		Action goToPickup2 =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.SHOOT)
						.setTangent(tanLeft)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.PICKUP2.position,
								hShoot
						)
						.build();

		// PICKUP2 -> PICKUP2L
		Action goToPickup2F =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUP2)
						.setTangent(tanForward)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.PICKUP2L.position,
								hShoot
						)
						.build();

		// PICKUP2L -> SHOOT (your “works” style, made consistent)
		Action pickup2L_to_Shoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUP2L)
						.setTangent(tanForward)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.SHOOT.position,
								tanForward
						)
						.build();

		// SHOOT -> PICKUP3L  (FIXED: 138 must be radians if you meant degrees)
		Action shoot_to_pickup3L =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.SHOOT)
						.setTangent(Math.toRadians(138))   // <-- FIX: was 138 (wrong units)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.PICKUP3L.position,
								hShoot
						)
						.build();

		// PICKUP3L -> FINISHPICKUP
		Action pickup3L_to_finishpickup =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUP3L)
						.setTangent(tanForward)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.FINISHPICKUP.position,
								hShoot
						)
						.build();

		double h = WAYPOINTS_RED_CLOSE.SHOOT.heading.toDouble();
		Vector2d p0 = WAYPOINTS_RED_CLOSE.FINISHPICKUP.position;

// kick scurt în spate (5 inch)
		double backDist = 5.0;
		Vector2d pKick = new Vector2d(
				p0.x - backDist * Math.cos(h),
				p0.y - backDist * Math.sin(h)
		);

		Action finishpickup_to_Shoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.FINISHPICKUP)
						// KICK: spline foarte scurt, fără să-i dea timp să facă buclă
						.setTangent(h + Math.PI)
						.splineToConstantHeading(pKick, h + Math.PI)

						// TRASEU REAL spre SHOOT
						.setTangent(h + Math.PI)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.SHOOT.position,
								h + Math.PI
						)
						.build();


		/// === SHOOTER / INTAKE ACTIONS ===

		Action shooter_on = packet -> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
			turretHoldCurrent(TURRET_HOLD_POWER);
			return false;
		};

		Action shooter_off = packet -> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
			return false;
		};

		Action shootArtifact = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.transfer.setPower(-1);
			return false;
		};

		Action stopShooting = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.transfer.setPower(0);
			return false;
		};

		Action setAutonShooterAngle = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS); return false; };
		Action setAutonShooterAngleSecond = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS_SECOND); return false; };
		Action setAutonShooterAngleFinal = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS_FINAL); return false; };

		Action startIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD); return false; };
		Action stopIntake  = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP); return false; };

		/// === FULL AUTON SEQUENCE ===
		Actions.runBlocking(
				RunSequentially(
						RunInParallel(goToShoot, shooter_on),

						newAimTurretLL(),
						setAutonShooterAngle,
						WaitFor(0.2),

						// preload
						shootArtifact,
						WaitFor(0.7),
						shooter_off,
						stopShooting,
						startIntake,

						goToPickup,
						WaitFor(0.3),
						stopIntake,
						WaitFor(0.2),

						RunInParallel(backToShoot, shooter_on),

						// stack 1
						newAimTurretLLFinal(),
						setAutonShooterAngleSecond,
						WaitFor(0.2),
						shootArtifact,
						WaitFor(0.85),
						shooter_off,
						stopShooting,

						goToPickup2,
						WaitFor(0.4),
						startIntake,
						WaitFor(0.05),
						goToPickup2F,
						WaitFor(0.3),
						stopIntake,

						RunInParallel(pickup2L_to_Shoot, shooter_on),

						newAimTurretLL(),
						setAutonShooterAngleFinal,
						WaitFor(0.2),

						// stack 2
						shootArtifact,
						WaitFor(0.75),
						shooter_off,
						stopShooting,

						RunInParallel(shoot_to_pickup3L, startIntake),

						pickup3L_to_finishpickup,
						WaitFor(0.2),
						stopIntake,
						WaitFor(0.2),

						RunInParallel(finishpickup_to_Shoot, shooter_on),

						newAimTurretLL(),
						setAutonShooterAngleFinal,
						WaitFor(0.2),

						// stack 3
						shootArtifact,
						WaitFor(0.85),
						shooter_off,
						stopShooting,

						finishline
				)
		);
	}
}
