package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous(name = "Autonom_Red_Close6", group = "Auto")
public class Auto_RED_Close_6 extends BaseOpMode {
	private RobotHardware robot;

	// === TUNE THESE ===
	private static final int AUTON_TURRET_TICKS = 20;      // tune this
	private static final double AUTON_SHOOTER_POS = 0.67;  // tune this
	private static final double TURRET_HOLD_POWER = 0.1;  // tune 0.05–0.20

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();

		// Zero turret encoder at known starting angle
		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		// Important: resist motion when power=0
		turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// Optional: start holding current position immediately (prevents any bump drift)
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

	/** Create a fresh LL aim action each time (prevents "works once, then doesn't" from stale internal state). */
	private Action newAimTurretLL() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.045,   // kP
				0.12,    // minPower
				0.40,    // maxPower
				0.45,     // lockThreshold degrees
				2500,    // timeout ms
				+1.0,    // directionSign (keep as sign; tune kP instead)
				TURRET_HOLD_POWER
		);
	}

	/** Slightly more tolerant aim for the 2nd shooting cycle (recover after fast drive + vibration). */
	private Action newAimTurretLLFinal() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.055,   // kP (a bit stronger)
				0.12,    // minPower
				0.45,    // maxPower (a bit higher)
				0.70,    // lockThreshold degrees (more forgiving)
				3500,    // timeout ms (more time to reacquire)
				+1.0,    // directionSign
				TURRET_HOLD_POWER
		);
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_RED_Close_6 op;      // to access turretHoldCurrent()
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
				Auto_RED_Close_6 op,
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

				// Open-loop for smooth aiming, but with BRAKE so it doesn't freewheel
				op.turretOpenLoopBrake();
				robot.turret.getMotor().setPower(0);
			}

			long now = System.currentTimeMillis();
			long elapsed = now - startTimeMs;

			DcMotorEx turretMotor = robot.turret.getMotor();

			// Timeout -> hold and finish
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

			// Useful debug (especially for the 2nd cycle)
			packet.put("LL valid", (result != null && result.isValid()));
			packet.put("Turret mode", turretMotor.getMode().toString());

			if (result == null || !result.isValid()) {
				// No target -> stop turning, but keep waiting until timeout (do NOT finish yet)
				turretMotor.setPower(0);
				packet.put("LL Aim", "NO TARGET");
				return true;
			}

			double tx = result.getTx(); // deg
			double absTx = Math.abs(tx);

			packet.put("LL tx", tx);
			packet.put("LL absTx", absTx);
			packet.put("LL timeLeftMs", (timeoutMs - elapsed));

			// Locked on -> hold and finish
			if (absTx <= lockThresholdDeg) {
				turretMotor.setPower(0);
				op.turretHoldCurrent(holdPower);
				packet.put("LL Aim", "ALIGNED->HOLD");
				return false;
			}

			// P-control
			double power = directionSign * (kP * tx);

			// enforce minimum power so it actually moves
			if (power > 0) power = Math.max(power, minPower);
			else           power = Math.min(power, -minPower);

			// clamp max power
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

		/// === PATH ACTIONS ===

		// START -> SHOOT

		Action goToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.START)
				.splineTo(
						new Vector2d(WAYPOINTS_RED_CLOSE.SHOOT.position.x, WAYPOINTS_RED_CLOSE.SHOOT.position.y),
						WAYPOINTS_RED_CLOSE.SHOOT.heading.toDouble()
				)
				.build();


		//PICKUP PATH
		double tan = WAYPOINTS_RED_CLOSE.SHOOT.heading.toDouble(); // -142° în rad
		Action goToPickup = robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.SHOOT)
				.setTangent(tan)
				.splineToConstantHeading(
						new Vector2d(WAYPOINTS_RED_CLOSE.PICKUPL1.position.x, WAYPOINTS_RED_CLOSE.PICKUPL1.position.y),
						tan
				)
				.build();


		// BACK TO SHOOT

		double tanBack = tan + Math.PI;

		Action backToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUPL1)
				.setTangent(tanBack)
				.splineToConstantHeading(
						new Vector2d(WAYPOINTS_RED_CLOSE.SHOOT.position.x, WAYPOINTS_RED_CLOSE.SHOOT.position.y),
						tanBack
				)
				.build();

		//PICKUP2

		double heading = WAYPOINTS_RED_CLOSE.SHOOT.heading.toDouble(); // -142°

		double tanLeft = heading + Math.toRadians(90);

		Action goToPickup2 =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.SHOOT)
						.setTangent(tanLeft)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.PICKUP2.position,
								heading
						)
						.build();

		double tanForward = heading;

		Action goToPickup2F =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUP2)
						.setTangent(tanForward)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.PICKUP2L.position,
								heading
						)
						.build();

		double tanBack2 = heading + Math.toRadians(180);

		Action backToPickup2 =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUP2L)
						.setTangent(tanBack2)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.PICKUP2.position,
								heading
						)
						.build();

		double tanRight = heading - Math.toRadians(90);

		Action backToShoot2 =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUP2L)
						.setTangent(tanRight)
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.SHOOT.position,
								heading
						)
						.build();

		double headingback = WAYPOINTS_RED_CLOSE.SHOOT.heading.toDouble();

		Action pickup2L_to_Shoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE.PICKUP2L)
						.setTangent(headingback) // pleacă drept înainte în direcția heading-ului
						.splineToConstantHeading(
								WAYPOINTS_RED_CLOSE.SHOOT.position,
								heading
						)
						.build();





		/// === SHOOTER AND INTAKE ACTIONS ===

		// start shooter and open stopper
		Action shooter_on = packet -> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

			// extra safety: ensure turret is holding before the vibration starts
			turretHoldCurrent(TURRET_HOLD_POWER);

			return false;
		};

		//start shooter and close stopper
		Action shooter_off = packet -> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
			return false;
		};

		// feed artifacts to shooter
		Action shootArtifact = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.transfer.setPower(-1);
			return false;
		};

		// stop feeding artifacts to shooter
		Action stopShooting = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.transfer.setPower(0);
			return false;
		};

		// turret preset angle
		Action moveTurretToAutonAngle = packet -> {
			DcMotorEx turretMotor = robot.turret.getMotor();
			turretMotor.setTargetPosition(AUTON_TURRET_TICKS);
			turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			turretMotor.setPower(0.3);
			return false;
		};

		// shooter platform servo angle
		Action setAutonShooterAngle = packet -> {
			robot.turretTumbler.setPosition(AUTON_SHOOTER_POS);
			return false;
		};

		// floor intake (for pickup path)
		Action startIntake = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			return false;
		};

		Action stopIntake = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			return false;
		};

		// === FULL AUTON SEQUENCE ===

		Actions.runBlocking(
				RunSequentially(
						goToShoot,
						WaitFor(0.35),
						newAimTurretLL(),
						setAutonShooterAngle,

						shooter_on,
						WaitFor(0.8),
						shootArtifact,
						WaitFor(1.8),
						shooter_off,

						goToPickup,
						WaitFor(0.3),
						stopShooting,
						WaitFor(0.2),

						backToShoot,
						WaitFor(0.5),

						newAimTurretLLFinal(),
						setAutonShooterAngle,
						shooter_on,
						WaitFor(0.5),
						shootArtifact,
						WaitFor(1.8),
						shooter_off,
						stopShooting,
						goToPickup2







				)
		);
	}
}
