package org.firstinspires.ftc.teamcode.autonomous.Auto_Blue;

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
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_BLUE_CLOSE_EXP;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "🔵🔵Close_3🔵🔵", group = "Auto")
public class Auto_BLUE_Close_3 extends BaseOpMode {
	private RobotHardware robot;

	// === TUNE THESE ===
	private static final int AUTON_TURRET_TICKS = 20;      // tune this
	private static final double AUTON_SHOOTER_POS = 0.54;  // tune this
	private static final double AUTON_SHOOTER_POS_SECOND = 0.54;
	private static final double AUTON_SHOOTER_POS_THIRD = 0.54;
	private static final double AUTON_SHOOTER_POS_FINAL = 0.54;
	private static final double TURRET_HOLD_POWER = 0.1;  // tune 0.05–0.20
	private static final double SHOOT_SAFE_IN = 11.0;

	// ===== shooter control loop (NECESAR pt custom PID) =====
	private boolean shooterEnabled = false;
	private double shooterRpmCmd = 0.0;
	private boolean autonDone = false;

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();
		robot.limelight.pipelineSwitch(0);

		// shooter rpm config
		OuttakeSystem.TICKS_PER_REV = 28;

		// Zero turret encoder at known starting angle
		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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

	/** Create a fresh LL aim action each time (prevents stale state). */
	private Action newAimTurretLL() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.045,   // kP
				0.12,    // minPower
				0.40,    // maxPower
				0.45,    // lockThreshold degrees
				750,     // timeout ms
				1.0,    // directionSign
				TURRET_HOLD_POWER
		);
	}

	private Action newAimTurretLLSecond() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.045,   // kP
				0.12,    // minPower
				0.40,    // maxPower
				0.45,    // lockThreshold degrees
				750,     // timeout ms
				1.0,    // directionSign
				TURRET_HOLD_POWER
		);
	}

	private Action newAimTurretLLThird() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.045,
				0.12,
				0.40,
				0.45,
				750,
				1.0,
				TURRET_HOLD_POWER
		);
	}

	/** Slightly more tolerant aim for later cycle. */
	private Action newAimTurretLLFinal() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.055,
				0.12,
				0.45,
				0.70,
				750,
				1.0,
				TURRET_HOLD_POWER
		);
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_BLUE_Close_3 op;
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
				Auto_BLUE_Close_3 op,
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

		/// === PATH ACTIONS ===

		Action startToShootBack =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.START)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
						))
						.build();

		Action shootToPickup1 =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.SHOOT)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP1.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP1.position.y
						))
						.build();

		Action leave =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.SHOOT)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.LEAVE.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.LEAVE.position.y
						))
						.build();
		Action pickup1ToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.PICKUP1)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x + SHOOT_SAFE_IN,
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
						))
						.build();

		Action shootToPickup2S =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.SHOOT)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2S.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2S.position.y
						))
						.build();

		Action pickup2SToPickup2 =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2S)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2.position.y
						))
						.build();

		Action pickup2ToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x + SHOOT_SAFE_IN,
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
						))
						.build();

		Action shootTopickup3S =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.SHOOT)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3S.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3S.position.y
						))
						.build();

		Action pickup3SToPickup3 =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3S)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3.position.y
						))
						.build();

		Action pickup3ToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x + SHOOT_SAFE_IN,
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
						))
						.build();

		Action pickupToOpengateS =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.PICKUP1)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.y
						))
						.build();

		Action opengateSToOpengate =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.OPENGATE.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.OPENGATE.position.y
						))
						.build();

		Action opengateToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.OPENGATE)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x,
								WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
						))
						.build();

		Action finishline =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_CLOSE_EXP.SHOOT)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.x - SHOOT_SAFE_IN,
								WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.y
						))
						.build();

		/// === SHOOTER AND INTAKE ACTIONS ===

		// Background controller: ține PID-ul alive tot auton-ul
		Action shooterController = packet -> {
			if (autonDone) return false;

			if (shooterEnabled) {
				robot.outtake1.setRpm(shooterRpmCmd);
				robot.outtake2.setRpm(shooterRpmCmd);
			}
			return true; // continuă în background
		};

		Action shooter_on = packet -> {
			shooterRpmCmd = 4000;
			shooterEnabled = true;
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
			turretHoldCurrent(TURRET_HOLD_POWER);
			return false;
		};

		Action shooter_off = packet -> {
			shooterEnabled = false;
			robot.outtake1.stop();
			robot.outtake2.stop();
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
		Action setAutonShooterAngleThird = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS_THIRD); return false; };
		Action setAutonShooterAngleFinal = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS_FINAL); return false; };

		Action startIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD); return false; };
		Action stopIntake  = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP); return false; };

		Action endAuton = packet -> {
			autonDone = true;
			shooterEnabled = false;
			robot.outtake1.stop();
			robot.outtake2.stop();
			robot.transfer.setPower(0);
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			return false;
		};

		/// === FULL AUTON SEQUENCE ===
	Actions.runBlocking(
			RunInParallel(
					shooterController,
					RunSequentially(

							RunInParallel(
									startToShootBack,
									WaitFor(0.1),
									shooter_on
							),

							setAutonShooterAngle,
							newAimTurretLL(),
							WaitFor(0.1),
							shootArtifact,
							WaitFor(1.0),
							stopShooting,
							shooter_off,
							WaitFor(0.5),
							leave
					)
			)
	);
	}
}
