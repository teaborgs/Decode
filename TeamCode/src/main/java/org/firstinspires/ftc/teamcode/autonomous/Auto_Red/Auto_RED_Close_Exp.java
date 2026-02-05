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
import org.firstinspires.ftc.teamcode.RobotHardwareTEST;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_RED_CLOSE_EXP;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "Autonom_Red_Close_Exp", group = "Auto")
public class Auto_RED_Close_Exp extends BaseOpMode {
	private RobotHardwareTEST robot;

	// === TUNE THESE ===
	private static final int AUTON_TURRET_TICKS = 20;      // tune this
	private static final double AUTON_SHOOTER_POS = 0.75;  // tune this
	private static final double AUTON_SHOOTER_POS_SECOND = 0.60;
	private static final double AUTON_SHOOTER_POS_THIRD = 0.60;
	private static final double AUTON_SHOOTER_POS_FINAL = 0.60;
	private static final double TURRET_HOLD_POWER = 0.1;  // tune 0.05–0.20
	private static final double SHOOT_SAFE_IN = 8.0;


	@Override
	protected void OnInitialize() {
		robot = new RobotHardwareTEST(hardwareMap);
		robot.init();
		robot.limelight.pipelineSwitch(1);
		OuttakeSystem.TICKS_PER_REV = 28;

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
				750,    // timeout ms
				-1.0,    // directionSign (keep as sign; tune kP instead)
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
				0.45,     // lockThreshold degrees
				750,    // timeout ms
				-1.0,    // directionSign (keep as sign; tune kP instead)
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
				750,    // timeout ms (more time to reacquire)
				-1.0,    // directionSign
				TURRET_HOLD_POWER
		);
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_RED_Close_Exp op;      // to access turretHoldCurrent()
		private final RobotHardwareTEST robot;

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
				Auto_RED_Close_Exp op,
				RobotHardwareTEST robot,
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

		// SHOOT -> PICKUP2 (left strafe-ish)
		Action startToShootBack =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.START)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.x,
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.y
								)
						)
						.build();

		Action shootToPickup1 =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.SHOOT)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.PICKUP1.position.x,
										WAYPOINTS_RED_CLOSE_EXP.PICKUP1.position.y
								)
						)
						.build();

		Action pickup1ToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.PICKUP1)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.x + SHOOT_SAFE_IN,
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.y
								)
						)
						.build();

		Action shootToPickup2S =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.SHOOT)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.PICKUP2S.position.x,
										WAYPOINTS_RED_CLOSE_EXP.PICKUP2S.position.y
								)
						)
						.build();

		Action pickup2SToPickup2 =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.PICKUP2S)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.PICKUP2.position.x,
										WAYPOINTS_RED_CLOSE_EXP.PICKUP2.position.y
								)
						)
						.build();

		Action pickup2ToPickup2S =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.PICKUP2)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.PICKUP2S.position.x,
										WAYPOINTS_RED_CLOSE_EXP.PICKUP2S.position.y
								)
						)
						.build();

		Action pickup2ToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.PICKUP2)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.x + SHOOT_SAFE_IN,
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.y
								)
						)
						.build();

		Action shootTopickup3S =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.SHOOT)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.PICKUP3S.position.x,
										WAYPOINTS_RED_CLOSE_EXP.PICKUP3S.position.y
								)
						)
						.build();
		Action pickup3SToPickup3 =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.PICKUP3S)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.PICKUP3.position.x,
										WAYPOINTS_RED_CLOSE_EXP.PICKUP3.position.y
								)
						)
						.build();

		Action pickup3ToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.PICKUP3)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.x + SHOOT_SAFE_IN,
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.y
								)
						)
						.build();

		Action pickupToOpengateS =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.PICKUP1)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.OPENGATES.position.x,
										WAYPOINTS_RED_CLOSE_EXP.OPENGATES.position.y
								)
						)
						.build();

		Action opengateSToOpengate =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.OPENGATES)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.OPENGATE.position.x,
										WAYPOINTS_RED_CLOSE_EXP.OPENGATE.position.y
								)
						)
						.build();

		Action opengateToShoot =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.OPENGATE)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.x,
										WAYPOINTS_RED_CLOSE_EXP.SHOOT.position.y
								)
						)
						.build();

		Action finishline =
				robot.drivetrain.actionBuilder(WAYPOINTS_RED_CLOSE_EXP.SHOOT)
						.strafeTo(
								new Vector2d(
										WAYPOINTS_RED_CLOSE_EXP.OPENGATES.position.x - SHOOT_SAFE_IN,
										WAYPOINTS_RED_CLOSE_EXP.OPENGATES.position.y
								)
						)
						.build();








		// START -> SHOOT

		/// === SHOOTER AND INTAKE ACTIONS ===

		// start shooter and open stopper
		Action shooter_on = packet -> {
			robot.outtake1.setRpm(4000);
			robot.outtake2.setRpm(4000);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

			// safety: hold turret
			turretHoldCurrent(TURRET_HOLD_POWER);

			return false;
		};

		//start shooter and close stopper
		Action shooter_off = packet -> {
			robot.outtake1.stop();
			robot.outtake2.stop();
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
		Action setAutonShooterAngleSecond = packet -> {
			robot.turretTumbler.setPosition(AUTON_SHOOTER_POS_SECOND);
			return false;
		};
		Action setAutonShooterAngleThird = packet -> {
			robot.turretTumbler.setPosition(AUTON_SHOOTER_POS_THIRD);
			return false;
		};
		Action setAutonShooterAngleFinal = packet -> {
			robot.turretTumbler.setPosition(AUTON_SHOOTER_POS_FINAL);
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
						WaitFor(0.1),

						//pickup1
						startIntake,
						shootToPickup1,
						WaitFor(0.15),
						stopIntake,

						//opengate
						pickupToOpengateS,
						WaitFor(0.1),
						opengateSToOpengate,
						WaitFor(0.5),


						shooter_on,
						opengateToShoot,

						WaitFor(0.1),

						setAutonShooterAngleSecond,
						newAimTurretLLSecond(),
						WaitFor(0.1),
						shootArtifact,
						WaitFor(1.0),
						stopShooting,
						shooter_off,


						RunInParallel(
						shootToPickup2S,
						startIntake),

						WaitFor(0.1),
						pickup2SToPickup2,
						WaitFor(0.15),
						stopIntake,
						RunInParallel(
						pickup2ToShoot,
						WaitFor(0.1),
						shooter_on),
						WaitFor(0.1),


						setAutonShooterAngleThird,
						newAimTurretLLFinal(),
						WaitFor(0.1),
						shootArtifact,
						WaitFor(1.0),
						stopShooting,
						shooter_off,
						WaitFor(0.2),

						startIntake,
						shootTopickup3S,
						WaitFor(0.15),
						pickup3SToPickup3,
						stopIntake,
						RunInParallel(
						pickup3ToShoot,
						WaitFor(0.1),
						shooter_on),
						setAutonShooterAngleFinal,
						shootArtifact,
						WaitFor(1.0),
						finishline




				)
		);
	}
}
