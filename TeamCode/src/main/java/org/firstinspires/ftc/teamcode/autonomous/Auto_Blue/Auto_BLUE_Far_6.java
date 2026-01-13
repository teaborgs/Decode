package org.firstinspires.ftc.teamcode.autonomous.Auto_Blue;

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
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_BLUE_FAR;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "Autonom_Blue_Far6", group = "Auto")
public class Auto_BLUE_Far_6 extends BaseOpMode {
	private RobotHardware robot;

	// === TUNE THESE ===
	private static final int AUTON_TURRET_TICKS = 20;      // tune this
	private static final double AUTON_SHOOTER_POS = 0.75;  // tune this
	private static final double TURRET_HOLD_POWER = 0.1;  // tune 0.05–0.20

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();
		robot.limelight.pipelineSwitch(0);

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
				0.035,   // kP
				0.7,    // minPower
				0.35,    // maxPower
				0.30,     // lockThreshold degrees
				750,    // timeout ms
				+1.0,    // directionSign (keep as sign; tune kP instead)
				TURRET_HOLD_POWER
		);
	}

	/** Slightly more tolerant aim for the 2nd shooting cycle (recover after fast drive + vibration). */
	private Action newAimTurretLLFinal() {
		return new AimTurretWithLimelightAction(
				this,
				robot,
				0.035,   // kP
				0.7,    // minPower
				0.35,    // maxPower
				0.30,     // lockThreshold degrees
				750,    // timeout ms
				+1.0,    // directionSign (keep as sign; tune kP instead)
				TURRET_HOLD_POWER
		);
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_BLUE_Far_6 op;      // to access turretHoldCurrent()
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
				Auto_BLUE_Far_6 op,
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
		Action goToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.START)
				.setTangent(Math.toRadians(-90))
				.lineToY(WAYPOINTS_BLUE_FAR.SHOOT.position.y)
				.build();

		// SHOOT -> PICKUP
		Action goToPickupF = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.SHOOT)
				.setTangent(Math.toRadians(-90))
				.lineToY(WAYPOINTS_BLUE_FAR.PICKUPF.position.y)
				.build();

		Action goToPickup = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPF)
				.setTangent(Math.toRadians(0))
				.lineToX(WAYPOINTS_BLUE_FAR.PICKUP.position.x)
				.build();

		Action goToPickupL = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUP)
				.setTangent(Math.toRadians(0))
				.lineToX(WAYPOINTS_BLUE_FAR.PICKUPL.position.x)
				.build();

		// BACK TO SHOOT
		Action backToPickup = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPL)
				.setTangent(Math.toRadians(180))
				.lineToX(WAYPOINTS_BLUE_FAR.PICKUPF.position.x)
				.build();

		double tan = WAYPOINTS_BLUE_FAR.SHOOT.heading.toDouble();
		Action backToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPL)
				.splineToConstantHeading(
						new Vector2d(WAYPOINTS_BLUE_FAR.SHOOT.position.x, WAYPOINTS_BLUE_FAR.SHOOT.position.y),
						tan
				)
				.build();

		// ending location
		Action finishLine = robot.drivetrain.actionBuilder((WAYPOINTS_BLUE_FAR.SHOOT))
				.splineTo(
						new Vector2d(WAYPOINTS_BLUE_FAR.PARK.position.x, WAYPOINTS_BLUE_FAR.PARK.position.y),
						WAYPOINTS_BLUE_FAR.PARK.heading.toDouble()
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
						WaitFor(0.3),

						// Aim + tilt
						newAimTurretLL(),
						setAutonShooterAngle,
						WaitFor(0.3),

						shooter_on,
						WaitFor(1.0),

						// BALL 1
						shootArtifact,
						WaitFor(0.2),
						stopShooting,
						WaitFor(1.2),

						// BALL 2
						shootArtifact,
						WaitFor(0.28),
						stopShooting,
						WaitFor(1.2),

						// BALL 3

						shootArtifact,
						WaitFor(0.55),
						stopShooting,
						WaitFor(0.2),

						stopShooting,
						shooter_off,

						// pickup
						goToPickupF,
						WaitFor(0.2),

						goToPickup,
						WaitFor(0.1),

						startIntake,
						WaitFor(0.1),

						goToPickupL,
						WaitFor(0.6),

						stopIntake,
						WaitFor(0.4),


						backToShoot,
						WaitFor(0.35),

						// Aim + tilt
						newAimTurretLLFinal(),
						setAutonShooterAngle,
						WaitFor(0.5),

						shooter_on,
						WaitFor(1.0),

						// BALL 1
						shootArtifact,
						WaitFor(0.27),
						stopShooting,
						WaitFor(1.3),

						// BALL 2
						shootArtifact,
						WaitFor(0.25),
						stopShooting,
						WaitFor(1.2),

						// BALL 3
						shootArtifact,
						WaitFor(0.5),
						stopShooting,
						WaitFor(0.4),

						stopShooting,
						shooter_off,

						finishLine
				)
		);
	}
}
