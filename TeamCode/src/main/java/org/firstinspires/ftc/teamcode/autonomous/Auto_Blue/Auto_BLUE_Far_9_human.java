package org.firstinspires.ftc.teamcode.autonomous.Auto_Blue;

import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_BLUE_FAR;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "🔵🔵Far_9_Human🔵🔵", group = "Auto")
public class Auto_BLUE_Far_9_human extends BaseOpMode {

	private RobotHardware robot;

	private static final int AUTON_TURRET_TICKS = 20;
	private static final double AUTON_SHOOTER_POS = 0.47;
	private static final double TURRET_HOLD_POWER = 0.1;

	private static final double SHOOT_SAFE_IN = 8.0;
	private static final double SHOOT_SAFE_IN_2 = 15.0;
	private static final double SHOOT_SAFE_IN_START = 2.5;

	private boolean shooterEnabled = false;
	private double shooterRpmCmd = 4800;
	private boolean autonDone = false;

	// ===== Shooter debug timing (ca Far_9) =====
	private long shooterOnMs = -1;
	private int shootBurstIndex = 0;

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();
		robot.limelight.pipelineSwitch(0);

		OuttakeSystem.TICKS_PER_REV = 28;

		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		turretHoldCurrent(TURRET_HOLD_POWER);
	}

	@Override
	protected void jOnInitialize() {}

	private void turretHoldCurrent(double holdPower) {
		DcMotorEx turret = robot.turret.getMotor();
		turret.setTargetPosition(turret.getCurrentPosition());
		turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		turret.setPower(holdPower);
	}

	private void turretOpenLoopBrake() {
		DcMotorEx turret = robot.turret.getMotor();
		turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private Action newAimTurretLL() {
		return new AimTurretWithLimelightAction(this, robot, 0.035, 0.07, 0.5, 0.30, 750, 1.0, TURRET_HOLD_POWER);
	}
	private Action newAimTurretLLNEW() {
		return new AimTurretWithLimelightAction(this, robot, 0.035, 0.07, 0.5, 0.30, 750, 1.0, TURRET_HOLD_POWER);
	}
	private Action newAimTurretLLSecond() { return newAimTurretLL(); }
	private Action newAimTurretLLFinal()  { return newAimTurretLL(); }

	/** Wait până când avgRPM e aproape de target (ca în Far_9), cu timeout. */
	private Action waitUntilShooterRpm(double targetRpm, double tolRpm, long timeoutMs) {
		return new Action() {
			boolean init = false;
			long start;

			@Override
			public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
				if (!init) {
					init = true;
					start = System.currentTimeMillis();
				}
				if (System.currentTimeMillis() - start >= timeoutMs) return false;

				double r1 = robot.outtake1.getRpm();
				double r2 = robot.outtake2.getRpm();
				double avg = (r1 + r2) / 2.0;

				packet.put("WAIT target", targetRpm);
				packet.put("WAIT avg", avg);
				packet.put("WAIT err", targetRpm - avg);
				packet.put("WAIT tol", tolRpm);

				return avg < targetRpm - tolRpm;
			}
		};
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_BLUE_Far_9_human op;
		private final RobotHardware robot;
		private final double kP, minPower, maxPower, lockDeg, dir, hold;
		private final long timeout;
		private boolean init = false;
		private long start;

		AimTurretWithLimelightAction(
				Auto_BLUE_Far_9_human op,
				RobotHardware robot,
				double kP,
				double minPower,
				double maxPower,
				double lockDeg,
				long timeout,
				double dir,
				double hold
		) {
			this.op = op;
			this.robot = robot;
			this.kP = kP;
			this.minPower = minPower;
			this.maxPower = maxPower;
			this.lockDeg = lockDeg;
			this.timeout = timeout;
			this.dir = dir;
			this.hold = hold;
		}

		@Override
		public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
			if (!init) {
				init = true;
				start = System.currentTimeMillis();
				op.turretOpenLoopBrake();
				robot.turret.getMotor().setPower(0);
			}

			if (System.currentTimeMillis() - start >= timeout) {
				op.turretHoldCurrent(hold);
				return false;
			}

			LLResult r = robot.limelight.getLatestResult();
			if (r == null || !r.isValid()) {
				robot.turret.getMotor().setPower(0);
				return true;
			}

			double tx = r.getTx();
			if (Math.abs(tx) <= lockDeg) {
				op.turretHoldCurrent(hold);
				return false;
			}

			double p = dir * kP * tx;
			p = Math.max(Math.min(p, maxPower), -maxPower);
			if (p > 0) p = Math.max(p, minPower);
			else       p = Math.min(p, -minPower);

			robot.turret.getMotor().setPower(p);
			return true;
		}
	}

	@Override
	protected void OnRun() {

		robot.drivetrain.updatePoseEstimate();

		// ===== PATHS (PASTRATE EXACT ca la tine) =====

		Action goToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.START)
				.setTangent(Math.toRadians(90))
				.lineToY(WAYPOINTS_BLUE_FAR.SHOOT.position.y)
				.build();

		Action finishline = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.SHOOT)
				.setTangent(Math.toRadians(90))
				.lineToY(WAYPOINTS_BLUE_FAR.FINISHLINE.position.y)
				.build();



		AccelConstraint humanAccel = new ProfileAccelConstraint(
				MecanumDrive.PARAMS.minProfileAccel,
				35
		);

		Action HumanPark = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.START, humanAccel)
				.lineToX(WAYPOINTS_BLUE_FAR.HUMAN.position.x)
				.build();

		Action HumanPark2 = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.START, humanAccel)
				.lineToX(WAYPOINTS_BLUE_FAR.HUMAN.position.x)
				.build();

		Action HumanPark3 =
				robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.SHOOT)
						.strafeTo(new Vector2d(
								WAYPOINTS_BLUE_FAR.HUMAN.position.x,
								WAYPOINTS_BLUE_FAR.HUMAN.position.y
						))
						.build();

		Action goToPickup = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPF)
				.setTangent(Math.toRadians(0))
				.lineToX(WAYPOINTS_BLUE_FAR.PICKUP.position.x)
				.build();

		AccelConstraint pickupAccel = new ProfileAccelConstraint(
				MecanumDrive.PARAMS.minProfileAccel,
				30
		);

		Action goToPickupL = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUP, pickupAccel)
				.setTangent(Math.toRadians(0))
				.lineToX(WAYPOINTS_BLUE_FAR.PICKUPL.position.x)
				.build();

		AccelConstraint backstartAccel = new ProfileAccelConstraint(
				MecanumDrive.PARAMS.minProfileAccel,
				20
		);

		Action backToStart = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.SHOOT, backstartAccel)
				.setTangent(Math.toRadians(90))
				.lineToY(WAYPOINTS_BLUE_FAR.START.position.y)
				.build();

		Action backToStart2 = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.SHOOT, backstartAccel)
				.setTangent(Math.toRadians(90))
				.lineToY(WAYPOINTS_BLUE_FAR.START.position.y)
				.build();

		Action HumanToStart = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.HUMAN)
				.splineToConstantHeading(
						new Vector2d(
								WAYPOINTS_BLUE_FAR.SHOOT.position.x + SHOOT_SAFE_IN,
								WAYPOINTS_BLUE_FAR.SHOOT.position.y
						),
						WAYPOINTS_BLUE_FAR.SHOOT.heading.toDouble()
				)
				.build();

		Action HumanToStart2 = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.HUMAN)
				.splineToConstantHeading(
						new Vector2d(
								WAYPOINTS_BLUE_FAR.SHOOT.position.x + SHOOT_SAFE_IN,
								WAYPOINTS_BLUE_FAR.SHOOT.position.y
						),
						WAYPOINTS_BLUE_FAR.SHOOT.heading.toDouble()
				)
				.build();

		Action backToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPL)
				.splineToConstantHeading(
						new Vector2d(
								WAYPOINTS_BLUE_FAR.SHOOT.position.x + SHOOT_SAFE_IN_2,
								WAYPOINTS_BLUE_FAR.SHOOT.position.y - SHOOT_SAFE_IN_START
						),
						WAYPOINTS_BLUE_FAR.SHOOT.heading.toDouble()
				)
				.build();

		// ===== SHOOTER / INTAKE (ca Far_9) =====

		Action shooterController = packet -> {
			if (autonDone) return false;

			packet.put("PoseX", robot.drivetrain.pose.position.x);
			packet.put("PoseY", robot.drivetrain.pose.position.y);
			packet.put("PoseHdeg", Math.toDegrees(robot.drivetrain.pose.heading.log()));

			if (shooterEnabled) {
				robot.outtake1.setRpm(shooterRpmCmd);
				robot.outtake2.setRpm(shooterRpmCmd);
			}

			double r1 = robot.outtake1.getRpm();
			double r2 = robot.outtake2.getRpm();
			double avg = (r1 + r2) / 2.0;

			packet.put("Shooter Enabled", shooterEnabled);
			packet.put("Shooter TargetRPM", shooterRpmCmd);
			packet.put("OT1 RPM", r1);
			packet.put("OT2 RPM", r2);
			packet.put("AVG RPM", avg);
			packet.put("RPM Error", shooterRpmCmd - avg);
			packet.put("Shooter READY", avg >= shooterRpmCmd - 150);

			packet.put("PWR OT1", robot.outtake1.raw().getPower());
			packet.put("PWR OT2", robot.outtake2.raw().getPower());

			if (shooterOnMs > 0) packet.put("Shooter ON ms", System.currentTimeMillis() - shooterOnMs);

			LLResult ll = robot.limelight.getLatestResult();
			boolean llValid = (ll != null && ll.isValid());
			packet.put("LL valid", llValid);
			if (llValid) packet.put("LL tx", ll.getTx());

			packet.put("Burst idx", shootBurstIndex);

			return true;
		};

		Action shooter_on = packet -> {
			shooterEnabled = true;
			shooterOnMs = System.currentTimeMillis();

			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
			turretHoldCurrent(TURRET_HOLD_POWER);

			packet.put("EVENT", "shooter_on");
			return false;
		};

		Action shooter_off = packet -> {
			shooterEnabled = false;
			robot.outtake1.stop();
			robot.outtake2.stop();
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);

			packet.put("EVENT", "shooter_off");
			return false;
		};

		Action shootArtifact = packet -> {
			shootBurstIndex++;
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.transfer.setPower(-1);
			packet.put("EVENT", "shootArtifact");
			return false;
		};

		Action stopShooting = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.transfer.setPower(0);
			packet.put("EVENT", "stopShooting");
			return false;
		};

		Action turretToTicks = robot.turret.goToTicksAction(
				212, // target
				0.6,                // power (0..1)
				8,                  // toleranta ticks
				1200,               // timeout ms
				TURRET_HOLD_POWER   // hold power dupa ce ajunge
		);

		Action turretHomeReset = robot.turret.goToZeroAndResetAction(
				0.6,                // power
				8,                  // toleranta
				1500,               // timeout ms
				TURRET_HOLD_POWER   // hold power
		);

		Action startIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD); packet.put("EVENT", "startIntake"); return false; };
		Action stopIntake  = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);    packet.put("EVENT", "stopIntake");  return false; };

		Action setAutonShooterAngle = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS); packet.put("EVENT", "setShooterAngle"); return false; };

		// ===== RUN =====
		Actions.runBlocking(
				RunInParallel(
						shooterController,
						RunSequentially(

								turretToTicks,
								// ===== SHOOT 1 =====
								RunInParallel(
										packet -> { shooterRpmCmd = 4800; return shooter_on.run(packet); },
										goToShoot
								),
								waitUntilShooterRpm(4800, 150, 700),

								newAimTurretLL(),
								setAutonShooterAngle,

								// burst 1
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								WaitFor(1.0),

								stopShooting,
								shooter_off,
								WaitFor(0.2),

								// ===== HUMAN CYCLE 1 =====
								backToStart,
								startIntake, WaitFor(0.1),
								HumanPark, WaitFor(0.6),
								stopIntake,

								// ===== SHOOT 2 =====
								RunInParallel(
										HumanToStart,
										packet -> { shooterRpmCmd = 4800; return shooter_on.run(packet); }
								),
								waitUntilShooterRpm(4800, 150, 700),

								newAimTurretLLSecond(),
								setAutonShooterAngle,

								// burst 2
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								WaitFor(1.0),

								stopShooting,
								shooter_off,
								WaitFor(0.2),

								// ===== HUMAN CYCLE 2 (pastrez cum ai tu) =====
								backToStart2,
								startIntake, WaitFor(0.2),
								HumanPark2, WaitFor(0.6),
								stopIntake, WaitFor(0.1),

								// ===== SHOOT 3 =====
								RunInParallel(
										HumanToStart2,
										packet -> { shooterRpmCmd = 4800; return shooter_on.run(packet); }
								),
								waitUntilShooterRpm(4800, 150, 700),

								newAimTurretLLNEW(),
								setAutonShooterAngle,

								// burst 3
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
								shootArtifact, WaitFor(0.30), stopShooting, WaitFor(0.25),
								WaitFor(1.0),

								stopShooting,
								shooter_off,

								// ===== FINAL PARK (cum ai pus) =====

								RunInParallel(turretHomeReset, finishline)
						)
				)
		);

		autonDone = true;
	}
}