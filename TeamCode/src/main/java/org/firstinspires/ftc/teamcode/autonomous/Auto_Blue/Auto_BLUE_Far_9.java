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

	@Autonomous(name = "🔵🔵Far_9🔵🔵", group = "Auto")
	public class Auto_BLUE_Far_9 extends BaseOpMode {

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

		@Override protected void jOnInitialize() {}

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
			return new AimTurretWithLimelightAction(this, robot, 0.035,
					0.07,
					0.5,
					0.30,
					750,
					-1.0,
					TURRET_HOLD_POWER);
		}
		private Action newAimTurretLLSecond() { return newAimTurretLL(); }
		private Action newAimTurretLLFinal() { return newAimTurretLL(); }

		private static class AimTurretWithLimelightAction implements Action {
			private final Auto_BLUE_Far_9 op;
			private final RobotHardware robot;
			private final double kP, minPower, maxPower, lockDeg, dir, hold;
			private final long timeout;
			private boolean init = false;
			private long start;

			AimTurretWithLimelightAction(Auto_BLUE_Far_9 op, RobotHardware robot,
										 double kP, double minPower, double maxPower,
										 double lockDeg, long timeout, double dir, double hold) {
				this.op = op; this.robot = robot; this.kP = kP;
				this.minPower = minPower; this.maxPower = maxPower;
				this.lockDeg = lockDeg; this.timeout = timeout;
				this.dir = dir; this.hold = hold;
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
				else p = Math.min(p, -minPower);

				robot.turret.getMotor().setPower(p);
				return true;
			}
		}

		@Override
		protected void OnRun() {

			robot.drivetrain.updatePoseEstimate();

			Action goToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.START)
					.setTangent(Math.toRadians(90))
					.lineToY(WAYPOINTS_BLUE_FAR.SHOOT.position.y).build();

			AccelConstraint humanAccel = new ProfileAccelConstraint(
					MecanumDrive.PARAMS.minProfileAccel,
					25 // aici pui maxProfileAccel doar pt HumanPark
			);

			Action HumanPark = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.START, humanAccel)
					.lineToX(WAYPOINTS_BLUE_FAR.HUMAN.position.x)
					.build();


			Action goToPickup = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPF)
					.setTangent(Math.toRadians(0))
					.lineToX(WAYPOINTS_BLUE_FAR.PICKUP.position.x).build();

			AccelConstraint pickupAccel = new ProfileAccelConstraint(
					MecanumDrive.PARAMS.minProfileAccel,
					20  // aici pui maxProfileAccel doar pt HumanPark
			);

			Action goToPickupL = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUP, pickupAccel)
					.setTangent(Math.toRadians(0))
					.lineToX(WAYPOINTS_BLUE_FAR.PICKUPL.position.x).build();

			AccelConstraint backstartAccel = new ProfileAccelConstraint(
					MecanumDrive.PARAMS.minProfileAccel,
					10  // aici pui maxProfileAccel doar pt HumanPark
			);

			Action backToStart = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.SHOOT, backstartAccel)
					.setTangent(Math.toRadians(90))
					.lineToY(WAYPOINTS_BLUE_FAR.START.position.y).build();


			Action HumanToStart = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.HUMAN)
					.splineToConstantHeading(
							new Vector2d(WAYPOINTS_BLUE_FAR.SHOOT.position.x + SHOOT_SAFE_IN,
									WAYPOINTS_BLUE_FAR.SHOOT.position.y),
							WAYPOINTS_BLUE_FAR.SHOOT.heading.toDouble()).build();

			Action backToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPL)
					.splineToConstantHeading(
							new Vector2d(WAYPOINTS_BLUE_FAR.SHOOT.position.x + SHOOT_SAFE_IN_2,
									WAYPOINTS_BLUE_FAR.SHOOT.position.y+SHOOT_SAFE_IN_START),
							WAYPOINTS_BLUE_FAR.SHOOT.heading.toDouble()).build();

			Action shooterController = packet -> {
				if (autonDone) return false;
				if (shooterEnabled) {
					robot.outtake1.setRpm(shooterRpmCmd);
					robot.outtake2.setRpm(shooterRpmCmd);
				}
				return true;
			};

			Action shooter_on = packet -> {
				shooterRpmCmd = 4800;
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

			Action startIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD); return false; };
			Action stopIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP); return false; };

			Action setAutonShooterAngle = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS); return false; };

			Actions.runBlocking(
					RunInParallel(
							shooterController,
							RunSequentially(

									RunInParallel(shooter_on, goToShoot),
									WaitFor(0.3),

									newAimTurretLL(),
									setAutonShooterAngle,
									WaitFor(0.2),



									shootArtifact, WaitFor(0.5), stopShooting, WaitFor(0.35),
									shootArtifact, WaitFor(0.5), stopShooting, WaitFor(0.35),
									shootArtifact, WaitFor(0.5), stopShooting, WaitFor(0.35),

									stopShooting, shooter_off, WaitFor(0.2),

									backToStart,
									startIntake, WaitFor(0.1),
									HumanPark, WaitFor(0.7),
									stopIntake,

									RunInParallel(HumanToStart, shooter_on ),
									newAimTurretLLSecond(),
									setAutonShooterAngle, WaitFor(0.2),

									shootArtifact, WaitFor(0.5), stopShooting, WaitFor(0.35),
									shootArtifact, WaitFor(0.5), stopShooting, WaitFor(0.35),
									shootArtifact, WaitFor(0.5), stopShooting,
									shooter_off,

									startIntake, WaitFor(0.2),
									goToPickup, WaitFor(0.15),
									goToPickupL, WaitFor(0.2),
									stopIntake, WaitFor(0.1),

									RunInParallel(backToShoot, shooter_on ),
									newAimTurretLLFinal(),
									setAutonShooterAngle, WaitFor(0.3),

									shootArtifact, WaitFor(0.5), stopShooting, WaitFor(0.35),
									shootArtifact, WaitFor(0.5), stopShooting, WaitFor(0.35),
									shootArtifact, WaitFor(0.5), stopShooting,
									shooter_off
							)
					)
			);
		}
	}
