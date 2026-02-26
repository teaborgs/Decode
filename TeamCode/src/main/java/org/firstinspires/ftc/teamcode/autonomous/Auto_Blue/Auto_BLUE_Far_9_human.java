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

    private static final double AUTON_SHOOTER_POS = 0.44;
    private static final double TURRET_HOLD_POWER = 0.1;

    private static final double SHOOT_SAFE_IN = 8.0;
    private static final double SHOOT_SAFE_IN_2 = 15.0;
    private static final double SHOOT_SAFE_IN_START = 2.5;

    private boolean shooterEnabled = false;
    private double shooterRpmCmd = 3000;
    private boolean autonDone = false;

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
    private Action newAimTurretLLSecond() { return newAimTurretLL(); }
    private Action newAimTurretLLFinal()  { return newAimTurretLL(); }

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

        Action goToShoot = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.START)
                .setTangent(Math.toRadians(90))
                .lineToY(WAYPOINTS_BLUE_FAR.SHOOT.position.y + 4)
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

        AccelConstraint pickupAccel = new ProfileAccelConstraint(
                MecanumDrive.PARAMS.minProfileAccel,
                30
        );

        Action goToPickup = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.PICKUPF)
                .setTangent(Math.toRadians(0))
                .lineToX(WAYPOINTS_BLUE_FAR.PICKUP.position.x)
                .build();

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
                                WAYPOINTS_BLUE_FAR.SHOOT.position.x + 13.5,
                                WAYPOINTS_BLUE_FAR.SHOOT.position.y
                        ),
                        WAYPOINTS_BLUE_FAR.SHOOT.heading.toDouble()
                )
                .build();

        Action HumanToStart2 = robot.drivetrain.actionBuilder(WAYPOINTS_BLUE_FAR.HUMAN)
                .splineToConstantHeading(
                        new Vector2d(
                                WAYPOINTS_BLUE_FAR.SHOOT.position.x + 13.5,
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

        Action shooterController = packet -> {
            if (autonDone) return false;

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

            robot.outtake1.triggerKick();
            robot.outtake2.triggerKick();

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
            shootBurstIndex++;
            robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
            robot.transfer.setPower(-1);
            return false;
        };

        Action stopShooting = packet -> {
            robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
            robot.transfer.setPower(0);
            return false;
        };

        Action turretToTicks = robot.turret.goToTicksAction(
                212,
                0.6,
                8,
                1200,
                TURRET_HOLD_POWER
        );

        Action turretHomeReset = robot.turret.goToZeroAndResetAction(
                0.6,
                8,
                1500,
                TURRET_HOLD_POWER
        );

        Action startIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD); return false; };
        Action stopIntake  = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);    return false; };

        Action setAutonShooterAngle = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS); return false; };

        Actions.runBlocking(
                RunInParallel(
                        shooterController,
                        RunSequentially(
                                turretToTicks,

                                RunInParallel(
                                        packet -> { shooterRpmCmd = 3000; return shooter_on.run(packet); },
                                        goToShoot
                                ),
                                waitUntilShooterRpm(shooterRpmCmd, 150, 700),

                                newAimTurretLL(),
                                setAutonShooterAngle,

                                shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
                                shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
                                shootArtifact,
                                WaitFor(0.5),

                                stopShooting,
                                shooter_off,
                                WaitFor(0.2),

                                backToStart,
                                startIntake, WaitFor(0.1),
                                HumanPark, WaitFor(0.6),
                                stopIntake,

                                RunInParallel(
                                        HumanToStart,
                                        packet -> { shooterRpmCmd = 3000; return shooter_on.run(packet); }
                                ),
                                waitUntilShooterRpm(shooterRpmCmd, 150, 700),

                                newAimTurretLLSecond(),
                                setAutonShooterAngle,

                                shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
                                shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
                                shootArtifact,
                                WaitFor(0.5),

                                stopShooting,
                                shooter_off,
                                WaitFor(0.2),

                                backToStart2,
                                startIntake, WaitFor(0.2),
                                HumanPark2, WaitFor(0.6),
                                stopIntake, WaitFor(0.05),

                                RunInParallel(
                                        HumanToStart2,
                                        packet -> { shooterRpmCmd = 3000; return shooter_on.run(packet); }
                                ),
                                waitUntilShooterRpm(shooterRpmCmd, 150, 700),

                                newAimTurretLLFinal(),
                                setAutonShooterAngle,

                                shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
                                shootArtifact, WaitFor(0.25), stopShooting, WaitFor(0.25),
                                shootArtifact,
                                WaitFor(0.5),

                                stopShooting,
                                shooter_off,
                                WaitFor(0.1),

                                RunInParallel(turretHomeReset, finishline)
                        )
                )
        );

        autonDone = true;
    }
}