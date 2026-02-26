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

@Autonomous(name = "🔵🔵Close_12+Open🔵🔵", group = "Auto")
public class Auto_BLUE_Close_12GATE extends BaseOpMode {
    private RobotHardware robot;

    private static final int AUTON_TURRET_TICKS = 200;
    private static final double AUTON_SHOOTER_POS = 0.21;
    private static final double AUTON_SHOOTER_POS_SECOND = 0.28;
    private static final double AUTON_SHOOTER_POS_THIRD = 0.28;
    private static final double AUTON_SHOOTER_POS_FINAL = 0.28;
    private static final double TURRET_HOLD_POWER = 0.1;
    private static final double SHOOT_SAFE_IN = 11.0;

    private boolean shooterEnabled = false;
    private double shooterRpmCmd = 2200;
    private boolean autonDone = false;

    private boolean started = false;

    @Override
    protected void OnInitialize() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.limelight.pipelineSwitch(0);
        robot.drivetrain.pose = WAYPOINTS_BLUE_CLOSE_EXP.START;

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
        if (robot == null || robot.turret == null) return;
        DcMotorEx turret = robot.turret.getMotor();
        int pos = turret.getCurrentPosition();
        turret.setTargetPosition(pos);
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
        return new AimTurretWithLimelightAction(this, robot, 0.045, 0.12, 0.40, 0.45, 400, 1.0, TURRET_HOLD_POWER, 120, 25);
    }

    private Action newAimTurretLLSecond() {
        return new AimTurretWithLimelightAction(this, robot, 0.045, 0.12, 0.40, 0.45, 400, 1.0, TURRET_HOLD_POWER, 120, 25);
    }

    private Action newAimTurretLLThird() {
        return new AimTurretWithLimelightAction(this, robot, 0.045, 0.12, 0.40, 0.45, 400, 1.0, TURRET_HOLD_POWER, 120, 25);
    }

    private Action newAimTurretLLFinal() {
        return new AimTurretWithLimelightAction(this, robot, 0.055, 0.12, 0.45, 0.70, 400, 1.0, TURRET_HOLD_POWER, 120, 25);
    }

    private static class AimTurretWithLimelightAction implements Action {
        private final Auto_BLUE_Close_12GATE op;
        private final RobotHardware robot;

        private final double kP;
        private final double minPower;
        private final double maxPower;
        private final double lockThresholdDeg;
        private final long timeoutMs;
        private final double directionSign;
        private final double holdPower;

        private final long noTargetGraceMs;
        private final long pollEveryMs;

        private boolean initialized = false;
        private long startTimeMs = 0;
        private long lastPollMs = 0;
        private LLResult cached;

        AimTurretWithLimelightAction(
                Auto_BLUE_Close_12GATE op,
                RobotHardware robot,
                double kP,
                double minPower,
                double maxPower,
                double lockThresholdDeg,
                long timeoutMs,
                double directionSign,
                double holdPower,
                long noTargetGraceMs,
                long pollEveryMs
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
            this.noTargetGraceMs = noTargetGraceMs;
            this.pollEveryMs = pollEveryMs;
        }

        @Override
        public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                startTimeMs = System.currentTimeMillis();
                lastPollMs = 0;
                cached = null;
                op.turretOpenLoopBrake();
                robot.turret.getMotor().setPower(0);
            }

            long now = System.currentTimeMillis();
            long elapsed = now - startTimeMs;

            packet.put("aimElapsedMs", elapsed);

            DcMotorEx turretMotor = robot.turret.getMotor();

            if (elapsed >= timeoutMs) {
                packet.put("aimEnd", "timeout");
                turretMotor.setPower(0);
                op.turretHoldCurrent(holdPower);
                return false;
            }

            if (robot.limelight == null) {
                packet.put("aimEnd", "noLL");
                turretMotor.setPower(0);
                op.turretHoldCurrent(holdPower);
                return false;
            }

            if (lastPollMs == 0 || (now - lastPollMs) >= pollEveryMs) {
                lastPollMs = now;
                try {
                    cached = robot.limelight.getLatestResult();
                } catch (Exception e) {
                    cached = null;
                }
            }

            boolean valid = cached != null && cached.isValid();
            packet.put("llValid", valid);

            if (!valid) {
                if (elapsed >= noTargetGraceMs) {
                    packet.put("aimEnd", "noTarget");
                    turretMotor.setPower(0);
                    op.turretHoldCurrent(holdPower);
                    return false;
                }
                turretMotor.setPower(0);
                return true;
            }

            double tx = cached.getTx();
            double absTx = Math.abs(tx);

            packet.put("tx", tx);

            if (absTx <= lockThresholdDeg) {
                packet.put("aimEnd", "locked");
                turretMotor.setPower(0);
                op.turretHoldCurrent(holdPower);
                return false;
            }

            double power = directionSign * (kP * tx);

            if (power > 0) power = Math.max(power, minPower);
            else power = Math.min(power, -minPower);

            if (power > maxPower) power = maxPower;
            if (power < -maxPower) power = -maxPower;

            packet.put("turretPwr", power);

            turretMotor.setPower(power);
            return true;
        }
    }

    private interface ActionFactory { Action build(); }

    private static class DeferredAction implements Action {
        private final ActionFactory factory;
        private Action built;

        DeferredAction(ActionFactory factory) { this.factory = factory; }

        @Override
        public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            if (built == null) built = factory.build();
            return built.run(packet);
        }
    }

    private Action defer(ActionFactory f) { return new DeferredAction(f); }

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
                long elapsed = System.currentTimeMillis() - start;
                packet.put("rpmWaitMs", elapsed);

                if (elapsed >= timeoutMs) return false;

                double r1 = robot.outtake1.getRpm();
                double r2 = robot.outtake2.getRpm();
                double avg = (r1 + r2) / 2.0;

                packet.put("rpmAvg", avg);
                packet.put("rpmTarget", targetRpm);

                return avg < targetRpm - tolRpm;
            }
        };
    }

    @Override
    protected void OnRun() {
        if (started) return;
        started = true;

        robot.drivetrain.updatePoseEstimate();

        Action startToShootBack = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
                        ))
                        .build()
        );

        Action shootToPickup1 = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP1.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP1.position.y
                        ))
                        .build()
        );

        Action shootToPickup2S = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2S.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2S.position.y
                        ))
                        .build()
        );

        Action pickup2SToPickup2 = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP2.position.y
                        ))
                        .build()
        );

        Action pickup2ToShoot = defer(() -> {
            double BACK_FIRST = 6.0;
            double currentX = robot.drivetrain.pose.position.x;

            return robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                    .lineToXConstantHeading(currentX - BACK_FIRST)
                    .splineToConstantHeading(
                            new Vector2d(
                                    WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x + 10,
                                    WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
                            ),
                            2.532
                    )
                    .build();
        });

        Action shootTopickup3S = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3S.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3S.position.y
                        ))
                        .build()
        );

        Action pickup3SToPickup3 = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.PICKUP3.position.y
                        ))
                        .build()
        );

        Action pickup3ToShoot = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x + 10,
                                WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
                        ))
                        .build()
        );

        Action pickupToOpengateS = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.y + 5
                        ))
                        .build()
        );

        Action opengateSToOpengate = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.OPENGATE.position.x,
                                WAYPOINTS_BLUE_CLOSE_EXP.OPENGATE.position.y
                        ))
                        .build()
        );

        Action opengateToShoot = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.x + 10,
                                WAYPOINTS_BLUE_CLOSE_EXP.SHOOT.position.y
                        ))
                        .build()
        );

        Action finishline = defer(() ->
                robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .strafeTo(new Vector2d(
                                WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.x - SHOOT_SAFE_IN,
                                WAYPOINTS_BLUE_CLOSE_EXP.OPENGATES.position.y
                        ))
                        .build()
        );

        Action shooterController = packet -> {
            if (autonDone) return false;
            if (shooterEnabled) {
                robot.outtake1.setRpm(shooterRpmCmd);
                robot.outtake2.setRpm(shooterRpmCmd);
            }
            packet.put("shooterOn", shooterEnabled);
            packet.put("shooterCmd", shooterRpmCmd);
            return true;
        };

        Action shooter_on = packet -> {
            shooterRpmCmd = 2200;
            shooterEnabled = true;

            robot.outtake1.triggerKick();
            robot.outtake2.triggerKick();

            robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
            turretHoldCurrent(TURRET_HOLD_POWER);
            return false;
        };

        Action shooter_on_slow = packet -> {
            shooterRpmCmd = 2150;
            shooterEnabled = true;

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
        Action stopIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP); return false; };

        Action endAuton = packet -> {
            autonDone = true;
            shooterEnabled = false;
            robot.outtake1.stop();
            robot.outtake2.stop();
            robot.transfer.setPower(0);
            robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
            return false;
        };

        Action turretToTicks = robot.turret.goToTicksAction(
                AUTON_TURRET_TICKS,
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

        Actions.runBlocking(
                RunInParallel(
                        shooterController,
                        RunSequentially(
                                turretToTicks,
                                RunInParallel(
                                        startToShootBack,
                                        shooter_on
                                ),

                                setAutonShooterAngle,
                                newAimTurretLL(),
                                waitUntilShooterRpm(shooterRpmCmd, 150, 500),
                                shootArtifact,
                                WaitFor(0.9),
                                stopShooting,
                                shooter_off,

                                startIntake,
                                shootToPickup1,
                                WaitFor(0.15),
                                stopIntake,

                                pickupToOpengateS,
                                WaitFor(0.1),
                                opengateSToOpengate,
                                WaitFor(0.3),

                                RunInParallel(shooter_on, opengateToShoot),
                                WaitFor(0.1),

                                setAutonShooterAngleSecond,
                                newAimTurretLLSecond(),
                                waitUntilShooterRpm(shooterRpmCmd, 150, 500),
                                shootArtifact,
                                WaitFor(0.9),
                                RunInParallel(stopShooting, shooter_off),

                                RunInParallel(
                                        shootToPickup2S,
                                        startIntake
                                ),
                                WaitFor(0.1),
                                pickup2SToPickup2,
                                WaitFor(0.15),
                                stopIntake,

                                RunInParallel(
                                        pickup2ToShoot,
                                        WaitFor(0.1),
                                        shooter_on
                                ),
                                WaitFor(0.2),

                                setAutonShooterAngleThird,
                                newAimTurretLLThird(),
                                waitUntilShooterRpm(shooterRpmCmd, 150, 500),
                                shootArtifact,
                                WaitFor(0.9),
                                RunInParallel(stopShooting, shooter_off),
                                WaitFor(0.1),

                                startIntake,
                                shootTopickup3S,
                                WaitFor(0.15),
                                pickup3SToPickup3,
                                stopIntake,
                                WaitFor(0.2),

                                RunInParallel(
                                        pickup3ToShoot,
                                        WaitFor(0.1),
                                        shooter_on_slow
                                ),
                                WaitFor(0.2),

                                setAutonShooterAngleFinal,
                                newAimTurretLLFinal(),
                                waitUntilShooterRpm(2150, 150, 500),
                                shootArtifact,
                                WaitFor(0.9),
                                stopShooting,
                                shooter_off,

                                turretHomeReset,
                                finishline,

                                endAuton
                        )
                )
        );
    }
}