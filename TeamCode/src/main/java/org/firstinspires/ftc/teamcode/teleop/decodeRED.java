package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@TeleOp(name = "🔴🔴decodeaza-mi-l🔴🔴", group = "TeleOp")
public final class decodeRED extends BaseOpMode {
    private InputSystem driveInput, armInput;

    private RobotHardware robot;

    private static class Keybindings {
        public static class Drive {
            // shoot / intake
            public static final InputSystem.Key SHOOTER_KEY = new InputSystem.Key("a"); // hold = shoot
            public static final InputSystem.Key INTAKE_REVERSE_KEY = new InputSystem.Key("b");
            public static final InputSystem.Key INTAKE_KEY = new InputSystem.Key("right_bumper");

            // shooter angle manual
            public static final InputSystem.Axis SHOOTER_ANGLE = new InputSystem.Axis("right_stick_y");

            // drivetrain
            public static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
            public static final InputSystem.Axis DRIVE_X = new InputSystem.Axis("left_stick_x");
            public static final InputSystem.Axis DRIVE_Y = new InputSystem.Axis("left_stick_y");
            public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
            public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");

            // turret manual axis (când autoAim e OFF)
            public static final InputSystem.Axis TURRET_MANUAL = new InputSystem.Axis("right_stick_x");

            // reset/select anchor
            public static final InputSystem.Key RESET_ANCHOR_A = new InputSystem.Key("dpad_left");
            public static final InputSystem.Key RESET_ANCHOR_B = new InputSystem.Key("dpad_right");

            // toggle auto aim
            public static final InputSystem.Key AIM_TOGGLE = new InputSystem.Key("dpad_up");
            // reset turret encoder
            public static final InputSystem.Key RESET_TURRET_ENCODER = new InputSystem.Key("dpad_down");

            // auto shooter angle toggle
            public static final InputSystem.Key AUTO_SHOOTER_ENABLE_KEY = new InputSystem.Key("start");
            public static final InputSystem.Key AUTO_SHOOTER_DISABLE_KEY = new InputSystem.Key("back");
        }
    }

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

    // =========================
    // GateV4 constants + state
    // =========================

    // ===== Reset pose (odometrie) =====
    private static final double RESET_X_IN = 0.0;
    private static final double RESET_Y_IN = 0.0;
    private static final double RESET_H_DEG = 0.0;

    // ===== 2 anchors (ținte) selectabile =====
    private static final double ANCHOR_A_X = -2.78;
    private static final double ANCHOR_A_Y = 52.13;

    private static final double ANCHOR_B_X = 41.06;
    private static final double ANCHOR_B_Y = 114.76;

    private boolean anchorAActive = false;
    private boolean anchorBActive = false;

    // ===== Turret mechanical limits (deg intention) =====
    private static final double TURRET_LIMIT_DEG = 170.0;
    private static final double TURRET_RETURN_OK_DEG = 165.0; // hysteresis

    // ===== Ticks safety =====
    private static final int TICKS_BUFFER = 10;

    // ===== PID in ticks (fallback / odometry) =====
    private static final double kP = 0.0048;
    private static final double kD = 0.0009;
    private static final double MAX_POWER = 0.50;
    private static final int ON_TARGET_TICKS = 6;

    // ===== PID in ticks (LL aggressive) =====
    private static final double kP_LL = 0.0060;
    private static final double kD_LL = 0.0012;
    private static final double MAX_POWER_LL = 0.60;

    // ===== turret ticks limits (LOGICE) =====
    // Acestea rămân “cum le-ai calibrat tu” în coordonata logică.
    private static final int turretMinTicks = -472;
    private static final int turretMaxTicks =  438;

    private static final double degPerTick = 340.0 / 910.0; // ≈ 0.373626

    // ==========================================================
    // UNICUL LOC unde reparăm semnul la putere + semnul la encoder
    // ==========================================================
    //
    // Dacă în TurretSystem ai setDirection(REVERSE) (cum e acum în codul tău),
    // atunci HW_SIGN trebuie să fie -1 ca “logica” să rămână consistentă.
    //
    // Dacă vei schimba TurretSystem pe FORWARD, pune HW_SIGN = +1.
    //
    private static final int TURRET_HW_SIGN = 1;

    private int getTurretTicks() {
        // ticks “LOGICE”
        return TURRET_HW_SIGN * robot.turret.getMotor().getCurrentPosition();
    }

    private void setTurretPowerLogical(double logicalPower) {
        // power “LOGIC” (în coordonata logică)
        robot.turret.getMotor().setPower(TURRET_HW_SIGN * logicalPower);
    }

    // ===== Limelight FULL override (tx -> turret) =====
    private static final double LL_DZ_STOP  = 1.2;
    private static final double LL_DZ_START = 2.0;

    private static final double LL_DEG_PER_TX = 1.8;
    private static final double LL_MAX_STEP_DEG = 14.0;

    private static final double LL_HOLD_SEC = 0.20;
    private static final double LL_DISABLE_NEAR_LIMIT_DEG = 160.0;

    // Semnul de “tx -> stânga/dreapta” (dacă LL se duce invers, schimbă aici)
    private static final double LL_TURN_SIGN = +1.0;

    // ===== State =====
    private boolean autoAim = false;
    private boolean aimInvalid = false;

    private boolean prevResetAnchorA = false;
    private boolean prevResetAnchorB = false;
    private boolean prevAimToggle = false;
    private boolean prevResetTurretEncoder = false;

    private int targetTicks = 0;
    private double errPrev = 0.0;
    private final ElapsedTime dtTimer = new ElapsedTime();

    // LL state
    private boolean llAligned = false;
    private boolean llHasLock = false;
    private double llLastSeenSec = -1.0;

    // debug
    private boolean useLL_dbg = false;
    private double tx_dbg = 0.0;
    private double turretCmdDeg_dbg = 0.0;
    private double turretDesiredDeg_dbg = 0.0;
    private double targetAngleFieldDeg_dbg = 0.0;
    private double activeAnchorX_dbg = 0.0;
    private double activeAnchorY_dbg = 0.0;

    // ================= SHOOTER / OUTTAKE RPM =================
    private boolean transfer_servo = false;

    private double shooterTargetRpm = 4500;
    private final double shooterTargetRpmNear = 3850;
    private final double shooterTargetRpmFar  = 3400;

    // ================= SHOOTER ANGLE =================
    private static final double SHOOTER_DEADZONE = 0.15;
    private double shooterPosition = 0.5;

    // ===== LL distance memory =====
    private double lastDistance = 0.0;
    private boolean hasLastDistance = false;

    private boolean autoShooterAngle = true;
    private boolean prevAutoShooterKeyPressed = false;
    private boolean prevAutoAngleKeyPressed = false;

    @Override
    protected void OnStart() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        OuttakeSystem.TICKS_PER_REV = 28;

        robot.limelight.pipelineSwitch(1);

        // turret encoder reset
        DcMotorEx turretMotor = robot.turret.getMotor();
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reset pose default
        setPoseInches(RESET_X_IN, RESET_Y_IN, RESET_H_DEG);

        // default anchor A
        anchorAActive = true;
        anchorBActive = false;

        // START: turret manual
        autoAim = false;
        aimInvalid = false;
        llAligned = false;
        llHasLock = false;
        llLastSeenSec = -1.0;
        errPrev = 0.0;
        dtTimer.reset();

        shooterPosition = robot.turretTumbler.getPosition();
    }

    @Override
    protected void OnRun() {
        if (robot != null) {
            robot.drivetrain.updatePoseEstimate();
        }


        boolean resetA = driveInput.isPressed(Keybindings.Drive.RESET_ANCHOR_A);
        if (resetA && !prevResetAnchorA) {
            setPoseInches(RESET_X_IN, RESET_Y_IN, RESET_H_DEG);

            anchorAActive = true;
            anchorBActive = false;
        }
        prevResetAnchorA = resetA;


        boolean resetB = driveInput.isPressed(Keybindings.Drive.RESET_ANCHOR_B);
        if (resetB && !prevResetAnchorB) {
            setPoseInches(RESET_X_IN, RESET_Y_IN, RESET_H_DEG);

            anchorAActive = false;
            anchorBActive = true;
        }
        prevResetAnchorB = resetB;

        // === DPAD_DOWN: RESET TURRET ENCODER (failsafe) ===
        boolean resetTurret = driveInput.isPressed(Keybindings.Drive.RESET_TURRET_ENCODER);
        if (resetTurret && !prevResetTurretEncoder) {

            DcMotorEx turretMotor = robot.turret.getMotor();

            turretMotor.setPower(0);

            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            targetTicks = 0;
            errPrev = 0.0;
            dtTimer.reset();
        }
        prevResetTurretEncoder = resetTurret;

        // === DPAD_UP: toggle autoAim ===
        boolean aimToggle = driveInput.isPressed(Keybindings.Drive.AIM_TOGGLE);
        if (aimToggle && !prevAimToggle) {
            autoAim = !autoAim;

            aimInvalid = false;
            llAligned = false;
            llHasLock = false;
            llLastSeenSec = -1.0;

            errPrev = 0.0;
            dtTimer.reset();
        }
        prevAimToggle = aimToggle;

        // === TOGGLE autoShooterAngle with START/BACK ===
        boolean autoShooterKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_ENABLE_KEY);
        if (autoShooterKey && !prevAutoShooterKeyPressed) {
            autoShooterAngle = !autoShooterAngle;
        }
        prevAutoShooterKeyPressed = autoShooterKey;

        boolean autoAngleKey = driveInput.isPressed(Keybindings.Drive.AUTO_SHOOTER_DISABLE_KEY);
        if (autoAngleKey && !prevAutoAngleKeyPressed) {
            autoShooterAngle = !autoShooterAngle;
        }
        prevAutoAngleKeyPressed = autoAngleKey;

        Drive();
        TurretAimV4_IN_TELEOP();
        Shooter();
        ShooterAngle();
        Intake();
    }

    // ================= DRIVE =================
    private void Drive() {
        float speed = 1f;
        if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;

        robot.drivetrain.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                driveInput.getValue(Keybindings.Drive.DRIVE_Y),
                                driveInput.getValue(Keybindings.Drive.DRIVE_X)
                        ).times(-speed),
                        (driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
                )
        );
    }

    // ================= SHOOTER / OUTTAKE RPM =================
    private boolean prevTransfer = false;

    private void Shooter() {

        boolean shooting = driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY);

        if (shooting) {

            robot.outtake1.setRpm(shooterTargetRpm);
            robot.outtake2.setRpm(shooterTargetRpm);

            if (!prevTransfer) {
                robot.outtake1.triggerKick();
                robot.outtake2.triggerKick();
            }

            transfer_servo = true;
            robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

        } else {

            robot.outtake1.stop();
            robot.outtake2.stop();

            transfer_servo = false;
            robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
        }

        prevTransfer = shooting;
    }

    // ================= TURRET AIM =================
    private static final double TURRET_MANUAL_DEADZONE = 0.12;

    private void TurretAimV4_IN_TELEOP() {
        DcMotorEx turret = robot.turret.getMotor();

        // MANUAL când autoAim e OFF
        if (!autoAim) {
            double x = driveInput.getValue(Keybindings.Drive.TURRET_MANUAL);

            if (Math.abs(x) < TURRET_MANUAL_DEADZONE) {
                setTurretPowerLogical(0);
                return;
            }

            setTurretPowerLogical(clamp(x, -0.6, 0.6));
            return;
        }

        // ======================
        // (A) Fallback (anchor) - SELECTABIL A/B
        // ======================
        Pose2d p = robot.drivetrain.pose;
        double robotX = p.position.x;
        double robotY = p.position.y;
        double robotHeadingDeg = Math.toDegrees(p.heading.log());

        double ax = anchorAActive ? ANCHOR_A_X : ANCHOR_B_X;
        double ay = anchorAActive ? ANCHOR_A_Y : ANCHOR_B_Y;
        activeAnchorX_dbg = ax;
        activeAnchorY_dbg = ay;

        double dx = ax - robotX;
        double dy = -(ay - robotY);

        double targetAngleFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        targetAngleFieldDeg_dbg = targetAngleFieldDeg;

        double turretDesiredDeg = wrapDeg(targetAngleFieldDeg + robotHeadingDeg);
        turretDesiredDeg_dbg = turretDesiredDeg;

        if (!aimInvalid) {
            if (Math.abs(turretDesiredDeg) > TURRET_LIMIT_DEG) aimInvalid = true;
        } else {
            if (Math.abs(turretDesiredDeg) < TURRET_RETURN_OK_DEG) aimInvalid = false;
        }

        // ======================
        // (B) Limelight override (tx) + hold
        // ======================
        LLResult r = robot.limelight.getLatestResult();
        boolean llValid = (r != null && r.isValid());

        double nowSec = getRuntime();
        if (llValid) {
            llLastSeenSec = nowSec;
            llHasLock = true;
            tx_dbg = r.getTx();
        } else {
            tx_dbg = 0.0;
            if (llHasLock && llLastSeenSec >= 0 && (nowSec - llLastSeenSec) > LL_HOLD_SEC) {
                llHasLock = false;
                llAligned = false;
            }
        }

        int nowTicks = getTurretTicks();
        double nowTurretDeg = nowTicks * degPerTick;
        boolean nearLimit = (Math.abs(nowTurretDeg) >= LL_DISABLE_NEAR_LIMIT_DEG);

        boolean useLL = llHasLock && !nearLimit;
        useLL_dbg = useLL;

        double turretCmdDeg;

        if (useLL) {
            double tx = r.getTx();

            if (llAligned) {
                if (Math.abs(tx) > LL_DZ_START) {
                    llAligned = false;
                } else {
                    setTurretPowerLogical(0);
                    return;
                }
            } else {
                if (Math.abs(tx) < LL_DZ_STOP) {
                    llAligned = true;
                    setTurretPowerLogical(0);
                    return;
                }
            }

            double stepDeg = clamp(LL_TURN_SIGN * tx * LL_DEG_PER_TX, -LL_MAX_STEP_DEG, LL_MAX_STEP_DEG);
            turretCmdDeg = nowTurretDeg + stepDeg;
            turretCmdDeg = clamp(turretCmdDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);

        } else {
            if (aimInvalid) {
                setTurretPowerLogical(0);
                return;
            }
            turretCmdDeg = clamp(turretDesiredDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);
        }

        turretCmdDeg_dbg = turretCmdDeg;

        // ======================
        // (D) deg -> ticks target
        // ======================
        int unclamped = (int) Math.round(turretCmdDeg / degPerTick);
        targetTicks = clampTicks(unclamped); // LOGIC target ticks

        // ======================
        // (E) PID in ticks + hard guard (ALL LOGIC)
        // ======================
        nowTicks = getTurretTicks();
        int errTicks = targetTicks - nowTicks;

        double dt = Math.max(0.01, dtTimer.seconds());
        dtTimer.reset();

        double err = errTicks;
        double derr = (err - errPrev) / dt;
        errPrev = err;

        if (Math.abs(errTicks) < ON_TARGET_TICKS) {
            setTurretPowerLogical(0);
            return;
        }

        double pGain = useLL ? kP_LL : kP;
        double dGain = useLL ? kD_LL : kD;
        double maxPwr = useLL ? MAX_POWER_LL : MAX_POWER;

        double power = pGain * err + dGain * derr;
        power = clamp(power, -maxPwr, maxPwr);

        power = guardPowerByLimits(power, nowTicks);

        setTurretPowerLogical(power);
    }

    // LOGIC clamp
    private int clampTicks(int t) {
        int lo = turretMinTicks + TICKS_BUFFER;
        int hi = turretMaxTicks - TICKS_BUFFER;
        return Math.max(lo, Math.min(hi, t));
    }

    // LOGIC guard
    private double guardPowerByLimits(double pwr, int nowTicksLogic) {
        int lo = turretMinTicks + TICKS_BUFFER;
        int hi = turretMaxTicks - TICKS_BUFFER;

        if (nowTicksLogic >= hi && pwr > 0) return 0.0;
        if (nowTicksLogic <= lo && pwr < 0) return 0.0;

        return pwr;
    }

    private void setPoseInches(double xIn, double yIn, double headingDeg) {
        robot.drivetrain.pose = new Pose2d(new Vector2d(xIn, yIn), Math.toRadians(headingDeg));
    }

    private static double wrapDeg(double a) {
        a = a % 360.0;
        if (a <= -180.0) a += 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ================= SHOOTER ANGLE =================
    private double shooterAngleFromDistanceCm(double distanceCm) {
        double posNear = 0.54;
        double posFar  = 0.47;


        if (distanceCm >= 270) {
            shooterTargetRpm = shooterTargetRpmFar;
            return shooterPosition = posFar;
        }

	/*	if (distanceCm >= 210) {
			shooterTargetRpm = 2800;
			return shooterPosition = 0.54;
		}*/

        if (distanceCm >= 170) {
            shooterTargetRpm = 2500;
            return shooterPosition = 0.21;
        }

        if (distanceCm >= 90) {
            shooterTargetRpm = 2000;
            return shooterPosition = 0.21;
        }



        LLResult result = robot.limelight.getLatestResult();

        if ((result == null || !result.isValid())
                && hasLastDistance
                && lastDistance < 95) {

            shooterTargetRpm = 2500;
            return shooterPosition = 0.21;
        }

        shooterTargetRpm = 2500;
        return shooterPosition = 0.21;
    }

    private void ShooterAngle() {
        if (autoShooterAngle) {

            LLResult result = robot.limelight.getLatestResult();
            if (result == null || !result.isValid()) return;

            double ty = result.getTy();

            // ===== CONFIG REAL =====
            final double MOUNT_DEG = 14.0;          // ce ai masurat in Onshape (fata de orizontala)
            final double CAM_HEIGHT_CM = 28.5;      // centrul lentilei
            final double GOAL_HEIGHT_CM = 75.0;     // centrul AprilTag
            final double TY_SIGN = +1.0;            // daca distanta iese aiurea, schimba la +1

            double deltaH = GOAL_HEIGHT_CM - CAM_HEIGHT_CM;

            // ===== VARIANTA PRINCIPALA =====
            double totalDeg = MOUNT_DEG + TY_SIGN * ty;
            double totalRad = Math.toRadians(totalDeg);

            if (Math.abs(Math.tan(totalRad)) < 1e-3) return;

            double distanceCm = deltaH / Math.tan(totalRad);


            // clamp de siguranta
            distanceCm = clamp(distanceCm, 30.0, 500.0);

            // SAVE last valid distance
            lastDistance = distanceCm;
            hasLastDistance = true;

            // ===== DEBUG IMPORTANT =====
            double distPlus  = deltaH / Math.tan(Math.toRadians(MOUNT_DEG + ty));
            double distMinus = deltaH / Math.tan(Math.toRadians(MOUNT_DEG - ty));

            telemetry.addLine("---- LL DIST DEBUG ----");
            telemetry.addData("ty", ty);
            telemetry.addData("MountDeg", MOUNT_DEG);
            telemetry.addData("TotalUsedDeg", totalDeg);
            telemetry.addData("DistUsed", distanceCm);
            telemetry.addData("Dist(mount+ty)", distPlus);
            telemetry.addData("Dist(mount-ty)", distMinus);

            shooterPosition = shooterAngleFromDistanceCm(distanceCm);

            shooterPosition = clamp(shooterPosition, 0.0, 1.0);
            robot.turretTumbler.setPosition(shooterPosition);
            return;
        }

        double input = -driveInput.getValue(Keybindings.Drive.SHOOTER_ANGLE);
        if (Math.abs(input) < SHOOTER_DEADZONE) return;

        shooterPosition += input * 0.01;
        shooterPosition = clamp(shooterPosition, 0.0, 1.0);

        robot.turretTumbler.setPosition(shooterPosition);
    }


    // ================= INTAKE =================
    private void Intake() {
        if (driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY)
                && driveInput.isPressed(Keybindings.Drive.INTAKE_KEY)) {

            if (transfer_servo) robot.transfer.start();
            robot.intake.setIntakeDirection(IntakeDirection.FORWARD_SHOOT);
        }
        else if (driveInput.isPressed(Keybindings.Drive.INTAKE_KEY)) {
            if (transfer_servo) robot.transfer.start();
            robot.intake.setIntakeDirection(IntakeDirection.FORWARD);

        } else if (driveInput.isPressed(Keybindings.Drive.INTAKE_REVERSE_KEY)) {
            robot.intake.setIntakeDirection(IntakeDirection.REVERSE);
        } else {
            robot.transfer.stop();
            robot.intake.setIntakeDirection(IntakeDirection.STOP);
        }
    }

    // ================= TELEMETRY =================
    @Override
    protected void OnTelemetry(Telemetry telemetry) {
        super.OnTelemetry(telemetry);

        if (robot != null && robot.drivetrain != null) {
            Pose2d p = robot.drivetrain.pose;
            telemetry.addData("ODO X (in)", p.position.x);
            telemetry.addData("ODO Y (in)", p.position.y);
            telemetry.addData("ODO H (deg)", Math.toDegrees(p.heading.log()));
        }

        if (robot != null) {
            double r1 = robot.outtake1.getRpm();
            double r2 = robot.outtake2.getRpm();
            telemetry.addData("Shooter TargetRPM", (int) shooterTargetRpm);
            telemetry.addData("OT1 RPM", (int) r1);
            telemetry.addData("OT2 RPM", (int) r2);
            telemetry.addData("AVG RPM ABS", (int)((Math.abs(r1)+Math.abs(r2))/2.0));
        }

        telemetry.addLine(" ");
        telemetry.addData("Anchor", anchorAActive ? "A" : "B");
        telemetry.addData("AnchorX", activeAnchorX_dbg);
        telemetry.addData("AnchorY", activeAnchorY_dbg);

        telemetry.addData("Turret AutoAim", autoAim ? "ON (GateV4)" : "OFF (MANUAL)");
        telemetry.addData("AimInvalid", aimInvalid ? "YES (TURN ROBOT)" : "NO");
        telemetry.addData("UseLL", useLL_dbg ? "YES" : "NO");
        telemetry.addData("LLAligned", llAligned ? "YES" : "NO");

        if (robot == null || robot.limelight == null) {
            telemetry.addLine("Robot sau Limelight NULL");
            return;
        }

        LLResult r = robot.limelight.getLatestResult();
        telemetry.addData("LL valid", (r != null && r.isValid()));
        if (r != null && r.isValid()) telemetry.addData("LL tx", r.getTx());
        telemetry.addData("tx_dbg", tx_dbg);
        telemetry.addData("LL Last Update", robot.limelight.getTimeSinceLastUpdate());

        telemetry.addLine(" ");
        telemetry.addData("fieldAngleDeg", targetAngleFieldDeg_dbg);
        telemetry.addData("turretDesiredDeg", turretDesiredDeg_dbg);
        telemetry.addData("turretCmdDeg", turretCmdDeg_dbg);

        telemetry.addLine(" ");
        telemetry.addData("TurretTicks RAW", robot.turret.getMotor().getCurrentPosition());
        telemetry.addData("TurretTicks LOGIC", getTurretTicks());
        telemetry.addData("TargetTicks", targetTicks);
        telemetry.addData("MinTicks", turretMinTicks);
        telemetry.addData("MaxTicks", turretMaxTicks);
        telemetry.addData("degPerTick", degPerTick);
        telemetry.addData("BUFFER", TICKS_BUFFER);
        telemetry.addData("TURRET_HW_SIGN", TURRET_HW_SIGN);

        telemetry.addLine(" ");
        telemetry.addData("Auto Shooter Angle", autoShooterAngle ? "ON" : "OFF");
        telemetry.addData("Shooter Pos", shooterPosition);

        telemetry.addLine(" ");
        telemetry.addData("Keys", "DPAD_LEFT=Reset+AnchorA, DPAD_RIGHT=Reset+AnchorB, DPAD_UP=ToggleAutoAim");
    }
}