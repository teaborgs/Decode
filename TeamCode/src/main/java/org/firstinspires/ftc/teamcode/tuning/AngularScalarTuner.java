package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;


public class AngularScalarTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive;
        if (TuningOpModes.DRIVE_CLASS == PinpointDrive.class) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        }  else {
            throw new RuntimeException("Tuning Angular Scalar is only necessary with OctoQuad and OTOS.");
        }
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double radsTurned = 0;
        Rotation2d lastHeading = Rotation2d.fromDouble(0);
        telemetry.addLine("Angular Scalar Tuner");
        telemetry.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).");
        telemetry.addLine("Then copy the scalar into your drive class.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            radsTurned += drive.pose.heading.minus(lastHeading);
            lastHeading = drive.pose.heading;
            telemetry.addData("Uncorrected Degrees Turned", Math.toDegrees(radsTurned));
            telemetry.addData("Calculated Angular Scalar", 3600 / Math.toDegrees(radsTurned));
            telemetry.update();
        }
    }
}
