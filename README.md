# Localization Sensor Quickstart for Road Runner 1.0

There are currently three standalone localization sensors availiable for FTC Teams:
the SparkFun OTOS, the goBILDA Pinpoint, and the Digital Chicken Labs OctoQuad v2. 
This repository allows teams to integrate them into Road Runner as a drop-in replacement.

## Notes and Warnings

The custom localization is implemented using the sensor-specific drive classes,
which extend the AbsoluteLocalizerDrive class, which extends MecanumDrive.
This means that all of Road Runner's standard tuning should remain in MecanumDrive,
but you should use the sensor-specific drive class (such as SparkFunOTOSDrive) in your OpModes.

I have opened PRs for the underlying library changes required for my integration, but
they have not been merged; it seems like rbrott would prefer an official integration to rethink the
tuning process entirely (which is reasonable but not something I personally have the time/knowledge
for). See https://github.com/acmerobotics/road-runner-ftc/issues/8 and the linked PR's for further
discussion.

The repository URL does refer to only the OTOS sensor.
This is no longer correct, but at this point changing the url would break many links unfortunately.

## When things go wrong…

This quickstart has not been extensively tested, and you are likely to encounter bugs and issues.
If this happens, or if there's anything you're confused about or don't understand, the best way to
get help is
making a post in roadrunner-help on the FTC Discord with your MecanumDrive and sensor-specific drive class
attached and pinging me (@j5155).
If you're certain that you've found a bug,
or you have a feature request, you may also make an issue in the Issues tab above.

Do NOT make an issue on the official Roadrunner quickstart while you are using this one.

# Installation

## Downloading the quickstart as a new project

Download the files directly from https://github.com/jdhs-ftc/sparkfun-otos-quickstart/
or download with git using `git clone -b pinpoint https://github.com/jdhs-ftc/sparkfun-otos-quickstart.git`

## Adding to an existing project (with git)

Easiest way to copy over all the changes to an existing repo is to run these 2 commands:

`git remote add sensor-quickstart https://github.com/jdhs-ftc/sparkfun-otos-quickstart`
`git pull sensor-quickstart master --no-rebase`

It is *possible* to copy everything over manually but would not recommend, it is kind of a pain.

(If the git method doesn't work, it would be easier to copy your teamcode files to a new downloaded copy of my quickstart)

# Tuning

**For OTOS tuning, see OTOS_INSTRUCTIONS.md.**
**The below steps are for Pinpoint and OctoQuad tuning.**

## Hardware
Pinpoint:
Ensure the Pinpoint is mounted **sticker and ports facing up.**

Connect the forward and back/parallel pod to the X port
and the strafe/perpendicular pod to the Y port.

Connect the i2c port to an i2c port on your Control Hub.

OctoQuad:

The OctoQuad can be mounted in any orthogonal orientation
and automatically detects which one it is in.

Note that if you change the orientation of your OctoQuad you must tune the yaw scalar again.

The pods can be connected to any port on the OctoQuad,
you only need to specify the connected port in OctoQuadDrive.

The default is for the forward and back/parallel/X pod to be in port 1 (not 0),
and the strafe/perpendicular/Y pod to be in port 2.
These can however easily be changed.

## Software

Change the drive class in TuningOpModes, SplineTest,
and your op modes to the drive class you're using (PinpointDrive or OctoQuadDrive)

Ensure you configure your sensor with the same name as is set at the top of each drive class. 
The default name is `pinpoint` for the Pinpoint and `octoquad` for the OctoQuad.

Make sure that the sensor is configured under the Roadrunner Driver variant.

Leave inPerTick at 1.0 in MecanumDrive, and leave localizer as DriveLocalizer (or set it to `null`).

Set Encoder Resolution to the amount of ticks per millimeter tracked by your odometry pods. 
There are presets availiable for GoBILDA odometry.

To tune other odometry pods, first set encoder resolution to 1.0.

Next, use ForwardPushTest and push the robot some distance.

Then divide the number of ticks recorded by the number of millimeters traveled.
For example, if the bot recorded 12087 ticks traveled and you measure the distance traveled at 500 mm,
your ticks per mm will be 12087/500 = 24.174.

Use DeadWheelDirectionDebugger to properly reverse your dead wheels in your sensor drive class.

Measure your odometry offsets **manually** (with a tape measure).
Note that AngularRampLogger cannot automatically tune them.

The way in which odometry offsets work is device-specific. 
Review the quickstart guide for the device you are using for a full explanation;
alternately, some slight trial and error with Localization Test may be easiest. 

Your offsets don't need to be perfect.
Any error will show up as the robot orbiting around a point on FTC Dashboard when spinning in place IRL.

If you are using OctoQuad, you must **manually tune** Angular Scalar. Follow the instructions in the Angular Scalar OpMode.

Finally, you can begin standard tuning on https://rr.brott.dev/docs/v1-0/tuning/ but make sure to **skip ForwardPushTest.**

Some teams have reported issues tuning track width with AngularRampLogger as well.
It may be easier to measure that manually.

If you use this, please let me know how it goes, both if things go wrong but *especially* if it all
works smoothly; I'd love more people to test this before moving forward with a more formal
release/community news post.

# Special Thanks

Ethan and John from GoBILDA for providing support and help with the GoBILDA Pinpoint integration,
providing me with a Pinpoint and odometry pods to test it with,
and sponsoring my team Capital City Dynamics.

Michael from Digital Chicken Labs for providing support and help with the 
Digital Chicken Labs OctoQuad integration
and providing me an OctoQuad to test it with.

Dryw from SparkFun for providing support and help with the SparkFun OTOS integration.