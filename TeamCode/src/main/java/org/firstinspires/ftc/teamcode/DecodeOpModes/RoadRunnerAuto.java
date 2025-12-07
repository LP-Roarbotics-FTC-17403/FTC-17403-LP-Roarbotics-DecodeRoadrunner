package org.firstinspires.ftc.teamcode.DecodeOpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.CameraSystem;
import org.firstinspires.ftc.teamcode.SubSystems.ColorSensorCode;
import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
import org.firstinspires.ftc.teamcode.SubSystems.Firecracker;
import org.firstinspires.ftc.teamcode.SubSystems.Hammer;
import org.firstinspires.ftc.teamcode.SubSystems.Inhaler;
import org.firstinspires.ftc.teamcode.SubSystems.LED;
import org.firstinspires.ftc.teamcode.SubSystems.MotorClass;

import com.acmerobotics.roadrunner.Action;

@Autonomous
public final class RoadRunnerAuto extends LinearOpMode {

    private Pose2d beginPose;
    private Pose2d launchPose;
    private MecanumDrive drive;
    private CameraSystem camera;
    private Firecracker rightFirecracker;
    private Firecracker leftFirecracker;
    private Inhaler inhaler1;
    private Inhaler inhaler2;
    private Feeder leftFeeder;
    private Feeder rightFeeder;
    private LED leftLight;
    private LED middleLight;
    private LED rightLight;
    private ColorSensorCode leftColor;
    private ColorSensorCode rightColor;
    private MotorClass vroom;

    private Hammer hammer;

    final double FEED_TIME_SECONDS = 0.80; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        beginPose                = new Pose2d(-49, -48, Math.toRadians(233.5));
        launchPose              = new Pose2d(-40,-35, Math.toRadians(233.5));
        drive              = new MecanumDrive(hardwareMap, beginPose);
        camera             = new CameraSystem(hardwareMap);
        rightFirecracker    =  new Firecracker(hardwareMap, "right_launcher");
        leftFirecracker     =  new Firecracker(hardwareMap, "left_launcher");
        inhaler1                 = new Inhaler(hardwareMap, "intake");
        inhaler2                = new Inhaler(hardwareMap, "transfer");
        leftFeeder               = new Feeder(hardwareMap, "left_feeder");
        rightFeeder              = new Feeder(hardwareMap, "right_feeder");
        leftLight                   = new LED(hardwareMap, "left_light");
        middleLight                 = new LED(hardwareMap, "middle_light");
        rightLight                  = new LED(hardwareMap, "right_light");
        leftColor       = new ColorSensorCode(hardwareMap, "left_color_sensor");
        rightColor      = new ColorSensorCode(hardwareMap, "right_color_sensor");
        vroom            = new MotorClass(hardwareMap);


        boolean reverse = true;
        boolean notReverse = false;
        int detectedID;

        //adjust reverse if needed
        leftFeeder.initialize(reverse);
        rightFeeder.initialize(notReverse);
        leftFirecracker.initialize(notReverse);
        rightFirecracker.initialize(reverse);
        inhaler1.initialize(reverse);
        inhaler2.initialize(reverse);
        vroom.initialize(true);
        //camera.cameraOn();

        Action t1 = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(-40,-35))
                .build();

        Action t2 = drive.actionBuilder(launchPose)
                .splineToSplineHeading(new Pose2d(-12,-25, Math.toRadians(270)), Math.toRadians(270))
                .afterDisp(0, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        inhaler1.intaking(),
                                        inhaler2.intaking()
                                )
                        ))
                .afterDisp(3, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        hammer.moveLeft()
                                )
                        ))
                .afterDisp(6, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        inhaler1.notIntaking(),
                                        inhaler2.notIntaking()
                                )
                        ))
                .lineToYLinearHeading(-45, Math.toRadians(270), new TranslationalVelConstraint(10.0))
                .splineToSplineHeading(new Pose2d(-40,-35, Math.toRadians(233.5)), Math.toRadians(233.5))
                .build();

        Action t3 = drive.actionBuilder(launchPose)
                .splineToSplineHeading(new Pose2d(12,-25, Math.toRadians(270)), Math.toRadians(270))
                .afterDisp(0, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        inhaler1.intaking(),
                                        inhaler2.intaking()
                                )
                        ))
                .afterDisp(3, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        hammer.moveLeft()
                                )
                        ))
                .afterDisp(6, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        inhaler1.notIntaking(),
                                        inhaler2.notIntaking()
                                )
                        ))
                .lineToYLinearHeading(-45, Math.toRadians(270), new TranslationalVelConstraint(10.0))
                .splineToSplineHeading(new Pose2d(-40,-35, Math.toRadians(233.5)), Math.toRadians(233.5))
                .build();

        Action t4 = drive.actionBuilder(launchPose)
                .splineToSplineHeading(new Pose2d(36,-25, Math.toRadians(270)), Math.toRadians(270))
                .afterDisp(0, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        inhaler1.intaking(),
                                        inhaler2.intaking()
                                )
                        ))
                .afterDisp(3, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        hammer.moveLeft()
                                )
                        ))
                .afterDisp(6, ()->
                        Actions.runBlocking(
                                new ParallelAction(
                                        inhaler1.notIntaking(),
                                        inhaler2.notIntaking()
                                )
                        ))
                .lineToYLinearHeading(-45, Math.toRadians(270), new TranslationalVelConstraint(10.0))
                .splineToSplineHeading(new Pose2d(-40,-35, Math.toRadians(233.5)), Math.toRadians(233.5))
                .build();


        waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            t1,
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    spitfire();
                                    return false;
                                }
                            },
                            t2,
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    spitfire();
                                    return false;
                                }
                            },
                            t3,
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    spitfire();
                                    return false;
                                }
                            },
                            t4,
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    spitfire();
                                    return false;
                                }
                            }


                    )
            );



    }

    void spitfire(){
        launchCycle(leftFeeder, leftFirecracker);

        //second fire cycle
        launchCycle(rightFeeder, rightFirecracker);

        //third fire cycle
        launchCycle(leftFeeder, leftFirecracker);

        launchCycle(rightFeeder, rightFirecracker);

        launchCycle(leftFeeder, leftFirecracker);
    }
    void launchCycle(Feeder feed, Firecracker launcher){
        leftFirecracker.crackDaFire();
        rightFirecracker.crackDaFire();

        while(opModeIsActive() && launcher.getCurrentVelocity()<launcherTarget){
            telemetry.addData("Current velocity: ", launcher.getCurrentVelocity());
            telemetry.update();
        }
        inhaler1.inhale_on();
        inhaler2.inhale_on();
        feed.feed_on();
        sleep(1000);
        feed.feed_off();
        inhaler1.inhale_off();
        inhaler2.inhale_off();
    }
}

