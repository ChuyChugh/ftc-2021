package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class RealAuto extends LinearOpMode {
    private Motor fL, fR, bL, bR, intake;
    private Motor shooterF, shooterB;
    private SimpleServo hopper, flicker, grabber, gearboxL, gearboxR;

    @Override

    public void runOpMode() throws InterruptedException {

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        shooterF = new Motor(hardwareMap, "shooterF");
        shooterB = new Motor(hardwareMap, "shooterB");
        intake = new Motor(hardwareMap, "intake");

        flicker = new SimpleServo(hardwareMap, "flicker", -90, 90);
        hopper = new SimpleServo(hardwareMap, "hopper", -90, 180);
        grabber = new SimpleServo(hardwareMap, "grabber", -90, 180);
        gearboxL = new SimpleServo(hardwareMap, "gearboxL", -90, 180);
        gearboxR = new SimpleServo(hardwareMap, "gearboxR", -90, 180);

        flicker.setInverted(true);
        flicker.setPosition(0.5);

        shooterF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        hopper.setPosition(1 - (40.0 / 270));
        gearboxR.setPosition(0.4);
        gearboxL.setPosition(0.4);
        grabber.setPosition(0);

        waitForStart();

        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
        fR.encoder.setDirection(Motor.Direction.REVERSE);

        Pose2d startPose = new Pose2d(-63, -48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory t1_014ring = drive.trajectoryBuilder(startPose)
                .forward(6)
                .build();

        Trajectory t2_014ring = drive.trajectoryBuilder(t1_014ring.end())
                .splineTo(new Vector2d(-24, -54), Math.toRadians(0))
                .build();

        Trajectory t3_0ring = drive.trajectoryBuilder(t2_014ring.end())
                .splineTo(new Vector2d(12, -48), Math.toRadians(0))
                .build();

        Trajectory t3_1ring = drive.trajectoryBuilder(t2_014ring.end())
                .splineTo(new Vector2d(36, -24), Math.toRadians(0))
                .build();

        Trajectory t3_4ring = drive.trajectoryBuilder(t2_014ring.end())
                .splineTo(new Vector2d(60, -48), Math.toRadians(0))
                .build();

        Trajectory t4_1ring = drive.trajectoryBuilder(t3_1ring.end())
                .back(24)
                .build();

        Trajectory t4_4ring = drive.trajectoryBuilder(t3_4ring.end())
                .back(48)
                .build();

        //0 rings
        shooterF.set(0.3);
        shooterB.set(0.3);
        drive.followTrajectory(t1_014ring);
        drive.turn(Math.toRadians(30));
        for (int i = 0; i < 3; i++){
            sleep(1000);
            flicker.setPosition(1);
            sleep(350);
            flicker.setPosition(0.5);
        }
        shooterF.set(0);
        shooterB.set(0);
        drive.turn(Math.toRadians(-30));
        drive.followTrajectory(t2_014ring);
        drive.followTrajectory(t3_0ring);
        gearboxL.setPosition(0.85);
        gearboxR.setPosition(0.85);
        grabber.setPosition(0.6);
        gearboxL.setPosition(0.4);
        gearboxR.setPosition(0.4);

        
        
        //1 ring
        shooterF.set(0.3);
        shooterB.set(0.3);
        drive.followTrajectory(t1_014ring);
        drive.turn(Math.toRadians(30));
        for (int i = 0; i < 3; i++){
            sleep(1000);
            flicker.setPosition(1);
            sleep(350);
            flicker.setPosition(0.5);
        }
        shooterF.set(0);
        shooterB.set(0);
        drive.turn(Math.toRadians(-30));
        drive.followTrajectory(t2_014ring);
        drive.followTrajectory(t3_1ring);
        gearboxL.setPosition(0.85);
        gearboxR.setPosition(0.85);
        grabber.setPosition(0.6);
        gearboxL.setPosition(0.4);
        gearboxR.setPosition(0.4);
        drive.followTrajectory(t4_1ring);

        
        
        //4 rings
        shooterF.set(0.3);
        shooterB.set(0.3);
        drive.followTrajectory(t1_014ring);
        drive.turn(Math.toRadians(30));
        for (int i = 0; i < 3; i++){
            sleep(1000);
            flicker.setPosition(1);
            sleep(350);
            flicker.setPosition(0.5);
        }
        shooterF.set(0);
        shooterB.set(0);
        drive.turn(Math.toRadians(-30));
        drive.followTrajectory(t2_014ring);
        drive.followTrajectory(t3_4ring);
        gearboxL.setPosition(0.85);
        gearboxR.setPosition(0.85);
        grabber.setPosition(0.6);
        gearboxL.setPosition(0.4);
        gearboxR.setPosition(0.4);
        drive.followTrajectory(t4_4ring);
    }
}
