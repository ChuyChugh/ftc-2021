package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BasicAuto extends LinearOpMode {
    MecanumDrive drive;
    SimpleServo grabber;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(
                new Motor(hardwareMap, "fL"),
                new Motor(hardwareMap, "fR"),
                new Motor(hardwareMap, "bL"),
                new Motor(hardwareMap, "bR")
        );
        grabber = new SimpleServo(hardwareMap, "grabber", -90, 180);
        grabber.setPosition(0.6);

        waitForStart();

        drive.driveRobotCentric(0, 0.55, 0);
        sleep(1250);
        drive.driveRobotCentric(-0.75, 0, 0);
        sleep(1700);
        drive.stop();

        while (opModeIsActive()) idle();
    }
}
