package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Date;

@TeleOp(name="tune pid shooter")
public class PIDTune extends LinearOpMode {

    private Motor shooterF, shooterB;
    private GamepadEx gamepad;
    private ButtonReader incrementUp, incrementDown, pUp, pDown, iUp, iDown, dUp, dDown;

    @Override

    public void runOpMode() throws InterruptedException {
        gamepad = new GamepadEx(gamepad1);

        shooterF = new Motor(hardwareMap, "shooterF");
        shooterB = new Motor(hardwareMap, "shooterB");

        pUp = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP);
        pDown = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        iUp = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT);
        iDown = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT);
        dUp = new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        dDown = new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        incrementUp = new ButtonReader(gamepad, GamepadKeys.Button.A);
        incrementDown = new ButtonReader(gamepad, GamepadKeys.Button.B);

        waitForStart();

        shooterF.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setVeloCoefficients(0, 0, 0);
        shooterF.setVeloCoefficients(0, 0, 0);

        shooterF.set(0.5);
        shooterB.set(0.5);

        double p = 0;
        double i = 0;
        double d = 0;
        double increment = 0.1;
        boolean just_pressed = false;

        while(opModeIsActive() && !isStopRequested()){
            if (pUp.wasJustPressed()){
                p += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (pDown.wasJustPressed()){
                p -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (iUp.wasJustPressed()){
                i -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (iDown.wasJustPressed()){
                i += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (dUp.wasJustPressed()){
                d -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (dDown.wasJustPressed()){
                d += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (incrementDown.wasJustPressed()){
                increment = 0.01;
                just_pressed = true;
            }
            if (incrementUp.wasJustPressed()) {
                increment = 0.1;
                just_pressed = true;zz
            }

            shooterB.resetEncoder();
            shooterF.resetEncoder();

            long startTime = System.currentTimeMillis();
            long elapsedTime = 0L;

            while (just_pressed && elapsedTime < 1000 && opModeIsActive() && !isStopRequested()){
                elapsedTime = (new Date()).getTime() - startTime;
            }

            just_pressed = false;

            telemetry.addData("Ticks Per Second Back", shooterB.getCurrentPosition());
            telemetry.addData("Ticks Per Second Front", shooterF.getCurrentPosition());
            telemetry.addData("Increment Level", increment);
            telemetry.addData("P Constant", p);
            telemetry.addData("I Constant", i);
            telemetry.addData("D Constant", d);
            telemetry.update();
            sleep(3000);
       }
    }
}
