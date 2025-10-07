package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Mecanum_WI by Emiliano")
public class MecanumDrive_WithoutIMU extends LinearOpMode {
    @Override
    public void runOpMode(){
        //Declarar 4 motores para el chasis
        DcMotor FrenteIzquierdo = hardwareMap.get(DcMotor.class, "FrenteIzquierdo");
        DcMotor FrenteDerecho = hardwareMap.get(DcMotor.class, "FrenteDerecho");
        DcMotor AtrasIzquierdo = hardwareMap.get(DcMotor.class, "AtrasIzquierdo");
        DcMotor AtrasDerecho = hardwareMap.get(DcMotor.class, "AtrasDerecho");

        //Inversion de dirección de motores
        AtrasIzquierdo.setDirection(DcMotorSimple.Direction.REVERSE);
        AtrasDerecho.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoders
        FrenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AtrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AtrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Limitar pontencia de motores
        double lowPower = 0.2;
        double highPower = 0.8;
        double currentPower = highPower;

        //Reseteo de IMU
        //Deadline imuLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        //Crear e inicializar el objeto para la IMU
        /*  IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        */

        //Iniciar la Driver Station
        waitForStart();

        //Empezar el código hasta que pare la Driver Station
        while  (opModeIsActive()){
            //Leer los valores de los joysticks izquierdo y derecho del control
            double lx = gamepad1.left_stick_x; //mover el robot hacia adelante
            double ly = gamepad1.left_stick_y; //mover el robot hacia atras
            double rx = gamepad1.right_stick_x; //girar el robot

            //Variable para que los motores no sobrepasen el limite
            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            /*  double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double LxAjustada = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double LyAjustada = ly * Math.cos(heading) + lx * Math.sin(heading);*/

            //Cambiar modo de velocidad
            if (gamepad1.dpad_down){
                if(currentPower == highPower){
                    currentPower = lowPower;
                } else {
                    currentPower = highPower;
                }
            }


            /*  Reseteo de IMU
            if(imuLimit.hasExpired() && gamepad1.b){
                imu.resetYaw();
                ireset();
            }*/

            //Lógica para mover los 4 motores del Mecanum
            FrenteIzquierdo.setPower(((ly+lx+rx)/max)*currentPower); //positivo hacia enfrente
            AtrasIzquierdo.setPower(((ly-lx+rx)/max)*currentPower); //invertimos lx para que sea igual que FrenteIzquierdo
            FrenteDerecho.setPower(((ly-lx-rx)/max)*currentPower); //opuesto a FrenteIzquierdo
            AtrasDerecho.setPower(((ly+lx-rx)/max)*currentPower); //opuesto a AtrasIzquierdo
        }
    }
}

