package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumDriveTrain by Emiliano")
public class Mecanum_Test extends LinearOpMode {
    @Override
    public void runOpMode(){

        //Declarar variables de movimiento de robot
        double drive;
        double turn;
        double strafe;

        //Declarar variables de poder de motores
        double FIPower;
        double FDPower;
        double AIPower;
        double ADPower;

        //Inicializacion de los 4 motores para el chasis
        DcMotor FrenteIzquierdo = hardwareMap.get(DcMotor.class, "FrenteIzquierdo");
        DcMotor FrenteDerecho = hardwareMap.get(DcMotor.class, "FrenteDerecho");
        DcMotor AtrasIzquierdo = hardwareMap.get(DcMotor.class, "AtrasIzquierdo");
        DcMotor AtrasDerecho = hardwareMap.get(DcMotor.class, "AtrasDerecho");

        //Invertir motores para girar en el mismo sentido
        FrenteDerecho.setDirection(DcMotorSimple.Direction.REVERSE);
        AtrasDerecho.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoders
        FrenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AtrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AtrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Iniciar la Driver Station
        waitForStart();

        while(opModeIsActive()){
            //configurar valores del joystick del control
            drive = gamepad1.left_stick_y * -0.8; //mover el robot
            turn = gamepad1.right_stick_x; //girar el robot
            strafe = gamepad1.left_stick_x; //mover el robot para los lados

            //Declarar variables de poder de motores
            FIPower = drive + turn + strafe; //positivo hacia enfrente
            FDPower = drive - turn - strafe; //opuesto que frente izquierdo
            AIPower = drive + turn - strafe; //invertimos para ir hacia atras
            ADPower = drive - turn + strafe; //opuesto que atras izquierdo

            //Dar poder a los motores
            FrenteIzquierdo.setPower(FIPower);
            FrenteDerecho.setPower(FDPower);
            AtrasIzquierdo.setPower(AIPower);
            AtrasDerecho.setPower(ADPower);
        }

    }
}
