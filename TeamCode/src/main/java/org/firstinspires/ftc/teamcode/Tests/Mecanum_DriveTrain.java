package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumDriveTrain by Emiliano")
public class Mecanum_DriveTrain extends LinearOpMode {
    @Override
    public void runOpMode(){

        //Declarar variables de movimiento de robot
        double drive;   //mover hacia delante
        double turn;    //girar
        double strafe;  //ir en diagonal

        //Declarar variables de poder de motores
        double FIPower;
        double FDPower;
        double AIPower;
        double ADPower;

        //Inicializacion de los 4 motores para el chasis
        DcMotor FrenteIzquierdo = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor FrenteDerecho = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor AtrasIzquierdo = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor AtrasDerecho = hardwareMap.get(DcMotor.class, "rightBack");

        //Invertir motores para girar en el mismo sentido
        FrenteDerecho.setDirection(DcMotorSimple.Direction.FORWARD);
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
            strafe = gamepad1.left_stick_x; //mover el robot en diagonal

            //Modo lento
            if(gamepad1.a){ //Presionar boton a
                drive = gamepad1.left_stick_y * -0.2;       //limitar motor a 0.2 de potencia
            }else{drive = gamepad1.left_stick_y * -0.8;}    //si no se presiona el boton, dejar el motor a 0.8 de potencia

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

//33 was here