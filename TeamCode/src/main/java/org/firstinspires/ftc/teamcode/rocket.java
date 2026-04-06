
package org.firstinspires.ftc.teamcode; // Define o pacote do código

// Importações necessárias para o FTC funcionar
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Define que esse código é um TeleOp (controle manual)
@TeleOp(name = "ROCKET", group = "FTC")
public class rocket extends OpMode {

    // 🔹 Declaração dos motores (nomes que você escolheu)
    private DcMotor FE0; // Motor Frente Esquerda (porta 0)
    private DcMotor FD1; // Motor Frente Direita (porta 1)
    private DcMotor TE2; // Motor Trás Esquerda (porta 2)
    private DcMotor TD3; // Motor Trás Direita (porta 3)

    @Override
    public void init() {

        // 🔹 Aqui fazemos o "link" entre o código e o robô (hardwareMap)
        // Os nomes ENTRE ASPAS devem ser iguais ao Robot Configuration
        FE0 = hardwareMap.dcMotor.get("FE0"); // Porta 0
        FD1 = hardwareMap.dcMotor.get("FD1"); // Porta 1
        TE2 = hardwareMap.dcMotor.get("TE2"); // Porta 2
        TD3 = hardwareMap.dcMotor.get("TD3"); // Porta 3

        // 🔹 Inverte os motores do lado direito
        // Isso é necessário porque eles giram ao contrário dos da esquerda
        FD1.setDirection(DcMotor.Direction.REVERSE);
        TD3.setDirection(DcMotor.Direction.REVERSE);

        // 🔹 Define o modo dos motores (sem encoder = mais simples)
        FE0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FD1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TE2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TD3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        // 🎮 Leitura do controle (gamepad)

        // Stick esquerdo Y → movimento para frente e para trás
        // O sinal é invertido (-) porque o controle vem invertido por padrão
        double frente = -gamepad1.left_stick_y;

        // Stick direito X → giro (virar o rob
        double giro = gamepad1.right_stick_x;

        // 🔹 Cálculo do movimento (Arcade Drive)
        // Soma = lado esquer
        // Subtração = lado direito
        double esquerda = frente + giro;
        double direita = frente - giro;

        // 🔹 Envia potência para os motores do lado esquerdo
        FE0.setPower(esquerda);
        TE2.setPower(esquerda);

        // 🔹 Envia potência para os motores do lado direito
        FD1.setPower(direita);
        TD3.setPower(direita);
    }
}