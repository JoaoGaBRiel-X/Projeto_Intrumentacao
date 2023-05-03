#include <Arduino.h>
#include <Servo.h> 
#include <NewPing.h>
#include <PID_v1.h>
#include <AFMotor.h>

// assinaturas de funcoes
void medir_distancia_ultrassonico();
void seguir_pessoa(int distancia);
void monitorar_sensores_pir();
int detectar_pessoa_esquerda();
int detectar_pessoa_direita();

//Instanciando motores
AF_DCMotor motor_esq(3); //Seleciona o motor 1
AF_DCMotor motor_dir(4); //Seleciona o motor 4

//PWM inicia 130
//Defines
#define MAX_DISTANCE 50
#define veloc_esq 19
#define veloc_dir 18
#define TEMPO_DEBOUNCE 800 //us

//Definindo as constantes para os pinos dos sensores 
const int PIRD_PIN = 52;
const int PIRE_PIN = 22; 
const int ULTRASONIC_TRIG_PIN = A0; 
const int ULTRASONIC_ECHO_PIN = A1;
 
// Definindo a constante para o ângulo inicial do servo motor 
const int SERVO_ANGLE_INITIAL = 90; 

// Definindo as variáveis para armazenar os valores dos sensores 
int pirEValue = 0; 
int pirDValue = 0;
unsigned long ultrasonicDistance = 0; 

// Criando a instância do servo motor 
Servo ultrasonicServo; 
int angleServo;

// NewPing setup
NewPing sonar(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, MAX_DISTANCE);

// Variaveis PID
#define pwm_max 255
#define pwm_min 0
#define kp_dir 0.0036
#define ki_dir -0.8
#define kd 0

double rpm; // Define varável rpm 
double velocidade = 0; // output
double Setpoint=3400; // Primeiro valor de referência

// Variaveis sensor encoder
double rpm_esq;
double rpm_dir;
volatile long pulsos_esq;
volatile long pulsos_dir;
unsigned long timeold;
double velocidadeEsq = 0;
double velocidadeDir = 0;
unsigned int pulsos_por_volta = 1; //Altere o numero abaixo de acordo com o seu disco encoder
unsigned long timestamp_ultimo_acionamento1 = 0;
unsigned long timestamp_ultimo_acionamento2 = 0;
volatile unsigned long timer; 


// Configuração incial do PID
PID motorPID_dir(&rpm_dir, &velocidadeDir, &Setpoint, kp_dir, ki_dir, kd, DIRECT); 

// Interrupção
void contador_dir(){
  /* Conta acionamentos do botão considerando debounce */
  if ( (micros() - timestamp_ultimo_acionamento1) >= TEMPO_DEBOUNCE ){
    //Incrementa contador
    pulsos_dir++;
    timestamp_ultimo_acionamento1 = micros();
  }
}
void contador_esq(){
  /* Conta acionamentos do botão considerando debounce */
  if ( (micros() - timestamp_ultimo_acionamento2) >= TEMPO_DEBOUNCE ){
    //Incrementa contador
    pulsos_esq++;
    timestamp_ultimo_acionamento2 = micros();
  }
}


void setup() { 
    
    //Configurando os pinos do sensor hall como entrada pullup 
    pinMode(veloc_dir, INPUT_PULLUP);
    pinMode(veloc_esq, INPUT_PULLUP);
    
    // Configurando os pinos dos sensores como entrada 
    pinMode(PIRE_PIN, INPUT); 
    pinMode(PIRD_PIN, INPUT);
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); 
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    
    //Configurando a velocidade inicial dos motores como zero
    //Gira o Motor 1 - Direita
    motor_dir.setSpeed(130);
    motor_dir.run(FORWARD);
    //Gira o Motor 2 - Esquerda
    motor_esq.setSpeed(130);
    motor_esq.run(FORWARD);
    
    //Configurando o ângulo inicial do servo motor 
    ultrasonicServo.attach(10);
    ultrasonicServo.write(SERVO_ANGLE_INITIAL); 
    
    // Iniciando a comunicação serial para debug 
    Serial.begin(9600); 
    angleServo = SERVO_ANGLE_INITIAL;

    attachInterrupt(digitalPinToInterrupt(veloc_esq), contador_esq, FALLING);
    attachInterrupt(digitalPinToInterrupt(veloc_dir), contador_dir, FALLING);
    pulsos_esq = 0;
    pulsos_dir = 0;
    rpm_esq = 0;
    rpm_dir = 0;
    timeold = millis();
    // Configuração de controle do motor
    motorPID_dir.SetOutputLimits(pwm_min,pwm_max); // Limites de output do PID
    motorPID_dir.SetMode(AUTOMATIC); // Configura PID como automático
  
} 

void loop() { 
    if(velocidade > 0){
        if (millis()- timeold>= 100){
        // Desabilita a interrupção durante o cálculo
        detachInterrupt(digitalPinToInterrupt(veloc_dir));
        detachInterrupt(digitalPinToInterrupt(veloc_esq)); 
        // Realiza o cáculo de rpm
        rpm_dir = (60 * 1000 / pulsos_por_volta ) / (millis() - timeold) * pulsos_dir;
        rpm_esq = (60 * 1000 / pulsos_por_volta ) / (millis() - timeold) * pulsos_esq;
        pulsos_dir = 0;
        pulsos_esq = 0;
        // Mostra o valor de RPM no serial monitor
        Serial.print("PWMDir:");
        Serial.print(velocidadeDir, 2);
        Serial.print("    ");
        Serial.print("RPM_dir:");
        Serial.println(rpm_dir, 0);
        // Mostra o valor de RPM no serial monitor
        Serial.print("PWMEsq:");
        Serial.print(velocidadeEsq, 2);
        Serial.print("    ");
        Serial.print("RPM_esq:");
        Serial.println(rpm_esq, 0);

        timeold = millis();
        //Habilita interrupcao
        attachInterrupt(digitalPinToInterrupt(veloc_dir), contador_dir, FALLING);
        attachInterrupt(digitalPinToInterrupt(veloc_esq), contador_esq, FALLING);
        }
        motorPID_dir.Compute(); // Realiza o cálculo de controle
        motor_esq.setSpeed(velocidadeEsq);
        motor_dir.setSpeed(velocidadeDir);
    }
    // Imprimindo os valores dos sensores na porta serial para debug
    Serial.print("PIR_Esq: ");
    Serial.print(pirEValue);
    Serial.print(", PIR_Dir: ");
    Serial.print(pirDValue);
    Serial.print(", Ultrasonic: ");
    Serial.print(ultrasonicDistance);
    Serial.print(", Angulo: ");
    Serial.print(angleServo);
    medir_distancia_ultrassonico();
    if (ultrasonicDistance > 0 && ultrasonicDistance < 50) {
        // Pessoa ainda detectada, seguir pessoa
        if(ultrasonicDistance > 20){
            seguir_pessoa(ultrasonicDistance);
        } else {
            // Parar o carrinho
            motor_esq.run(BRAKE);
            motor_esq.setSpeed(0);
            motor_dir.run(BRAKE);
            motor_dir.setSpeed(0);
            Serial.print(", PESSOA CAPTURADA");
            while(ultrasonicDistance < 20 && ultrasonicDistance > 0) {
                medir_distancia_ultrassonico();
                delay(500);
            }
        }
    } else {
        // Pessoa não detectada, voltar a monitorar os sensores PIR
        monitorar_sensores_pir();
    }
    // Aguardando 100 milissegundos para a próxima leitura dos sensores
    delay(100);
    Serial.print("\n");
}

void seguir_pessoa(int distancia) {
    // Movimentar o carrinho para frente com a velocidade ajustada
    motor_esq.setSpeed(130);
    motor_esq.run(FORWARD);
    motor_dir.setSpeed(130);
    motor_dir.run(FORWARD);
    Serial.print(", Direcao: RETO ");
}

void monitorar_sensores_pir() {
    pirEValue = digitalRead(PIRE_PIN); 
    pirDValue = digitalRead(PIRD_PIN); 
    // Ativar a monitoração dos sensores PIR
    if(pirEValue == HIGH) detectar_pessoa_esquerda();
    if(pirDValue == HIGH) detectar_pessoa_direita();

    // Parar o carrinho
    motor_esq.run(BRAKE);
    motor_dir.run(BRAKE);
}

int detectar_pessoa_esquerda() {
    // Mover o servo motor para a esquerda
    int detectado = 0;
    angleServo = 180;
    ultrasonicServo.write(angleServo);

    // Aguardar alguns milissegundos para estabilização do servo
    delay(500);

    while (pirEValue == HIGH && angleServo != 90){
        medir_distancia_ultrassonico();
        pirEValue = digitalRead(PIRE_PIN);
        if(angleServo > 90){
            angleServo-=5;
            ultrasonicServo.write(angleServo);
            delay(100);
        }
        while(angleServo > 90){ // virando para Esquerda
            medir_distancia_ultrassonico();
            if(detectado == 1){
                motor_esq.setSpeed(140);
                motor_esq.run(FORWARD);
                motor_dir.setSpeed(200);
                motor_dir.run(FORWARD);
                Serial.println("Direcao: ESQUERDA ");
            }
            angleServo-=5;
            ultrasonicServo.write(angleServo);
            delay(100);
            if(ultrasonicDistance > 20) {
                detectado = 1;
            }
        }
        if(detectado == 1){
            return detectado;
        }
    }
    return detectado;
        
}

int detectar_pessoa_direita() {
    // Mover o servo motor para a direita
    int detectado = 0;
    angleServo = 0;
    ultrasonicServo.write(angleServo);

    // Aguardar alguns milissegundos para estabilização do servo
    delay(500);

    while (pirDValue == HIGH && angleServo != 90){
        medir_distancia_ultrassonico();
        pirDValue = digitalRead(PIRD_PIN);
        if(angleServo < 90){
            angleServo+=5;
            ultrasonicServo.write(angleServo);
            delay(100);
        }
        while(angleServo < 90){ // virando para direita
            medir_distancia_ultrassonico();
            if(detectado == 1){
                motor_esq.setSpeed(200);
                motor_esq.run(FORWARD);
                motor_dir.setSpeed(140);
                motor_dir.run(FORWARD);
                Serial.println("Direcao: DIREITA ");
            }
            angleServo+=5;
            ultrasonicServo.write(angleServo);
            delay(100);
            if(ultrasonicDistance > 20) {
                detectado = 1;
            }
        }
        if(detectado == 1){
            return detectado;
        }
    }
    return detectado;
        
}

void medir_distancia_ultrassonico(){
    ultrasonicDistance = sonar.ping_cm();
}
