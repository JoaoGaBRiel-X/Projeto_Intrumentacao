#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

// assinaturas de funcoes
void medir_distancia_ultrassonico();
void seguir_pessoa(int distancia);
void monitorar_sensores_pir();
int detectar_pessoa_esquerda();
int detectar_pessoa_direita();


//PWM inicia 130
//Defines
#define MAX_DISTANCE 50

// Definindo as constantes para os pinos do motor 1 - Direita
const int MOTOR1_PWMD_PIN = 12; 
const int MOTOR1_IN3_PIN = 8; 
const int MOTOR1_IN4_PIN = 9; 

// Definindo as constantes para os pinosdo motor 2 - Esquerda
const int MOTOR2_PWME_PIN = 13; 
const int MOTOR2_IN1_PIN = 10;
const int MOTOR2_IN2_PIN = 11; 

//Definindo as constantes para os pinos dos sensores 
const int PIRD_PIN = 5;
const int PIRE_PIN = 4; 
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

void setup() { 
  
  //Configurando os pinos dos motores como saída 
  pinMode(MOTOR1_PWMD_PIN, OUTPUT);
  pinMode(MOTOR1_IN3_PIN, OUTPUT); 
  pinMode(MOTOR1_IN4_PIN, OUTPUT); 
  pinMode(MOTOR2_PWME_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT); 
  pinMode(MOTOR2_IN2_PIN, OUTPUT); 
  
  // Configurando os pinos dos sensores como entrada 
  pinMode(PIRE_PIN, INPUT); 
  pinMode(PIRD_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); 
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
   
  //Configurando a velocidade inicial dos motores como zero
  //Gira o Motor 1 - Direita
  digitalWrite(MOTOR1_IN3_PIN, LOW);
  digitalWrite(MOTOR1_IN4_PIN, HIGH);
  //Gira o Motor 2 - Esquerda
  digitalWrite(MOTOR2_IN1_PIN, HIGH);
  digitalWrite(MOTOR2_IN2_PIN, LOW);
  
  //Configurando o ângulo inicial do servo motor 
  ultrasonicServo.attach(6);
  ultrasonicServo.write(SERVO_ANGLE_INITIAL); 
  
  // Iniciando a comunicação serial para debug 
  Serial.begin(9600); 
  angleServo = SERVO_ANGLE_INITIAL;
  
} 

void loop() { 
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
            analogWrite(MOTOR1_PWMD_PIN, 0);
            analogWrite(MOTOR2_PWME_PIN, 0);
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
    analogWrite(MOTOR1_PWMD_PIN, 200);
    analogWrite(MOTOR2_PWME_PIN, 200);
    Serial.print(", Direcao: RETO ");
}

void monitorar_sensores_pir() {
    pirEValue = digitalRead(PIRE_PIN); 
    pirDValue = digitalRead(PIRD_PIN); 
    // Ativar a monitoração dos sensores PIR
    if(pirEValue == HIGH) detectar_pessoa_esquerda();
    if(pirDValue == HIGH) detectar_pessoa_direita();

    // Parar o carrinho
    analogWrite(MOTOR1_PWMD_PIN, 0);
    analogWrite(MOTOR2_PWME_PIN, 0);
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
                analogWrite(MOTOR1_PWMD_PIN, 200);
                analogWrite(MOTOR2_PWME_PIN, 140); 
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
                analogWrite(MOTOR1_PWMD_PIN, 140);
                analogWrite(MOTOR2_PWME_PIN, 200); 
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
