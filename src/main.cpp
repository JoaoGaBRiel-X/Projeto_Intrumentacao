#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

//PWM inicia 130
//Defines
#define MAX_DISTANCE 200

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
  
} 

void loop() { 
  
  // Lendo os valores dos sensores 
  pirEValue = digitalRead(PIRE_PIN); 
  pirDValue = digitalRead(PIRD_PIN); 

  
  // Se o sensor PIR direita detectar movimento 
  if (pirDValue == HIGH) { 
    // Movendo o carrinho para a direita 
    analogWrite(MOTOR1_PWMD_PIN, 140);
    analogWrite(MOTOR2_PWME_PIN, 200); 
    Serial.println("Direita");
  } 
  // Se o sensor PIR Esquerda detectar movimento
  else if (pirEValue == HIGH) { 
    // Movendo o carrinho para a esquerda
    analogWrite(MOTOR1_PWMD_PIN, 200); 
    analogWrite(MOTOR2_PWME_PIN, 140); 
    Serial.println("Esquerda");

  } 
  // Se nenhum sensor detectar movimento 
  else { 
    // Parando o carrinho
    analogWrite(MOTOR1_PWMD_PIN, 0); 
    analogWrite(MOTOR2_PWME_PIN, 0); 
  }
  // Movimentando o servo motor para a direita
  for (int angle = SERVO_ANGLE_INITIAL; angle <= 180; angle += 5) {
    ultrasonicServo.write(angle);
    delay(50);
    ultrasonicDistance = sonar.ping_cm();
    if (ultrasonicDistance < 20) {
    // Se o objeto estiver próximo, parar o carrinho
      analogWrite(MOTOR1_PWMD_PIN, 0);
      analogWrite(MOTOR2_PWME_PIN, 0);
      break;
    }
  }
  // Movimentando o servo motor para a esquerda
  for (int angle = 180; angle >= SERVO_ANGLE_INITIAL; angle -= 5) {
    ultrasonicServo.write(angle);
    delay(50);
    ultrasonicDistance = sonar.ping_cm();
    if (ultrasonicDistance < 20) {
      // Se o objeto estiver próximo, parar o carrinho
      analogWrite(MOTOR1_PWMD_PIN, 0);
      analogWrite(MOTOR2_PWME_PIN, 0);
      break;
    }
  }
  // Imprimindo os valores dos sensores na porta serial para debug
  Serial.print("PIR_Esq: ");
  Serial.print(pirEValue);
  Serial.print(", PIR_Dir: ");
  Serial.print(pirDValue);
  Serial.print(", Ultrasonic: ");
  Serial.println(ultrasonicDistance);
  // Aguardando 100 milissegundos para a próxima leitura dos sensores
  delay(100);
}

