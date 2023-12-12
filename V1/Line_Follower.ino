#include <QTRSensors.h>

// Constantes para PID
float KP = .1; // 1.0 /valor maximo teorico de 65535
float KD = 1.1; // 1.0;
float Ki = 0.001;//.006

// Regulación de la velocidad Máxima
int Velmax = 245  ;   //135//40 //Maximo 255 nieveles
int VelFreno=225;     //160 por lomenos 10uds menos que el max
int VelFrenoRev=255;  //10  por lomenos 10uds mayor que el max

//Limites para la lectura de la barra de sensores 
int LSuperior = 6900;
int LInferior = 100;

int Target = 3500; // Setpoint (Como utilizamos 6 sensores, la línea debe estar entre 0 y 5000, por lo que el ideal es que esté en 2500)


//Mapeo de pines
#define LED 13

//*****Linea negra
#define AIN1  8
#define AIN2  7
#define PWMB  6

#define BIN1  9
#define BIN2  4
#define PWMA  5
//*****

//*****Linea blanca
//#define AIN1  4
//#define AIN2  3
//#define PWMB  5

//#define PWMA  6
//#define BIN1  8
//#define BIN2  7
//*****

#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             11
#define LED     13
#define GO 10

#define BOTON  12//10 //12 es para arrancador y 12 es para boton
#define BOTON1 12  //12   

// Data para intrgal
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

unsigned int position = 0;
//declara variables para utilizar PID
int proporcional = 0;         // Proporcional
int integral = 0;           //Intrgral
int derivativo = 0;          // Derivativo
int diferencial = 0;   // Diferencia aplicada a los motores
int last_prop;         // Última valor del proporcional (utilizado para calcular la derivada del error)


// Configuración de la librería QTR-8A
QTRSensorsAnalog qtra((unsigned char[])
{
  A0, A1, A2, A3, A4, A5, A6, A7
}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

// Función accionamiento motor izquierdo
void Motoriz(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value ;
  }
  analogWrite(PWMB, value);
}

// Función accionamiento motor derecho
void Motorde(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMA, value);
}
//Accionamiento de motores
void Motor(int left, int righ)
{
  //digitalWrite(STBY, HIGH);
  Motoriz(left);
  Motorde(righ);
}

//función de freno
void freno(boolean left, boolean righ, int value)
{
  digitalWrite(LED, HIGH);
  if ( left )
  {
 digitalWrite(LED, LOW);

    //    digitalWrite(BIN1, LOW);
    //    digitalWrite(BIN2, HIGH);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite (PWMB, value);
  }
  if ( righ )
  {
    digitalWrite(LED, LOW);

    //    digitalWrite(AIN1, LOW);
    //    digitalWrite(AIN2, HIGH);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite (PWMA, value);
  }

  
}

// BOTON DE ESPERA
void WaitBoton() {
  while (digitalRead(BOTON));
// while (!digitalRead(BOTON));

}

/////////////////////////////////////////////////////

// BOTON DE ESPERA
void WaitGO() {
  while (!digitalRead(GO));
}

///////////////////////////////////////////////////////

void frenos_contorno (int flanco_comparacion)
{
    if  (position<=LInferior)
{ 
      
       digitalWrite(BIN1, HIGH);
       digitalWrite(BIN2, LOW);
       analogWrite (PWMB, VelFreno);

       digitalWrite(AIN1, LOW);     //REVERSA
       digitalWrite(AIN2, HIGH);
       analogWrite (PWMA, VelFrenoRev);
           digitalWrite(LED, HIGH);
while(true)
  {
      qtra.read(sensorValues);
      if (sensorValues[0] > flanco_comparacion || sensorValues[1] > flanco_comparacion)

            {
              digitalWrite(LED, LOW);
              break;
            }
     }
}

       if (position >= LSuperior)
       { 

        
       digitalWrite(BIN1, LOW);  //REVERSA
       digitalWrite(BIN2, HIGH);
       analogWrite (PWMB, VelFrenoRev);

       digitalWrite(AIN1, HIGH);
       digitalWrite(AIN2, LOW);
       analogWrite (PWMA, VelFreno);
                  digitalWrite(LED, HIGH);

       while(true)
       {
        qtra.read(sensorValues);
        if (sensorValues[7]>flanco_comparacion ||sensorValues[6]>flanco_comparacion)     
        {
          digitalWrite(LED, LOW);
          break;
          }
            }
       }    
}
void PID()
{
  position = qtra.readLine(sensorValues, true, false);

  
  proporcional = ((int)position) - Target;

  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  last_prop = proporcional;


  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;


  int diferencial = ( proporcional * KP ) + ( derivativo * KD ) + (integral * Ki) ;

  if ( diferencial > Velmax ) diferencial = Velmax;
  else if ( diferencial < -Velmax ) diferencial = -Velmax;

  ( diferencial < 0 ) ?
  Motor(Velmax + diferencial, Velmax) : Motor(Velmax, Velmax - diferencial);
}
void setup()
{
  // Declaramos como salida los pines utilizados
  pinMode(LED   , OUTPUT);
  pinMode(BIN2  , OUTPUT);
  pinMode(LED  , OUTPUT);
  pinMode(BIN1  , OUTPUT);
  pinMode(PWMB  , OUTPUT);
  pinMode(AIN1  , OUTPUT);
  pinMode(AIN2  , OUTPUT);
  pinMode(PWMA  , OUTPUT);
  pinMode(BOTON, INPUT);
  pinMode(BOTON1, INPUT);
  //pinMode(BUZZER, OUTPUT);

 WaitBoton(); //Presiono para iniciar la calibración
  // Calibramos con la función qtra.calibrate();, y dejamos parpadeando el led, mientras se produce la calibración.
  //WaitBoton(); //PRESIONO BOTON PARA CALIBRAR
  for ( int i = 0; i < 70; i++)
  {
    digitalWrite(LED, HIGH); delay(20);
    qtra.calibrate();
    digitalWrite(LED, LOW);  delay(20);
  }
  
  delay(1000);
  WaitGO(); //YA CALIBRADO PRESIONO BOTON PARA CONTINUAR
}

void loop(){
   if (digitalRead(GO) == LOW)
  {
    // Si el pin 10 está en LOW, detener el seguidor de línea
    freno(true, true, 0);
  }
  else
  {
    // Continuar con el seguimiento de línea
    PID();
    frenos_contorno(600);
  }
}
