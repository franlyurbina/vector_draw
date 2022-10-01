#include <TinyStepper_28BYJ_48.h>
#include <Servo.h>
#include <SD.h>

#define STEPS_PER_TURN  (2048) //el motor paso a paso gira 360 grados por cada 2048 pasos
#define SPOOL_DIAMETER  (35)   //diametro del carrete en milimetros
#define SPOOL_CIRC      (SPOOL_DIAMETER * 3.1416) //perímetro de la circunferencia en milimetros = 35*3.14=109.956
#define TPS             (SPOOL_CIRC / STEPS_PER_TURN)//Unidad de longitud de cuerda en cada paso: 0.053689mm 
#define LIMXMAX         550 //Distancia de las cuerdas. Ancho del plano de dibujo.
#define LIMXMIN         (0) //Inicio del eje X
#define LIMYMIN         (440) //Punto medio de la altura del plano del dibujo
#define LIMYMAX         (-LIMYMIN)
#define PLOTTER_UP      110 //Angulo del servo para separar el lapiz
#define PLOTTER_DOWN    75 //Angulo del servo para unir el lapiz
#define BAUD            (9600) //Logs

Servo plotter; //Objeto servo
TinyStepper_28BYJ_48 stepperR; //Objeto motor R
TinyStepper_28BYJ_48 stepperL; //Objeto motor L

static long last_stepperR_position;
static long last_stepperL_position;
static float currentX;
static float currentY;

//Vector unitario, define la posicion del lapiz
float get_position(float dx, float dy) { 
  return round(sqrt(dx * dx + dy * dy) / TPS);
}

//Siguiente coordenada
void next_coordinate(float x, float y) {
  long current_stepperR_position = get_position(x - LIMXMIN, y - LIMYMIN);
  long current_stepperL_position = get_position(x - LIMXMAX, y - LIMYMIN);

  currentX = x;
  currentY = y;
  last_stepperR_position = current_stepperR_position;
  last_stepperL_position = current_stepperL_position;
}

//Siguiente giro del motor
void next_movement(float x, float y) {
  long current_stepperR_position = get_position(x - LIMXMIN, y - LIMYMIN);
  long current_stepperL_position = get_position(x - LIMXMAX, y - LIMYMIN);
  long distanceR = current_stepperR_position - last_stepperR_position;
  long distanceL = current_stepperL_position - last_stepperL_position;
  int stepsR = distanceR > 0 ? -1 : 1;
  int stepsL = distanceL > 0 ? 1 : -1;
  int over = 0;

  long ad_1 = abs(distanceR);
  long ad_2 = abs(distanceL);
  int flag = 0;

  if (ad_1 <= ad_2) { //caso2
    ad_1 = abs(distanceL);
    ad_2 = abs(distanceR);
    flag = 1;
  }

  for (int i = 0; i < ad_1; ++i) {
    if (flag) { //caso 2
      stepperL.moveRelativeInSteps(stepsL);
    } else {
      stepperR.moveRelativeInSteps(stepsR);
    }
    over += ad_2;
    if (over >= ad_1) {
      over -= ad_1;
      if (flag) { //caso2
        stepperR.moveRelativeInSteps(stepsR);
      } else {
        stepperL.moveRelativeInSteps(stepsL);
      }
    }
    delayMicroseconds(2);
  }

  last_stepperR_position = current_stepperR_position;
  last_stepperL_position = current_stepperL_position;
  currentX = x;
  currentY = y;
}


void draw_line(float x, float y) {
  float dx = x - currentX;
  float dy = y - currentY;
  float distance = sqrt(dx * dx + dy * dy);

  if (distance > TPS) {
    float pieces = floor(distance / TPS);
    float a;
    for (float j = 0; j <= pieces; ++j) {
      a = j / pieces;

      next_movement((x - currentX)*a + currentX, (y - currentY)*a + currentY);
    }
  }
  next_movement(x, y);
}

int read_coordinates(String coordinates) {
  String xx, yy;
  float x, y;
  int px, py, activate_servo, servo;
  int isCoordinate = 1;

  coordinates.toUpperCase();
  activate_servo = coordinates.indexOf('M');
  px = coordinates.indexOf('X');
  py = coordinates.indexOf('Y');

  if (px == -1 || py == -1) isCoordinate = 0;

  if (activate_servo != -1) {
    servo = coordinates.substring(activate_servo + 1, activate_servo + 3).toInt();
    if (servo == 5) plotter.write(PLOTTER_UP);
    if (servo == 3) plotter.write(PLOTTER_DOWN);
  } else {
    activate_servo = coordinates.length();
  }

  xx = coordinates.substring(px + 1, py);
  yy = coordinates.substring(py + 1, activate_servo);

  if (isCoordinate) draw_line(xx.toFloat(), yy.toFloat());
}


void setup() {
  Serial.begin(BAUD);
  stepperR.connectToPins(7, 8, 9, 10);
  stepperL.connectToPins(2, 3, 5, 6);
  stepperR.setSpeedInStepsPerSecond(10000);
  stepperR.setAccelerationInStepsPerSecondPerSecond(100000);
  stepperL.setSpeedInStepsPerSecond(10000);
  stepperL.setAccelerationInStepsPerSecondPerSecond(100000);

  plotter.attach(A0);
  plotter.write(PLOTTER_UP);
  next_coordinate(0, 0);

  if (!SD.begin(53)) {
    Serial.println("initialization SD failed!");
    while (1);
  }

  Serial.println("START PROGRAM");
}


void loop() {
  String filename = "file.txt";
  Serial.print(filename);
  
  File gcode_file = SD.open(filename);
  
  if (gcode_file) {
    Serial.println(" opened correctly.");
    
    String line_file = "";
    int number_line = 0;
    char char_line = 0;
    
    while (gcode_file.available()) {
      char_line = gcode_file.read();

      if ((byte) char_line == 255) {
        gcode_file.close();
        Serial.print("ERROR 255 - memory overflow.");
        while (1);
        
      } else if (char_line == char(10)) {
        number_line++;
        Serial.print("[read_coordinates] line: ");
        Serial.print(number_line);
        Serial.println(" : " + line_file);
        read_coordinates(line_file);
        line_file = "";
      }  else {
        line_file += char_line;
      }
    }
    gcode_file.close();
  } else {
    Serial.println(" cannot be opened. File error.");
  }
  while (1);
}
