#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h> // Librería para funciones matemáticas como sqrt (raíz cuadrada)

// Configuración del LCD I2C (pantalla con dirección 0x27, 16 columnas y 2 filas)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pines y constantes
const int irSensorPin = 2; // Pin donde está conectado el sensor infrarrojo (IR)
const float longitudPendulo = 0.3; // Longitud del péndulo en metros (30 cm)
const float gravedadTeorica = 9.81; // Valor teórico de la gravedad (puedes cambiarlo según tu ciudad)

volatile int conteoOscilaciones = 0; // Cuenta cuántas oscilaciones completas ha hecho el péndulo
volatile bool pasoAnterior = false;  // Ayuda a detectar el sentido del paso del péndulo
volatile bool estadoSensor = false;  // Guarda el estado actual del sensor IR
unsigned long tiempoInicio = 0;      // Guarda el tiempo cuando inicia una oscilación
unsigned long tiempoFinal = 0;       // Guarda el tiempo cuando termina una oscilación
float tiempos[20];                   // Guarda los tiempos de cada oscilación (20 valores)
const unsigned long debounceTime = 200; // Tiempo mínimo entre detecciones para evitar errores por rebote (200 ms)

// Declaración de la función que cuenta las oscilaciones (se usa en la interrupción)
void contarOscilacion();

void setup() {
  // Configura el pin del sensor IR como entrada con resistencia interna
  pinMode(irSensorPin, INPUT_PULLUP);

  // Activa la interrupción para detectar cambios en el sensor IR
  attachInterrupt(digitalPinToInterrupt(irSensorPin), contarOscilacion, CHANGE);

  // Inicializa la pantalla LCD y muestra un mensaje de bienvenida
  lcd.init();
  lcd.backlight();
  lcd.print("Pendulo Simple");
  delay(2000); // Espera 2 segundos para que se lea el mensaje
  lcd.clear();

  // Inicializa la comunicación con la computadora para mostrar resultados
  Serial.begin(9600);
  Serial.println("Pendulo Simple - Esperando oscilaciones...");
  lcd.print("Esperando...");
}

void loop() {
  // Cuando se detectan 20 oscilaciones completas, se procesan los datos
  if (conteoOscilaciones >= 20) {
    // Detiene la interrupción para que no se cuenten más oscilaciones
    detachInterrupt(digitalPinToInterrupt(irSensorPin));

    // Suma todos los tiempos de las oscilaciones
    float sumaTiempos = 0;
    for (int i = 0; i < 20; i++) {
      sumaTiempos += tiempos[i];
    }
    // Calcula el periodo promedio (tiempo de una oscilación completa)
    float periodoPromedio = sumaTiempos / 10.0;

    // Ajuste de 1/4 de oscilación para mayor precisión experimental
    float ajusteCuarto = periodoPromedio / 20.0;
    periodoPromedio += ajusteCuarto;

    // Calcula la desviación estándar (qué tanto varían los tiempos)
    float sumaDesviaciones = 0;
    for (int i = 0; i < 20; i++) {
      sumaDesviaciones += pow(tiempos[i] - periodoPromedio, 2);
    }
    float desviacionEstandar = sqrt(sumaDesviaciones / 19); // 19 porque es una muestra, no toda la población
    float incertidumbre = desviacionEstandar / sqrt(20);    // Incertidumbre estándar del periodo

    // Calcula la gravedad experimental usando la fórmula del péndulo
    float gravedad = (4 * PI * PI * longitudPendulo) / (periodoPromedio * periodoPromedio);

    // Calcula la incertidumbre en la gravedad (propagación de errores)
    float incertidumbre_g = gravedad * 2 * (incertidumbre / periodoPromedio);

    // Calcula el error absoluto respecto a la gravedad teórica
    float errorAbsoluto = fabs(gravedad - gravedadTeorica);

    // Calcula el error relativo respecto a la gravedad teórica (en %)
    float errorRelativo = (errorAbsoluto / gravedadTeorica) * 100.0;

    // Calcula el coeficiente de variabilidad de g (en %)
    float coefVariabilidad = (incertidumbre_g / gravedad) * 100.0;

    // Muestra los resultados principales en la pantalla LCD
    lcd.clear();
    lcd.print("g = ");
    lcd.print(gravedad, 2); // Muestra la gravedad con 2 decimales
    lcd.setCursor(0, 1);
    lcd.print("T = ");
    lcd.print(periodoPromedio, 2); // Muestra el periodo promedio con 2 decimales

    delay(3000); // Espera 3 segundos para que se lean los resultados
    lcd.clear();
    lcd.print("Incertidumbre:");
    lcd.setCursor(0, 1);
    lcd.print("+/- ");
    lcd.print(incertidumbre_g, 4); // Muestra la incertidumbre de g con 4 decimales

    // Muestra los resultados detallados en la computadora (monitor serie)
    Serial.println("=================================");
    Serial.println("Resultados:");
    Serial.print("Periodo promedio (T): ");
    Serial.print(periodoPromedio, 4);
    Serial.println(" s");
    Serial.print("Gravedad calculada (g): ");
    Serial.print(gravedad, 4);
    Serial.println(" m/s^2");
    Serial.print("Incertidumbre de g: +/- ");
    Serial.print(incertidumbre_g, 4);
    Serial.println(" m/s^2");
    Serial.print("Error absoluto de g: ");
    Serial.print(errorAbsoluto, 4);
    Serial.println(" m/s^2");
    Serial.print("Error relativo de g: ");
    Serial.print(errorRelativo, 2);
    Serial.println(" %");
    Serial.print("Coeficiente de variabilidad de g: ");
    Serial.print(coefVariabilidad, 2);
    Serial.println(" %");
    Serial.println("=================================");

    // Muestra la lista de periodos individuales de cada oscilación
    Serial.println("Oscilaciones Individuales:");
    float sumaOscilaciones = 0;
    for (int i = 0; i < 10; i++) {
      float periodo = tiempos[2*i] + tiempos[2*i+1];
      if (i == 9) { // Solo a la última oscilación se le suma el ajuste de 1/4
        periodo += ajusteCuarto;
      }
      sumaOscilaciones += periodo;
      Serial.print("T");
      Serial.print(i+1);
      Serial.print(" = ");
      Serial.print(periodo, 4);
      Serial.println(" s");
    }
    Serial.print("Suma T1 a T10: ");
    Serial.print(sumaOscilaciones, 4);
    Serial.println(" s");
    Serial.println("=================================");

    // Detiene el programa para no repetir el cálculo
    while (true);
  }
}

// Esta función se ejecuta automáticamente cada vez que el sensor IR detecta el paso del péndulo
void contarOscilacion() {
  unsigned long tiempoActual = millis(); // Toma el tiempo actual en milisegundos

  // Ignora señales muy seguidas (rebotes o errores)
  if (tiempoActual - tiempoInicio < debounceTime) {
    return;
  }

  // Lee el estado actual del sensor IR (si detecta el péndulo o no)
  bool estadoActual = digitalRead(irSensorPin) == LOW;

  // Si el estado cambió, procesa la oscilación
  if (estadoActual != estadoSensor) {
    estadoSensor = estadoActual;

    if (estadoSensor) { // Solo cuenta cuando el sensor detecta el paso
      if (!pasoAnterior) {
        // Primer paso detectado (inicio de oscilación)
        pasoAnterior = true;
      } else {
        // Segundo paso detectado (oscilación completa)
        pasoAnterior = false;

        if (conteoOscilaciones == 0) {
          // Marca el inicio de la primera oscilación
          tiempoInicio = tiempoActual;
        } else {
          // Guarda el tiempo de la oscilación en el arreglo (en segundos)
          tiempoFinal = tiempoActual;
          tiempos[conteoOscilaciones - 1] = (tiempoFinal - tiempoInicio) / 1000.0;
          tiempoInicio = tiempoFinal;
        }

        // Incrementa el conteo de oscilaciones
        conteoOscilaciones++;
        Serial.print("Oscilacion completa: ");
        Serial.println(conteoOscilaciones);
      }
    }
  }
}