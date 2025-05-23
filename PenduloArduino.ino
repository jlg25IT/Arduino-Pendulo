#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h> // Para cálculos matemáticos como sqrt

// Configuración del LCD I2C (dirección 0x27, 16 columnas, 2 filas)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pines
const int irSensorPin = 2; // Pin del sensor IR para interrupciones
const float longitudPendulo = 0.3; // Longitud del péndulo en metros (30 cm)
const float gravedadTeorica = 9.81; // Valor teórico de la gravedad (puedes ajustarlo según tu localidad)

volatile int conteoOscilaciones = 0; // Cuenta las oscilaciones completas
volatile bool pasoAnterior = false; // Indica si ya pasó en un sentido
volatile bool estadoSensor = false; // Estado actual del sensor
unsigned long tiempoInicio = 0;
unsigned long tiempoFinal = 0;
float tiempos[20]; // Almacena los tiempos de cada oscilación
const unsigned long debounceTime = 200; // Tiempo mínimo entre detecciones (200 ms)

void contarOscilacion();

void setup() {
  pinMode(irSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(irSensorPin), contarOscilacion, CHANGE);

  lcd.init();
  lcd.backlight();
  lcd.print("Pendulo Simple");
  delay(2000);
  lcd.clear();

  Serial.begin(9600);
  Serial.println("Pendulo Simple - Esperando oscilaciones...");
  lcd.print("Esperando...");
}

void loop() {
  if (conteoOscilaciones >= 20) {
    detachInterrupt(digitalPinToInterrupt(irSensorPin));

    float sumaTiempos = 0;
    for (int i = 0; i < 20; i++) {
      sumaTiempos += tiempos[i];
    }
    float periodoPromedio = sumaTiempos / 10.0;

    // Calcular la desviación estándar
    float sumaDesviaciones = 0;
    for (int i = 0; i < 20; i++) {
      sumaDesviaciones += pow(tiempos[i] - periodoPromedio, 2);
    }
    float desviacionEstandar = sqrt(sumaDesviaciones / 19);
    float incertidumbre = desviacionEstandar / sqrt(20);

    // Calcular la gravedad experimental
    float gravedad = (4 * PI * PI * longitudPendulo) / (periodoPromedio * periodoPromedio);

    // Incertidumbre en g (propagación de errores)
    float incertidumbre_g = gravedad * 2 * (incertidumbre / periodoPromedio);

    // Error absoluto respecto a g teórica
    float errorAbsoluto = fabs(gravedad - gravedadTeorica);

    // Error relativo respecto a g teórica (%)
    float errorRelativo = (errorAbsoluto / gravedadTeorica) * 100.0;

    // Coeficiente de variabilidad de g (%)
    float coefVariabilidad = (incertidumbre_g / gravedad) * 100.0;

    // Mostrar resultados en el LCD
    lcd.clear();
    lcd.print("g = ");
    lcd.print(gravedad, 2);
    lcd.setCursor(0, 1);
    lcd.print("T = ");
    lcd.print(periodoPromedio, 2);

    delay(3000);
    lcd.clear();
    lcd.print("Incertidumbre:");
    lcd.setCursor(0, 1);
    lcd.print("+/- ");
    lcd.print(incertidumbre_g, 4);

    // Mostrar resultados en el monitor serie
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

    // Mostrar lista de Oscilaciones Individuales (sin ajuste en la última)
    Serial.println("Oscilaciones Individuales:");
    float sumaOscilaciones = 0;
    for (int i = 0; i < 10; i++) {
      float periodo = tiempos[2*i] + tiempos[2*i+1];
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

    while (true);
  }
}

void contarOscilacion() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoInicio < debounceTime) {
    return;
  }

  bool estadoActual = digitalRead(irSensorPin) == LOW;

  if (estadoActual != estadoSensor) {
    estadoSensor = estadoActual;

    if (estadoSensor) {
      if (!pasoAnterior) {
        pasoAnterior = true;
      } else {
        pasoAnterior = false;

        if (conteoOscilaciones == 0) {
          tiempoInicio = tiempoActual;
        } else {
          tiempoFinal = tiempoActual;
          tiempos[conteoOscilaciones - 1] = (tiempoFinal - tiempoInicio) / 1000.0;
          tiempoInicio = tiempoFinal;
        }

        conteoOscilaciones++;
        Serial.print("Oscilacion completa: ");
        Serial.println(conteoOscilaciones);
      }
    }
  }
}