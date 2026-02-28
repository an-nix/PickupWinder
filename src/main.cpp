#include <Arduino.h>
#include <FastAccelStepper.h>

#define STEP_PIN    32
#define DIR_PIN     33
#define ENABLE_PIN  25  // Active le driver (LOW = activé)

#define MICROSTEPPING         8     // Facteur de microstepping configuré sur le driver
#define STEPS_PER_REVOLUTION  (200 * MICROSTEPPING)  // 1600 pas/tour en 1/8 step
#define TOURS_PAR_CYCLE       30    // Nombre de tours par palier
#define VITESSE_HZ_MIN        2400  // Départ à 90 RPM en 1/8 step (300 Hz × 8)
#define VITESSE_HZ_MAX        80000 // Vitesse maximale ~3000 RPM en 1/8 step
#define VITESSE_HZ_STEP       800   // Incrément proportionnel au microstepping (100 × 8)
#define ACCELERATION          120000 // Accélération proportionnelle (15000 × 8)

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(9600);
  Serial.println("Démarrage moteur pas à pas...");

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Active le driver

  engine.init();

  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper == NULL) {
    Serial.println("Erreur : impossible d'initialiser le stepper !");
    while (true);
  }

  stepper->setDirectionPin(DIR_PIN);
  stepper->setAcceleration(ACCELERATION);

  Serial.println("Prêt. Début de la rampe de vitesse...");
}

int vitesse_courante = VITESSE_HZ_MIN;

void loop() {
  long steps = STEPS_PER_REVOLUTION * TOURS_PAR_CYCLE;

  stepper->setSpeedInHz(vitesse_courante);
  Serial.print("→ Vitesse : ");
  Serial.print(vitesse_courante);
  Serial.print(" Hz  (");
  Serial.print((float)vitesse_courante * 60.0 / STEPS_PER_REVOLUTION, 1);
  Serial.println(" RPM)");

  // Rotation dans un seul sens
  stepper->move(steps);
  while (stepper->isRunning()) delay(10);
  delay(300);

  // Incrément de vitesse
  if (vitesse_courante < VITESSE_HZ_MAX) {
    vitesse_courante += VITESSE_HZ_STEP;
  } else {
    Serial.println("=== Vitesse maximale atteinte ===");
    while (true);  // Stop
  }
}