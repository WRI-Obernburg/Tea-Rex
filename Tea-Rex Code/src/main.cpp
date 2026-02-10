#include <SoftwareSerial.h>            // Bibliothek für serielle Kommunikation über beliebige Pins (Softwareserial)
#include <DFRobotDFPlayerMini.h>       // Bibliothek zur Steuerung des DFPlayer Mini MP3-Moduls
#include <Servo.h>                     // Bibliothek zur Steuerung eines Servomotors
#include <Wire.h>                      // Bibliothek für I2C-Kommunikation (für das OLED Display)
#include <Adafruit_SSD1306.h>          // Bibliothek für das SSD1306 OLED-Display

// === Display Einstellungen ===
#define SCREEN_WIDTH 128               // Breite des OLED-Displays in Pixel
#define SCREEN_HEIGHT 64               // Höhe des OLED-Displays in Pixel
#define OLED_RESET (-1)                  // Kein Reset-Pin beim Display (-1 bei I2C Displays)
Adafruit_SSD1306* display = nullptr; // dynamisches Display-Objekt (optional)
bool displayPresent = false; // Flag: true, wenn ein OLED-Display erfolgreich initialisiert wurde

#define posHigh 0                     // Obere Position des Servos (Teebeutel oben) // Schrittweite bei der Bewegung des Servos (Geschwindigkeit)
#define posLow 60                     // Untere Position des Servos (Teebeutel unten)
#define Speed 1                       // Schrittweite bei der Bewegung des Servos (Geschwindigkeit)

#define buttonPin 2
#define potiPin A0

// === Servo ===
Servo myServo;                       // Servo-Objekt erstellen, um Servo anzusteuern
int servoPos = 0;                    // Variable speichert aktuelle Position des Servos (in Grad)
bool servoStatus = true;             // True = Teebeutel oben, False = Teebeutel unten

// === MP3-Modul ===
SoftwareSerial mp3Serial(10, 11);    // Software-Serial auf Pin 10 (RX), 11 (TX) für MP3-Modul initialisieren
DFRobotDFPlayerMini mp3;             // Objekt für MP3-Modul erstellen

// === Taster ===
bool lastButtonState = HIGH;         // Speichert letzten Tasterzustand (HIGH = ungedrückt durch Pullup)
bool buttonPressed = false;          // Flag, ob Taste als gedrückt erkannt wurde (für Entprellung)
unsigned long lastDebounceTime = 0; // Zeit, wann Taster zuletzt umgeschaltet hat (für Entprellung)
const unsigned long debounceDelay = 50; // Entprellzeit in ms (50 ms üblich)

// === Countdown Timer ===
bool countdownActive = false;        // Flag, ob Countdown gerade läuft
unsigned long countdownDuration = 0; // Countdown-Dauer in ms (wird vom Poti bestimmt)
unsigned long countdownStartTime = 0;// Startzeitpunkt des Countdowns (millis())
int countdownSeconds = 0;            // Countdown-Dauer in Sekunden (zur Anzeige)

// === Variablen zur Erkennung für Dauer langer Tastendruck ===
bool resetCountdownActive = false;   // Flag, ob Reset-Countdown läuft
unsigned long buttonPressStart = 0;  // Zeitpunkt, wann der Taster gedrückt wurde
unsigned long resetCountdownStart = 0; // Startzeit des Reset-Countdowns
const unsigned long longPressThreshold = 1500; // Zeit (ms), die gedrückt werden muss für Reset (1,5 Sekunden)
int resetTimeRemaining = 3000;       // Reset-Countdown Zeit in ms

// === Nach-Countdown MP3 Wiedergabe ===
bool postCountdownPlaying = false;   // Flag, ob MP3 nach Countdown läuft
unsigned long postCountdownStartTime = 0;  // Startzeit des Post-Countdown MP3s  TODO
//const unsigned long postCountdownDuration = 30000; // Dauer des MP3 nach Countdown (30 Sekunden) TODO

void setup() {                    // Initialisiert Komponenten
  Serial.begin(9600);                // Serielle Kommunikation für Debugging starten (Baudrate 9600)
  mp3Serial.begin(9600);             // Serielle Kommunikation mit MP3-Modul starten

  myServo.attach(9);                       // Servo an digitalen Pin 9 anschließen
  myServo.write(posHigh);                  // Servo auf Startposition 0 Grad setzen
  delay(10);                               // Kurze Pause zur Stabilisierung
  myServo.detach();                        // Servo trennen (spart Strom und verhindert Zucken)


  // Initialisierung des OLED-Displays (optional)
  // Wir erstellen das Display-Objekt zur Laufzeit, damit der Code nicht von einem vorhandenen Wire-Objekt in der globalen Initialisierung abhängig ist.
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  if (display && display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Wenn Start erfolgreich
    displayPresent = true;
    display->clearDisplay();             // Display-Inhalt löschen
    display->setTextColor(SSD1306_WHITE);// Textfarbe auf Weiß setzen
    display->setTextSize(1);             // Textgröße auf klein (1) setzen
    display->setCursor(0, 0);            // Cursor auf Position (0,0) setzen
    display->display();                  // Display aktualisieren (leeren)
  } else {
    // Falls das Display nicht initialisiert werden kann, geben wir eine Nachricht per Serial aus und löschen das Objekt
    displayPresent = false;
    Serial.println(F("SSD1306 allocation failed or not present"));
    if (display) {
      delete display;
      display = nullptr;
    }
  }

  // MP3-Modul initialisieren
  if (!mp3.begin(mp3Serial)) {        // Wenn Initialisierung fehlschlägt
    if (displayPresent && display) {
      display->setCursor(0, 10);         // Cursor versetzen
      display->println(F("MP3 not found!"));  // Meldung ausgeben
      display->display();                // Anzeige aktualisieren
    } else {
      Serial.println(F("MP3 not found!"));
    }
    while (true);                     // Endlosschleife, da MP3 nicht vorhanden
  }
  mp3.volume(20);                    // Lautstärke des MP3-Moduls einstellen (0-30)
  pinMode(buttonPin, INPUT_PULLUP);  // Taster-Pin als Eingang mit internem Pullup-Widerstand konfigurieren

} // Ende Vorkonfiguration


void lowerTeabag() {
  myServo.attach(9);                                         // Servo aktivieren
  for (int pos = posHigh; pos <= posLow; pos += Speed) {     // Von oben nach unten (neue Variable pos vom Typ int (Ganzzahl) erstellen; Bedingung, bei der die Schleife weiterlaufen darf; nach jedem Schleifendurchlauf wird pos um den Wert Speed erhöht)
    myServo.write(pos);                                      // Position setzen
    delay(15);                                               // Kurze Pause für gleichmäßige Bewegung
  }
  servoStatus = false;                                        // Merken: Teebeutel ist unten
  myServo.detach();                                           // Servo deaktivieren
}

void liftTeabag() {
  myServo.attach(9);                                          // Servo aktivieren
  for (int pos = posLow; pos >= posHigh; pos -= Speed) {      // Von unten nach oben (neue Variable pos vom Typ int (Ganzzahl) erstellen; Bedingung, bei der die Schleife weiterlaufen darf; nach jedem Schleifendurchlauf wird pos um den Wert Speed verringert)
    myServo.write(pos);                                       // Position setzen
    delay(15);                                                // Kurze Pause für gleichmäßige Bewegung
  }
  servoStatus = true;                                         // Teebeutel ist oben
  myServo.detach();                                           // Servo deaktivieren
}


// Soft Reset Funktion zum Zurücksetzen des Programms in den Anfangszustand, ohne das System physisch aus- und wieder einzuschalten
void softReset() {                // Funktion, die alle Variablen und Zustände zurücksetzt (wie ein Neustart), void gibt keinen Wert zurück
  mp3.stop();                     // Stoppt jede MP3-Wiedergabe (falls aktiv)
  if(servoStatus == false)        // Wenn der Teebeutel noch unten ist
  {
    liftTeabag();                // Servo auf 0 Grad setzen
  }
  delay(500);                     // Kleine Pause für die Bewegung

  // Alle Flags und Variablen zurücksetzen
  countdownActive = false;          // Kein Countdown aktiv
  postCountdownPlaying = false;     // Nach-Countdown MP3 ist beendet
  resetCountdownActive = false;     // Kein Reset-Countdown aktiv
  buttonPressStart = 0;             // Startzeit für Reset drücken zurücksetzen
  resetCountdownStart = 0;          // Startzeit des Reset-Countdowns zurücksetzen
  resetTimeRemaining = 3;        // Countdown-Dauer auf Standard zurücksetzen

  // Display zurücksetzen (nur wenn vorhanden)
  if (displayPresent && display) {
    display->clearDisplay();         // Anzeige löschen
  }
}                                 // Ende der Funktion softReset()

// Funktion zur Behandlung des langen Tastendrucks zum Reset
void handleResetButton(bool buttonState) {    // Funktion zur Auswertung eines langen Tastendrucks (Reset auslösen)
  unsigned long currentMillis = millis();  // Holt die aktuelle Zeit in Millisekunden (Systemzeit seit Start)

  if (buttonState == LOW) {                  // Wenn Taste gedrückt (LOW wegen INPUT_PULLUP)
    if (buttonPressStart == 0) {             // Wenn noch keine Startzeit für Tastendruck gespeichert ...
      buttonPressStart = currentMillis;      //  ... speichere den aktuellen Zeitpunkt als Start
    }                                        // Ende der Prüfung: Startzeit erfassen.

    unsigned long pressDuration = currentMillis - buttonPressStart;  // Berechne wie lange die Taste gehalten wird.
    if (!resetCountdownActive && pressDuration >= longPressThreshold) { // Wenn noch kein Reset läuft, aber die Taste schon lang genug gedrückt wurde (1,5 Sekunden)
      resetCountdownActive = true;           // Setze Flag, dass der Reset-Countdown läuft
      resetCountdownStart = currentMillis;   // Merke den Startzeitpunkt des Reset-Countdowns
      resetTimeRemaining = 2000;              // Reset-Countdown auf 3000 ms setzen
    }                                         //Ende der Prüfung auf langen Tastendruck

    if (resetCountdownActive) {                                        // Wenn Reset-Countdown läuft
      unsigned long elapsed = currentMillis - resetCountdownStart;     // Berechnet vergangene Zeit seit Reset-Start
      int newRemaining = 2000 - (elapsed);                         // Berechenet verbleibende Sekunden
      if (newRemaining < 0) newRemaining = 0;                          // Falls negativ auf 0 setzen

      if (newRemaining != resetTimeRemaining) {                    // Wenn sich Restzeit geändert hat
        resetTimeRemaining = newRemaining;                         // aktualisiere die angezeigte Restzeit
      }                                                               // Ende: Restzeit-Anzeige aktualisieren

      if (resetTimeRemaining == 0) {                               // Wenn Reset-Zeit abgelaufen ist
        if (displayPresent && display) {
          display->clearDisplay();                                       // Display löschen
          display->setCursor(10, 20);                                    // Position für "Reset!" setzen
          display->setTextSize(3);                                       // Schriftgroeße
          display->println(F("Reset!"));                                 // Text "Reset!" anzeigen
          display->display();                                            // Anzeige aktualisieren
        } else {
          Serial.println(F("Reset!"));
        }
        delay(500);                                                   // Kurze Pause, damit der Nutzer den Reset-Text sieht
        softReset();          // Aufruf der Soft-Reset-Funktion = zurück zum Anfangszustand
        Serial.println("Reset!");
      }                                                              // Ende: Reset-Zeit abgelaufen
    }                                                            // Ende: Reset-Countdown läuftsetup
  } else {                           // Wenn Taste losgelassen wurde
    buttonPressStart = 0;            // Startzeit zurücksetzen
    resetCountdownActive = false;    // Resetvorgang abbrechen
  }                                  // Ende der else-Bedingung (Taste losgelassen)
}                                  // Ende der Funktion handleResetButton


void loop() {                       //Hauptprogramm  läuft in Endlosschleife

  // Zuerst den Reset-Taster auswerten und ggf. Reset-Logik ausführen
  handleResetButton(digitalRead(buttonPin)); // Liest den aktuellen Zustand des Buttons und übergibt ihn an die Funktion, die prüft, ob ein Reset nötig ist (Langdruck > 1,5s

    // Poti nur lesen, wenn Countdown nicht aktiv ist und in Schritte umrechnen
  if (!countdownActive) {                   // Wenn aktuell kein Countdown läuft, darf der Poti zur Neueinstellung der Zeit ausgelesen werden
    int potiValue = analogRead(potiPin);    // Poti analog auslesen (0 bis 1023)
    int steps = map(potiValue, 0, 1023, 40, 0); // Poti-Wert in Schritte 0 bis 10 mappen (invertiert) 600s/15s=40 Steps
    countdownSeconds = steps * 15;             // Schritte in Sekunden umrechnen (jede Stufe = 60 Sekunden)
    countdownDuration = (unsigned long)countdownSeconds * 1000UL; // Sekunden in Millisekunden umwandeln
  }                                     // Ende der Bedingung für die Poti-Auswertung

  bool reading = digitalRead(buttonPin); // Aktuellen Zustand des Tasters lesen (LOW = gedrückt, HIGH = nicht gedrückt)

  // Wenn kein Countdown läuft, Servo nicht in Bewegung, kein Reset aktiv und MP3 nicht im Post-Countdown-Modus
  // und eine gültige Countdown-Zeit eingestellt ist, dann Taster-Input entprellen und auf gedrückt prüfen
  if (!countdownActive && !resetCountdownActive && !postCountdownPlaying && countdownSeconds > 0) { // Diese Bedingungen verhindern einen Start, wenn bereits ein Countdown läuft, der Servo sich bewegt, ein Reset läuft oder nach dem Countdown noch Audio spielt es muss eine gültige Zeit (> 0) eingestellt sein
    if (reading != lastButtonState) {   // Wenn Tasterzustand sich seit letztem Loop geändert hat
      lastDebounceTime = millis();      // Entprell-Timer starten: aktuelle Zeit merken
    }                                // Ende der Flankenerkennung
    if ((millis() - lastDebounceTime) > debounceDelay) {  // Wenn seit der letzten Änderung mehr als 50ms vergangen sind = entprellt
      if (reading == LOW && !buttonPressed) {            // Taste wurde gedrückt (LOW) und vorher nicht als gedrückt gemerkt wurde
        buttonPressed = true;                             // Taste als gedrückt merken ------TODO durch Funktion ersetzen
        mp3.play(1);                                      // MP3-Track1 abspielen
        lowerTeabag();  
        countdownActive = true;          // Countdown starten
        countdownStartTime = millis();   // Startzeit merken

      }                                                   // Ende der Bedingung: Taste wurde erkannt
    }                                             // Ende der Entprellung
    if (reading == HIGH) {             // Wenn Taste losgelassen = nicht gedrückt
      buttonPressed = false;           // Taste als nicht gedrückt markieren = freigeben, damit sie erneut gedrückt werden kann
    }                                   // Ende des Loslassens
  }                                 // Ende der Startbedingungen
  lastButtonState = reading;           // Aktuellen Tasterzustand für nächste Schleife speichern

  // Countdown läuft
  if (countdownActive) {                                    // Wenn Countdown gerade aktiv ist
    unsigned long elapsed = millis() - countdownStartTime;  // Berechne Zeit seit Countdown-Start
    long remaining = countdownDuration - elapsed;           // Verbleibende Zeit berechnen
    if (remaining < 0) remaining = 0;                        // Nicht negativ werden lassen

    int secondsLeft = remaining / 1000;                      // Rechne verbleibende Sekunden
    int minutes = secondsLeft / 60;                          // Minutenanteil
    int seconds = secondsLeft % 60;                          // Sekundenanteil

    if (remaining == 0) {                                    // Wenn Countdown abgelaufen
      countdownActive = false;                               // Countdown stoppen
      liftTeabag();                                          // Servo fährt zurück auf 0 Grad
      postCountdownPlaying = true;                           // "Nach-Countdown-MP3" Track2 wird gestartet
      postCountdownStartTime = millis();                     // Startzeit des MP3s merken
      mp3.play(2);                                           // MP3 Track2 starten
    }

    // Countdownzeit auf Display anzeigen
    if (displayPresent && display) {
      display->clearDisplay();                                  // Display löschen
      display->setCursor(10, 25);                           // Cursor setzen
      display->print(minutes);                                  // Minuten anzeigen
      display->print(F("m "));                                  // "m" für Minuten
      display->print(seconds);                                  // Sekunden anzeigen
      display->print(F("s"));                                   // "s" für Sekunden
      display->setTextSize(1);                                // Schriftgröße zurücksetzen
      delay(50);                                            // Notwendig wegen Display Aufblitzen
      display->setCursor(0, 0);                             // Cursor oben links setzen, von oben getauscht... TIMING!
      display->println(F("Fertig in..."));                      // Text anzeigen, von oben getauscht... TIMING!
      display->setTextSize(3);                                // Große Schrift, von oben getauscht... TIMING!
      display->display();                                       // Display aktualisieren
    } else {
      // optional: print to Serial for debugging when no display is connected
      Serial.print("Countdown: ");
      Serial.print(minutes);
      Serial.print("m ");
      Serial.print(seconds);
      Serial.println("s");
    }
  }// Ende des Countdown-Blocks

  // Nach Countdown, MP3 Track2 wird abgespielt
  if (postCountdownPlaying == true) {

    if (digitalRead(buttonPin) == LOW) {  // Wenn Taste gedrückt wird während MP3 Track 2 läuft
      delay(50);                         // Kleine Pause gegen Prellen
      if (digitalRead(buttonPin) == LOW) { // Wenn Taste immer noch gedrückt ist
        softReset();                      // Soft-Reset auslösen
      }                                   // Ende zweite Prüfung: Taste gedrückt halten
    }                                 // Ende: Taste gedrückt in Post-Phase

    if (displayPresent && display) {
      display->clearDisplay();                // Display löschen um "Fertig!" anzuzeigen
      display->setTextSize(3);                // Große Schrift
      display->setCursor(2, 20);              // Cursor setzen
      display->println(F("FERTIG!"));         // Text anzeigen
      display->display();                    // Anzeige aktualisieren
    } else {
      Serial.println(F("FERTIG!"));
    }

    if (mp3.readState() == -1) { // Readstate gibt bei Ende des Tracks ein "-1" aus
      postCountdownPlaying = false;       // MP3-Flag zurücksetzen = wieder bereit
      if (displayPresent && display) {
        display->clearDisplay();              // Display löschen
        display->setCursor(0, 20);            // Cursor setzen
        display->println(F("Bereit"));        // "Bereit" anzeigen
        display->display();                   // Display aktualisieren
      } else {
        Serial.println(F("Bereit"));
      }
    }// Ende: Post-Countdown-Zeit ist abgelaufen
  }// Ende des "postCountdownPlaying"

  // "Bereitzustand" anzeigen, wenn nichts läuft (kein Countdown, keine Bewegung, kein Reset, kein MP3)
  if (!countdownActive && !resetCountdownActive && !postCountdownPlaying) { // wenn Countdown nicht läuft und der Servo nicht gerade nicht auf 60° fährt und gerade nicht auf 0° fährt und kein Resetcountdown aktiv und kein Track2 gespielt wird

    // Poti-Countdownzeit anzeigen
    if (displayPresent && display) {
      display->clearDisplay();             // Display löschen
      display->setCursor(10, 25);          // Cursor setzen
      display->setTextSize(3);             // Schriftgroeße
      display->print(countdownSeconds / 60);  // Minuten anzeigen
      display->print(F("m "));             // "m" für Minuten
      display->print(countdownSeconds % 60);  // Sekunden anzeigen
      display->println(F("s"));            // "s" für Sekunden
      display->display();                  // Anzeige aktualisieren
    } else {
      Serial.print(F("Ready: "));
      Serial.print(countdownSeconds / 60);
      Serial.print(F("m "));
      Serial.print(countdownSeconds % 60);
      Serial.println(F("s"));
    }
  }// Ende der Anzeige im Leerlauf

  delay(10);                           // Kleine Pause, um den Loop nicht zu überlasten
}// Ende void loop




