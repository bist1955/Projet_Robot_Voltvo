/* ************
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/
 
/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */
 
#include <LibRobus.h> // Essentielle pour utiliser RobUS
#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include <LiquidCrystal.h>
#define CHAT 0
#define CHIEN 1
 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
 
/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces
 
bool bumperArr;
int pinU1 = A6;
int pinU2 = A5;
int pinU3 = A7;
int pin5kHz = A10;
int pinAmbiant = A11;

int etat = 0; // 0 arrêt 1 = avance 2 = recule 3 = TourneDroit 4 = TourneGauche
int etatPast = 0;
float vitesse = 0.2f;
float vitesse0 = 0.2f;
float vitesse1 = 0.2f;
int e1 = 0;
int e2 = 0;
int timer = 0;
int pulseMaxParSec = 10667;
float kp0 = 0.000096;
float kp1 = 0.00008;
float ki0 = 0.0;
float ki1 = 0.0;
float kd0 = 0;
float kd1 = 0;
int iterationsParSeconde = 50;
long deltaT = 1000.0/iterationsParSeconde;
float integralesErreur0=0;
float integralesErreur1=0;
int pulsesPrecedents0=0;
int pulsesPrecedents1=0;
int erreurPrecedente0 = 0;
int erreurPrecedente1 = 0;
 
bool start = false;
bool modeDepart = true;
bool obstacleDevant = false;
bool sonDetecte = false;
bool centreLibre =false;
bool parcoursFini= false;
int voie = 1; // 0 = gauche 1 = centre 2 = droite
int bp_r = 47;
int bp_v = 49;
int bp_j = 2;
int menuActuel = 0; // 0: principal, 1: manuel, 2: nourrir
int delai_affichage = 20;
bool last_bp_r = false;
bool last_bp_v = false;
LiquidCrystal lcd(38, 39, 44, 45, 46, 48, 40, 41, 42, 43);
uint16_t rouge, vert, bleu, clear;

int lastDirection = 0; // Derniere direction sur le suiveur de ligne (-1 = ligne à gauche, 1 = ligne à droite, 0 = centré / inconnu)

// Côtés physiques sur le robot par rapport à la ligne présentement
const int COTE_EXT = 0;
const int COTE_INT = 1;

// Position FIXE des réservoirs sur le robot (dans la vraie vie) si on regarde dans la direction des capteurs du suiveur de ligne
int reservoirChien = COTE_INT; // chien fixé à linterieur du cercle (Droite)
int reservoirChat  = COTE_EXT; // chat fixé à lexterieur du cercle (Gauche)

// Compteurs de corrections du suiveur
int nbCorrectionGauche = 0;  // corrections où le robot tourne vers la gauche
int nbCorrectionDroite = 0;  // corrections où le robot tourne vers la droite

//Capteur IR
int capteur_g = 8;
int capteur_d = 9;

int nbTourServo = 4;

/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
void avancer(int);
void reculer(int);
float calculPID(int);
void verserBouffe(int);
void afficher_menu_principal();
void resetPID();
void resetMenu();
bool checkJaune();
void recule(){
  MOTOR_SetSpeed(RIGHT,-vitesse);
  MOTOR_SetSpeed(LEFT, -vitesse);
  etat = 2;
}
void arret(int temps){
  long now;
  switch (etat)
  {
  case 1:
    now = millis();
    while(millis()-now<temps)
    {
    MOTOR_SetSpeed(RIGHT,vitesse*(temps-(millis()-now))/temps);
    MOTOR_SetSpeed(LEFT, vitesse*(temps-(millis()-now))/temps);
    }
    break;
  case 2:
   now = millis();
    while(millis()-now<temps)
    {
    MOTOR_SetSpeed(RIGHT,-vitesse*(temps-(millis()-now))/temps);
    MOTOR_SetSpeed(LEFT, -vitesse*(temps-(millis()-now))/temps);
    }
    break;
 
  case 3:
   now = millis();
  while(millis()-now<temps)
  {
  MOTOR_SetSpeed(RIGHT,-vitesse*(temps-(millis()-now))/temps);
  MOTOR_SetSpeed(LEFT, vitesse*(temps-(millis()-now))/temps);
  }
  break;
  case 4:
   now = millis();
    while(millis()-now<temps)
    {
    MOTOR_SetSpeed(RIGHT,vitesse*(temps-(millis()-now))/temps);
    MOTOR_SetSpeed(LEFT,-vitesse*(temps-(millis()-now))/temps);
    }
    break;
  default:
  MOTOR_SetSpeed(RIGHT,0);
  MOTOR_SetSpeed(LEFT, 0);
    break;
  }
  etat = 0;
 
}
//Tourne le robot à droite à l'arrêt
void tournerDroite(){
int pulsesMax = 1900;
  //Mettre à l'état tourne à droite
  etat=3;
  resetPID();
  vitesse0=vitesse;
  vitesse1=-vitesse;
  long now = millis();
  while (pulsesMax>ENCODER_Read(0)&&pulsesMax>-ENCODER_Read(1))
  {
    if(millis()-now>deltaT)
    {
      now = millis();
      MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
      MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    }
    if(checkJaune())
    {
      resetMenu();
      return;
    }
 
  }
 arret(80);
}

void tournerGauche(){
int pulsesMax = 1900;
  //Mettre à l'état tourne à droite
  etat=4;
  resetPID();
  vitesse0=vitesse;
  vitesse1=-vitesse;
  long now = millis();
  while (pulsesMax>ENCODER_Read(0)&&pulsesMax>-ENCODER_Read(1))
  {
    if(millis()-now>deltaT)
    {
      now = millis();
      MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
      MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    }
    if(checkJaune())
    {
      resetMenu();
      return;
    }
 
  }
 arret(80);
}
void tournerDroiteDance(int angle, float facteurVitesse){
int pulsesMax = angle*21.1;
  //Mettre à l'état tourne à gauche
  etat=3;
  resetPID();
  vitesse0=vitesse*facteurVitesse;
  vitesse1=-vitesse*facteurVitesse;
  long now= millis();
  while (pulsesMax>ENCODER_Read(0)&&pulsesMax>-ENCODER_Read(1))
  {
    if(millis()-now>deltaT)
    {
      now = millis();
      MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
      MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    }
    if(checkJaune())
    {
      resetMenu();
      return;
    }
 
  }
 arret(80);
}
 
void tournerJusquaLigne(int sens){
  if(sens==1)
  {
    etat=3;
    vitesse0=vitesse;
    vitesse1=-vitesse;
    long now = millis();
    while(analogRead(pinU2)<700)
    {
      if(millis()-now>10)
      {
        now = millis();
        resetPID();
        MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
        MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
      }
      if(checkJaune())
      {
        resetMenu();
        return;
      }
    }
   arret(50);
 
 }else
 {
    etat=4;
    vitesse0=-vitesse;
    vitesse1=vitesse;
    long now = millis();
    while(analogRead(pinU2)<700)
    {
      if(millis()-now>10)
      {
        now = millis();
        resetPID();
        MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
        MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
      }
      if(checkJaune())
      {
        resetMenu();
        return;
      }
    }
 }
   arret(50);
 
 }
 
 
void tournerGaucheDance(int angle, float facteurVitesse){
int pulsesMax = angle*21.1;
  //Mettre à l'état tourne à gauche
  etat=4;
  resetPID();
  vitesse0=-vitesse*facteurVitesse;
  vitesse1=vitesse*facteurVitesse;
   long now= millis();
  while (pulsesMax>-ENCODER_Read(0)&&pulsesMax>ENCODER_Read(1))
  {
    if(millis()-now>deltaT)
    {
      now = millis();
      MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
      MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    }
    if(checkJaune())
    {
      resetMenu();
      return;
    }
 
  }
 arret(80);
}
 
void beep(int count){
  for(int i=0;i<count;i++){
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);
}
 
 
//Avance et arrete a une ligne au sol
void avancerjusqualigne(){
 
    while(analogRead(pinU2)<700 && analogRead(pinU1)<700 && analogRead(pinU3)<700){
      avancer(30);
      tcs.getRawData(&rouge, &vert, &bleu, &clear);
      if(checkJaune())
      {
        resetMenu();
        return;
      }if((rouge > 125 && vert < 125 && bleu < 125) || (rouge > 100 && vert > 150 && bleu < 160))
      { 
        tournerDroite(); 
        resetMenu();
        return;
      }
    }
    arret(50);
 }
 //Fonction qui avance jusqua une ligne ou pendant un certain laps de temps
 //Retourne vrai si le temps s'est écoulé
 bool avancerOuTrouverLigne(int temps,  float facteurVitesse = 0.2f){
  vitesse0 = facteurVitesse;
  vitesse1 = facteurVitesse;
  long now = millis();
    while(analogRead(pinU2)<700&&millis()-now<temps)
    {
      resetPID();
      MOTOR_SetSpeed(RIGHT, vitesse1+calculPID(1));
      MOTOR_SetSpeed(LEFT, vitesse0+calculPID(0));
      if(checkJaune())
      {
        resetMenu();
        return false;
      }
      tcs.getRawData(&rouge, &vert, &bleu, &clear);

      bool isChien = (rouge > 125 && vert < 125 && bleu < 125);
      bool isChat  = (rouge > 100 && vert > 150 && bleu < 160);

      if (isChien || isChat) {
        return true;  
      }
    }
    //Serial.println(analogRead(pinU2));
    return(analogRead(pinU2)<700);
 }
 // Retrouver la ligne par balayage gauche/droite, utile après une étape
 
 //Fonction suiveur de ligne
void SuiveurDeLigne() {
  
  if (checkJaune()) {
    resetMenu();
    return;
  }

  // Seuil et vitesse de base 
  const int SEUIL = 700;
  float vitesseSuiveur = 0.2f;

  int U1 = analogRead(pinU1); // gauche
  int U2 = analogRead(pinU2); // centre
  int U3 = analogRead(pinU3); // droite

  // --- Cas 1 : ligne bien centrée ---
  if (U1 < SEUIL && U2 > SEUIL && U3 < SEUIL) {
    vitesse0 = vitesseSuiveur;
    vitesse1 = vitesseSuiveur;
    lastDirection = 0;
    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }
  // --- Cas 3 : ligne très à gauche ---
  else if (U1 > SEUIL && U2 < SEUIL && U3 < SEUIL) {
    vitesse0 = 0;                 
    vitesse1 = vitesseSuiveur;   
    lastDirection = -1;     
    nbCorrectionGauche++;         
    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }

  // --- Cas 4 : ligne un peu à gauche (U1 et U2) ---
  else if (U1 > SEUIL && U2 > SEUIL && U3 < SEUIL) {
    vitesse0 = vitesseSuiveur / 2; 
    vitesse1 = vitesseSuiveur;     
    lastDirection = -1;
    nbCorrectionGauche++;   
    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }

  // --- Cas 5 : ligne très à droite ---
  else if (U3 > SEUIL && U2 < SEUIL && U1 < SEUIL) {
    vitesse0 = vitesseSuiveur;
    vitesse1 = 0.0f;            
    lastDirection = 1;
    nbCorrectionDroite++;           
    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }

  // --- Cas 6 : ligne un peu à droite (U3 et U2) ---
  else if (U3 > SEUIL && U2 > SEUIL && U1 < SEUIL) { 
    vitesse0 = vitesseSuiveur;        
    vitesse1 = vitesseSuiveur / 2;    
    lastDirection = 1;
    nbCorrectionDroite++; 
    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }
  else if (U3 > SEUIL && U2 > SEUIL && U1 > SEUIL) { 
    vitesse0 = vitesseSuiveur;        
    vitesse1 = 0.0f;    
    //lastDirection = 1;
    //nbCorrectionDroite++; 
    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }
  // --- Cas : plus rien (les 3 capteurs dans le vide) ---
  else if (U1 < SEUIL && U2 < SEUIL && U3 < SEUIL) {
    tcs.getRawData(&rouge, &vert, &bleu, &clear);

    bool isChien = (rouge > 125 && vert < 125 && bleu < 125);
    bool isChat  = (rouge > 100 && vert > 150 && bleu < 160);

    if(isChien || isChat){

      vitesse0 = vitesseSuiveur;
      vitesse1 = vitesseSuiveur;

    }
    else{
      // Là on utilise la dernière direction connue
      if (lastDirection == -1) {
      // La ligne était à gauche → on tourne vers la gauche
      vitesse0 = 0;               
      vitesse1 = vitesseSuiveur;  
      } else if (lastDirection == 1) {
      // La ligne était à droite → on tourne vers la droite
      vitesse0 = vitesseSuiveur;  
      vitesse1 = 0;               
      } else {
      vitesse0 = vitesseSuiveur;
      vitesse1 = vitesseSuiveur;
      }
    }

    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }

  // --- Cas 7 : Rien
  else {
    vitesse0 = vitesseSuiveur;
    vitesse1 = vitesseSuiveur;
    MOTOR_SetSpeed(RIGHT, vitesse1);
    MOTOR_SetSpeed(LEFT,  vitesse0);
  }
}

//Fonction pour permettre de connaitre l'orientation des réservoirs
void OrientationReservoirs() {

    if (nbCorrectionDroite > nbCorrectionGauche) {
      reservoirChien = COTE_INT;
      reservoirChat = COTE_EXT;  
    } 
    else if (nbCorrectionGauche > nbCorrectionDroite) {
      reservoirChien = COTE_EXT;
      reservoirChat = COTE_INT;
    }

    // reset pour recommencer à accumuler
    nbCorrectionGauche = 0;
    nbCorrectionDroite = 0;
}

//Calcule le nombre de pulses attendus selon la vitesse
int calculPulse(float vitesse)
{
  return vitesse * pulseMaxParSec / iterationsParSeconde;
}
 
//Calcule l'erreur entre le nombre de pulses souhaités des moteurs et celui reçu des encodeurs
float calculPID(int indiceEncodeur)
{
  //Moteur de gauche
  if (indiceEncodeur == 0)
  {
    int pulsesReels = ENCODER_Read(0)-pulsesPrecedents0;
    pulsesPrecedents0=pulsesReels;
    int erreur = pulsesReels - calculPulse(vitesse0);
    float P = erreur*kp0;
    integralesErreur0 += erreur*deltaT;
    float I = integralesErreur0*ki0;
    float D = ((erreur - erreurPrecedente0)/deltaT)*kd0;
    erreurPrecedente0 = erreur;
    return P+I+D;
  }
  //Moteur de droite
  else
  {
    int pulsesReels = ENCODER_Read(1)-pulsesPrecedents1;
    pulsesPrecedents1=pulsesReels;
    int erreur = pulsesReels - calculPulse(vitesse1);
    float P = erreur*kp1;
    integralesErreur1 += erreur*deltaT;
    float I = integralesErreur1*ki1;
    float D = ((erreur - erreurPrecedente0)/deltaT)*kd1;
    erreurPrecedente1 = erreur;
    return P+I+D;
  }
}
 
 
void resetPID(){
    ENCODER_Reset(0);
  ENCODER_Reset(1);
  pulsesPrecedents0=0;
  pulsesPrecedents1=0;
}
 
//Fait avancer le robot en ligne droite
void avancer(int temps)
{
resetPID();
  //Changer l'état du robot à avance
  etat=1;
  float vitesseTemp;
  vitesse0 = vitesse;
  vitesse1 = vitesse;
  long now = millis();
  long timer = millis();
  //Corrige la vitesse des moteurs à une intervalle de deltaT, pour les deux moteurs
  while(millis()-now<temps){
 
    if(checkJaune())
    {
      resetMenu();
      return;
    }
    if(millis()-timer>deltaT)
    {
      timer = millis();
      //Corrige la vitesse du moteur de droite
      vitesseTemp = vitesse1 + calculPID(1);
      MOTOR_SetSpeed(RIGHT, vitesseTemp);
     
      //Corrige la vitesse du moteur de gauche
      vitesseTemp = vitesse0 + calculPID(0);
      MOTOR_SetSpeed(LEFT, (vitesseTemp));
 
    }
   
 
  }
}
 
 
 
void resetAll(){
ENCODER_Reset(0);
  ENCODER_Reset(1);
  timer = 0;
  start = false;
  nbCorrectionDroite=0;
  nbCorrectionGauche=0;
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
}
 
void resetMenu()
{
  menuActuel=0;
  lcd.clear();
  lcd.home();
  nbCorrectionDroite=0;
  nbCorrectionGauche=0;
  afficher_menu_principal();
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
}
 
void modeManuel(int mode)
{
  bool pasEncoreNourri = true;
 
  avancerjusqualigne();
  //avancer(200);
  //tournerJusquaLigne(1);
  
  while(pasEncoreNourri){
    if(checkJaune())
    {
      resetMenu();
      return;
    }
    tcs.getRawData(&rouge, &vert, &bleu, &clear);
    if(rouge>125&&vert<125&&bleu<125&&mode==CHIEN)
    {
      avancer(400);
      arret(200);
       OrientationReservoirs();
      if(reservoirChien==COTE_INT){
        tournerDroite();
        tournerDroite();
      }
      int count = 0;
      long now = millis();
      SERVO_Enable(1);
      delay(1500); 
      while(count<nbTourServo){
        if(millis()-now>1500){
          now = millis();
          SERVO_SetAngle(1, 180*(count%2));  
          count++;
        }
        if(checkJaune())
        {
          resetMenu();
          return;
        }
      }
      delay(3000);
      tournerGaucheDance(90, 1.5f);
      pasEncoreNourri=false;
    }
    else if(rouge>100&&vert>150&&bleu<160&&mode==CHAT)
    {
      avancer(400);
      arret(200);
      OrientationReservoirs();
      if(reservoirChat==COTE_INT){
        tournerDroite();
        tournerDroite();
      }

      int count = 0;
      long now = millis();
      SERVO_Enable(0);
      delay(1500);
      while(count<nbTourServo){
        if(millis()-now>1500){
          now = millis();
          SERVO_SetAngle(0, 180*(count%2)); 
          count++;
        }
        if(checkJaune())
        {
          resetMenu();
          return;
        }
      }
      delay(3000);
      tournerDroite();
      pasEncoreNourri=false;
    }
    SuiveurDeLigne();
  }
  avancer(1500);
  arret(3000);

  resetAll();
  resetMenu();
}
 
void deplacement()
{
    if(avancerOuTrouverLigne(2500))
    {
      if(checkJaune())
      {
        resetMenu();
        return;
      }
      int angle = rand();
      if(angle%2==0)
      tournerDroiteDance(angle%100,1);
      else
      tournerGaucheDance(angle%100,1);
    }else
    {
      if(checkJaune())
      {
        resetMenu();
        return;
      }
      tournerDroite();
      tournerDroite();
      avancer(300);
    }
}
 
void jouer()
{
    if(avancerOuTrouverLigne(2000, 0.3f))
    {
      if(checkJaune())
      {
        resetMenu();
        return;
      }
      int angle = rand();
      if(angle%2==0)
      tournerDroiteDance(angle%90+90,1.5);
      else
      tournerGaucheDance((angle%90+90),1.5);
    }else
    {
      if(checkJaune())
      {
        resetMenu();
        return;
      }
      tournerDroite();
      tournerDroite();
      avancer(300);
    }
}

bool cibleChien = false;
bool cibleChat  = false;

void modeAuto() {

  const float VITESSE_AVANT  = 0.15f;
  const float VITESSE_TOURNE = 0.0f;
  const uint16_t SEUIL_MUR   = 400;

  while (true) {
    bool cap_g = (digitalRead(capteur_g) == LOW);
    bool cap_d = (digitalRead(capteur_d) == LOW);

    uint16_t cap_distance = ROBUS_ReadIR(0);

    bool murProche = (cap_distance > SEUIL_MUR);

    if (checkJaune()) {
      resetMenu();
      resetAll();
      return;
    }

    if(murProche){
      MOTOR_SetSpeed(LEFT,  0.0f);
      MOTOR_SetSpeed(RIGHT, 0.0f);

      delay(200);

      bool isChien = false;
      bool isChat  = false;

      while(!isChien && !isChat){
        if (checkJaune()) {
        resetMenu();
        resetAll();
        return;
        }
        tcs.getRawData(&rouge, &vert, &bleu, &clear);

        isChien = (rouge > 100 && vert < 125 && bleu < 125);
        isChat  = (rouge > 100 && vert > 150 && bleu < 160);

        delay(100);
      }

      if(isChien){
      tournerGaucheDance(90, 1.5f);
      int count = 0;
      long now = millis();
      SERVO_Enable(1);
      delay(500);
      while(count<nbTourServo){
        if(millis()-now>1500){
          now = millis();
          SERVO_SetAngle(1, 180*(count%2)); 
          count++;
        }
        if(checkJaune())
        {
          resetMenu();
          return;
        }
      }

      }else if(isChat){

      tournerDroite();
      int count = 0;
      long now = millis();
      SERVO_Enable(0);
      delay(500);
      while(count<nbTourServo){
        if(millis()-now>1500){
          now = millis();
          SERVO_SetAngle(0, 180*(count%2)); 
          count++;
        }
        if(checkJaune())
        {
          resetMenu();
          return;
        }
      }
      }else{
      resetMenu();
      resetAll();
      return; 
      }

      delay(500);
      if(isChien){
        //tournerGauche();
        tournerGaucheDance(90, 1.5f);
      }else if (isChat){
        tournerDroite();
      }else{
      resetMenu();
      resetAll();
      return; 
      }

    }else{
        if (cap_g && cap_d) {
        MOTOR_SetSpeed(LEFT,  VITESSE_AVANT);
        MOTOR_SetSpeed(RIGHT, VITESSE_AVANT);
      }
      else if (cap_g && !cap_d) {
        MOTOR_SetSpeed(LEFT,  VITESSE_TOURNE);
        MOTOR_SetSpeed(RIGHT, VITESSE_AVANT);
      }
      else if (!cap_g && cap_d) {
        MOTOR_SetSpeed(LEFT,  VITESSE_AVANT);
        MOTOR_SetSpeed(RIGHT, VITESSE_TOURNE);
      }
      else {
        deplacement();
      }
    }
    delay(100);
  }
}

 
bool checkJaune()
{
  return (digitalRead(bp_j)==LOW);
}
void afficher_menu_principal(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CHOIX:MODE");
 
  lcd.setCursor(0,2);
  lcd.print("VERT:AUTO");
 
 
  lcd.setCursor(0,3);
  lcd.print("ROUGE:MANUEL");
}
 
void sous_menu_manuel(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MODE:MANUEL");
 
  lcd.setCursor(0,2);
  lcd.print("VERT:JOUER");
 
 
  lcd.setCursor(0,3);
  lcd.print("ROUGE:NOURRIR");
}
 
void sous_menu_nourrir(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MENU:NOURRIR");
 
  lcd.setCursor(0,2);
  lcd.print("VERT:CHAT");
 
  lcd.setCursor(0,3);
  lcd.print("ROUGE:CHIEN");
}

void menu_mode_auto(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MODE AUTO");
 
  lcd.setCursor(0,2);
  lcd.print("JAUNE:ARRET");
}

void menu_mode_jouer(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MODE JOUER");
 
  lcd.setCursor(0,2);
  lcd.print("JAUNE:ARRET");
}

void menu_mode_chien(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MODE CHIEN");
 
  lcd.setCursor(0,2);
  lcd.print("JAUNE:ARRET");
}

void menu_mode_chat(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MODE CHAT");
 
  lcd.setCursor(0,2);
  lcd.print("JAUNE:ARRET");
}
 
void setup() {
  BoardInit();
 
  Serial.begin(9600);
  SERVO_Disable(0);
  SERVO_Disable(1); 
  pinMode(pin5kHz, INPUT);
  pinMode(pinAmbiant, INPUT);
  pinMode(pinU1, INPUT);
  pinMode(pinU2, INPUT);
  pinMode(pinU3, INPUT);
  pinMode(bp_r, INPUT_PULLUP); //Rouge
  pinMode(bp_v, INPUT_PULLUP); //Vert
  pinMode(bp_j, INPUT_PULLUP); //Jaune
  pinMode(capteur_g, INPUT); //IR gauche
  pinMode(capteur_d, INPUT); //IR droite
 // attachInterrupt(digitalPinToInterrupt(2), resetAll, CHANGE);
 
  lcd.begin(16, 4);
  lcd.clear();
  afficher_menu_principal();
 
}
  
/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {

 /*
Serial.println("Distance IR:");
Serial.println(ROBUS_ReadIR(0));
delay(1000);
*/
/*
  Serial.println("CAPTEUR G:");
  Serial.println( digitalRead(capteur_g));
  //Serial.println( ROBUS_ReadIR(capteur_g));
  Serial.println("CAPTEUR D:");
  Serial.println( digitalRead(capteur_d));
  //Serial.println( ROBUS_ReadIR(capteur_d));
  Serial.println("--------------");
  delay(1000);
  */

  /*
  Serial.println("TROIS CAPTEURS:");
  Serial.println(analogRead(pinU1));
  Serial.println(analogRead(pinU2));
  Serial.println(analogRead(pinU3));
 
  Serial.println("------------------------");
  delay(1000);
  */

  /*
  uint16_t rouge, vert, bleu, clear;
  tcs.getRawData(&rouge, &vert, &bleu, &clear);
 
  Serial.println(rouge);
  Serial.println(vert);
  Serial.println(bleu);
  Serial.println(clear);
  Serial.println("-------------------------------------------------");

  delay(500);

  if(rouge>125&&vert<125&&bleu<125&&clear>300){
    beep(1);
  }else if(rouge>100&&vert>150&&bleu<150&&clear>300){
    beep(2);
  }

  delay(300);
  */

  
  SERVO_Disable(0);
  SERVO_Disable(1);

  bool is_bp_r = (digitalRead(bp_r) == LOW);
  bool is_bp_v = (digitalRead(bp_v) == LOW);
  bool is_bp_j = (digitalRead(bp_j) == LOW);

  //Assurer un click a la fois, lorsquon ce deplace dans le menu
  bool click_btn_r = is_bp_r && !last_bp_r;
  bool click_btn_v = is_bp_v && !last_bp_v;

  last_bp_r = is_bp_r;
  last_bp_v = is_bp_v;

  if (is_bp_j) {
    resetAll();
    menuActuel = 0;
    afficher_menu_principal();
    MOTOR_SetSpeed(RIGHT, 0);
    MOTOR_SetSpeed(LEFT, 0);
    return;
  }

  switch (menuActuel) {

    case 0: // Menu principal
      if (click_btn_v) {
        menu_mode_auto();                
        modeAuto();
        
      } else if (click_btn_r) {             
        menuActuel = 1;
        sous_menu_manuel();
      }
      break;

    case 1: // Menu manuel
      if (click_btn_v) {
        menu_mode_jouer();
        while (digitalRead(bp_j) != LOW) {
          jouer();
        }
      } else if (click_btn_r) {
        menuActuel = 2;
        sous_menu_nourrir();
      }
      break;

    case 2: // Menu nourrir
      if (click_btn_v) {
        menu_mode_chat();
        modeManuel(CHAT);
      } else if (click_btn_r) {
        menu_mode_chien();
        modeManuel(CHIEN);
      }
      break;

    default:
      resetMenu();
      break;
  }

  delay(100); 
 
}



 
 