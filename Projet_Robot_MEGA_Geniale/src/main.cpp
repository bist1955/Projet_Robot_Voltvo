 
/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/
 
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
 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
 
/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces
 
bool bumperArr;
int vertpin = 48;
int rougepin = 49;
int pinDELJaune = 39;
int pinDELRouge = 41;
int pinDELBleue = 43;
int pinDELVerte = 45;
int pinU1 = A6;
int pinU2 = A5;
int pinU3 = A7;
int pin5kHz = A10;
int pinAmbiant = A11;
bool vert = false;
bool rouge = false;
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
 
int dance_fait = false;
 
/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
void avancer(int);
void reculer(int);
float calculPID(int);
void resetPID();
void recule(){
  MOTOR_SetSpeed(RIGHT,-vitesse);
  MOTOR_SetSpeed(LEFT, -vitesse);
  etat = 2;
}
void arret(int temps){
 
  switch (etat)
  {
  case 1:
  for(int i=20;i!=0;i--){
    MOTOR_SetSpeed(RIGHT,vitesse*i/20);
    MOTOR_SetSpeed(LEFT, vitesse*i/20);
  //Serial.print(-vitesse*i/20);
    delay(temps/20);
  }
    break;
  case 2:
  for(int i=20;i!=0;i--){
  MOTOR_SetSpeed(RIGHT,-vitesse*i/20);
  MOTOR_SetSpeed(LEFT, -vitesse*i/20);
  //Serial.print(-vitesse*i/20);
  delay(temps/20);
}
  break;
  case 3:
  for(int i=20;i!=0;i--){
  MOTOR_SetSpeed(RIGHT,-vitesse*i/20);
  MOTOR_SetSpeed(LEFT, vitesse*i/20);
  //Serial.print(-vitesse*i/20);
  delay(temps/20);
}
case 4:
for(int i=20;i!=0;i--){
  MOTOR_SetSpeed(RIGHT,vitesse*i/20);
  MOTOR_SetSpeed(LEFT, -vitesse*i/20);
  //Serial.print(-vitesse*i/20);
  delay(temps/20);
}
break;
  break;
  default:
  MOTOR_SetSpeed(RIGHT,0);
  MOTOR_SetSpeed(LEFT, 0);
    break;
  }
  MOTOR_SetSpeed(RIGHT,0);
  MOTOR_SetSpeed(LEFT, 0);
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
 
  while (pulsesMax>ENCODER_Read(0)&&pulsesMax>-ENCODER_Read(1))
  {
    MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
    MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    delay(deltaT);
  }
 arret(80);
}
 void tournerJusquaLigne(int sens, int angle, void (*f1)()){
   int pulsesMax = (angle/90)*1900;
   int nbPulses0 = 0;
   int nbPulses1 = 0;
   if(sens==1){
     etat=3;
     vitesse0=0.4f;
     vitesse1=-0.4f;
     while (pulsesMax*0.9>nbPulses0&&pulsesMax*0.9>-nbPulses1){
       f1();
       //Mettre à l'état tourne à droite
       nbPulses0+=ENCODER_Read(0);
       nbPulses1+=ENCODER_Read(0);
       resetPID();
       MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
       MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
       delay(deltaT);
      }
   while(analogRead(pinU2)<600){
    resetPID();
    f1();
     MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
     MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
     delay(10);
   }
   arret(50);
 
 }else{
  etat=4;
  vitesse0=-vitesse;
  vitesse1=vitesse;
 
  while (pulsesMax*0.9>-nbPulses0&&pulsesMax*0.9>nbPulses1)
  {
    f1();
       //Mettre à l'état tourne à gauche
       nbPulses0+=ENCODER_Read(0);
       nbPulses1+=ENCODER_Read(0);
    resetPID();
    MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
    MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    delay(deltaT);
  }
 }
    while(analogRead(pinU2)<600){
      resetPID();
      f1();
     MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
     MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
     delay(10);
   }
   arret(50);
 
 }
 
//Tourne le robot à gauche à l'arrêt
void tournerGauche(){
int pulsesMax =1650;
  //Mettre à l'état tourne à droite
  etat=4;
  resetPID();
  vitesse0=-vitesse;
  vitesse1=vitesse;
 
  while (pulsesMax>-ENCODER_Read(0)&&pulsesMax>ENCODER_Read(1))
  {
    MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
    MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    delay(deltaT);
  }
 arret(80);
}
 void tournerGaucheDance(int nbPulses){
int pulsesMax =nbPulses;
  //Mettre à l'état tourne à droite
  etat=4;
  resetPID();
  vitesse0=-vitesse;
  vitesse1=vitesse;
 
  while (pulsesMax>-ENCODER_Read(0)&&pulsesMax>ENCODER_Read(1))
  {
    MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
    MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    delay(deltaT);
  }
 arret(80);
}
//Tourne le robot à gauche à l'arrêt
void tournerGauche45(){
int pulsesMax =762.5;
  //Mettre à l'état tourne à droite
  etat=4;
  resetPID();
  vitesse0=-vitesse;
  vitesse1=vitesse;
 
  while (pulsesMax>-ENCODER_Read(0)&&pulsesMax>ENCODER_Read(1))
  {
    MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
    MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    delay(deltaT);
  }
 arret(80);
}
 
//Tourne le robot à droite à l'arrêt
void tournerDroite45(){
int pulsesMax =925;
  //Mettre à l'état tourne à droite
  etat=3;
  resetPID();
  vitesse0=vitesse;
  vitesse1=-vitesse;
 
  while (pulsesMax>ENCODER_Read(0)&&pulsesMax>-ENCODER_Read(1))
  {
    MOTOR_SetSpeed(RIGHT,(vitesse1 + calculPID(1)));
    MOTOR_SetSpeed(LEFT, (vitesse0 + calculPID(0)));
    delay(deltaT);
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
void avancerUnBloc(){
  avancer(1800);
  arret(500);
  delay(50);
}
void reculerUnBloc(){
  reculer(1800);
  arret(500);
  delay(50);
}
void avancerEtapeRouge50(){
  avancer(1500);
  arret(500);
  delay(50);
}
 
void avancerEtapeRouge25(){
  avancer(1250);
  arret(500);
  delay(50);
}
 
void avancerEtapeRouge10(){
  avancer(700);
  arret(500);
  delay(50);
}
 
void avancerEtapeJaune(){
  avancer(1800);
  arret(500);
  delay(50);
}
 
//Avance et arrete a une ligne au sol
void avancerjusqualigne(){
 
    while(analogRead(pinU2)<700){
      avancer(30);
    }
    arret(50);
 }
 //Fonction qui avance jusqua une ligne ou pendant un certain laps de temps
 //Retourne vrai si le temps s'est écoulé
 bool avancerOuTrouverLigne(int temps){
  long now = millis();
    while(analogRead(pinU2)<700&&millis()-now<temps)
    {
      avancer(30);
    }
    return(analogRead(pinU2)<700);
 }
 // Retrouver la ligne par balayage gauche/droite, utile après une étape
void rechercherLigne()  
{
  int seuil = 700; // à ajuster selon le capteur et l'éclairage mais cetait 1000 pour une ligne noir
  float vitessePivot = 0.15f;
 
  int val = analogRead(pinU2);
 
  if (val > seuil)
  {
    return;
  }
  etat = 4;
  // Balayage vers la GAUCHE
  for (int i = 0; i < 15; ++i)
  {
    // pivot gauche (RIGHT avant, LEFT arrière)
    MOTOR_SetSpeed(RIGHT,  vitessePivot);
    MOTOR_SetSpeed(LEFT,  -vitessePivot);
    delay(100);
    arret(100);
 
    int val = analogRead(pinU2);
 
    if (val > seuil)
    {
      return;
    }
  }
 
  // Balayage vers la DROITE
  etat = 3;
  for (int i = 0; i < 30; ++i) {
    // pivot droite (RIGHT arrière, LEFT avant)
    MOTOR_SetSpeed(RIGHT, -vitessePivot);
    MOTOR_SetSpeed(LEFT,   vitessePivot);
    delay(100);
    arret(100);
 
    int val = analogRead(pinU2);
   
    if (val > seuil)
    {
      return;
    }
  }
 
  // Balayage vers la GAUCHE
  for (int i = 0; i < 30; ++i)
  {
    // pivot gauche (RIGHT avant, LEFT arrière)
    MOTOR_SetSpeed(RIGHT,  vitessePivot);
    MOTOR_SetSpeed(LEFT,  -vitessePivot);
    delay(100);
    arret(100);
 
    int val = analogRead(pinU2);
 
    if (val > seuil)
    {
      return;
    }
  }
 
  // Balayage vers la DROITE
  etat = 3;
  for (int i = 0; i < 15; ++i) {
    // pivot droite (RIGHT arrière, LEFT avant)
    MOTOR_SetSpeed(RIGHT, -vitessePivot);
    MOTOR_SetSpeed(LEFT,   vitessePivot);
    delay(100);
    arret(100);
 
    int val = analogRead(pinU2);
   
    if (val > seuil)
    {
      return;
    }
  }
 
 
}
 
 
 //Fonction qui avance en suivant (duh)
 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 //IL FAUT CHANGER LA VALEUR DE SEUIL
 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 void SuiveurDeLigne(){
  int U1 = analogRead(pinU1);
  int U2 = analogRead(pinU2);
  int U3 = analogRead(pinU3);
  float vitesseSuiveur = 0.25f;
  if(U1<700&&U2>700&&U3<700)
  {
    //cas normal, la ligne est au milieu
    //Serial.println("Centré");
    //on met les vitesses desirees
    //on fait avancer le robot tout droit
    vitesse0=vitesseSuiveur;
    vitesse1=vitesseSuiveur;
    MOTOR_SetSpeed(RIGHT,vitesse1);
    MOTOR_SetSpeed(LEFT,vitesse0);
  }else if(U3>700)
  {
    //cas ou la ligne est trop a gauche, donc il faut aller a gauche
    //Serial.println("trop a gauche");
    //on fait avancer le robot vers la gauche
    vitesse0=vitesseSuiveur*1.5;
    vitesse1=vitesseSuiveur*vitesseSuiveur*vitesseSuiveur;
    MOTOR_SetSpeed(RIGHT,vitesse1);
    MOTOR_SetSpeed(LEFT,vitesse0);
  }else if(U1>500)
  {
    //cas ou la ligne est trop a droite, donc il faut aller a droite
    //Serial.println("trop a droite");
    //on fait avancer le robot vers la droite
    vitesse1=vitesseSuiveur*1.5;
    vitesse0=vitesseSuiveur*vitesseSuiveur*vitesseSuiveur;
    MOTOR_SetSpeed(RIGHT,vitesse1);
    MOTOR_SetSpeed(LEFT,vitesse0);
  }else
  {
 
  }
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
  //Corrige la vitesse des moteurs à une intervalle de deltaT, pour les deux moteurs
  for(int i = 0; i<temps/deltaT;i++){
    //Corrige la vitesse du moteur de droite
    vitesseTemp = vitesse1 + calculPID(1);
    MOTOR_SetSpeed(RIGHT, vitesseTemp);
   
    //Corrige la vitesse du moteur de gauche
    vitesseTemp = vitesse0 + calculPID(0);
    MOTOR_SetSpeed(LEFT, (vitesseTemp));
   
    //Délai de deltaT pour corriger les moteurs une nouvelle fois après ce délai
    delay(deltaT);
  }
}
void avancerVersLaDroite(int temps){
  resetPID();
  //Changer l'état du robot à avance
  etat=1;
  float vitesseTemp;
  vitesse0 = vitesse;
  vitesse1 = vitesse*vitesse;
  //Corrige la vitesse des moteurs à une intervalle de deltaT, pour les deux moteurs
  for(int i = 0; i<temps/deltaT;i++){
    //Corrige la vitesse du moteur de droite
    vitesseTemp = vitesse1 + calculPID(1);
    MOTOR_SetSpeed(RIGHT, vitesseTemp);
   
    //Corrige la vitesse du moteur de gauche
    vitesseTemp = vitesse0 + calculPID(0);
    MOTOR_SetSpeed(LEFT, (vitesseTemp));
   
    //Délai de deltaT pour corriger les moteurs une nouvelle fois après ce délai
    delay(deltaT);
  }
}
void reculer(int temps)
{
resetPID();
  //Changer l'état du robot à avance
  etat=2;
  float vitesseTemp;
  vitesse0 = -vitesse;
  vitesse1 = -vitesse;
  //Corrige la vitesse des moteurs à une intervalle de deltaT, pour les deux moteurs
  for(int i = 0; i<temps/deltaT;i++){
    //Corrige la vitesse du moteur de droite
    vitesseTemp = vitesse1 + calculPID(1);
    MOTOR_SetSpeed(RIGHT, vitesseTemp);
   
    //Corrige la vitesse du moteur de gauche
    vitesseTemp = vitesse0 + calculPID(0);
    MOTOR_SetSpeed(LEFT, (vitesseTemp));
   
    //Délai de deltaT pour corriger les moteurs une nouvelle fois après ce délai
    delay(deltaT);
  }
}
 
 
void resetAll(){
ENCODER_Reset(0);
  ENCODER_Reset(1);
  timer = 0;
  modeDepart = false;
  sonDetecte = false;
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
}
 

 

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programmes
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales
 
void setup(){
  BoardInit();
 
  //SERVO_Disable(0);
  //SERVO_Disable(1);
  //SERVO_SetAngle(0,180);
  //SERVO_SetAngle(1,180);
 
  //Initialisation du capteur de couleur
  Serial.begin(9600);
  Serial.println("Initialisation du capteur de couleur");
 /*
  if (tcs.begin()) {
    Serial.println("Capteur détecté");
  } else {
    Serial.println("Erreur: capteur non détecté.");
    while (1);
  }
 */
  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  pinMode(pin5kHz, INPUT);
  pinMode(pinAmbiant, INPUT);
  pinMode(pinU1, INPUT);
  pinMode(pinU2, INPUT);
  pinMode(pinU3, INPUT);
  delay(100);
  beep(1);
 
    //Déclaration des pins pour les leds(bleu, rouge, vert et jaune)
  pinMode(pinDELJaune, OUTPUT);
  pinMode(pinDELRouge, OUTPUT);
  pinMode(pinDELBleue, OUTPUT);
  pinMode(pinDELVerte, OUTPUT);

}
 
 
/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"
 
void loop() {
 
 
  //Valeur de couleurs reçu par le capteur de couleur
  uint16_t rouge, vert, bleu, clear;
  //Tension reçu du suiveur de ligne
 
 
  // Serial.println("TROIS CAPTEURS:");
  // Serial.println(analogRead(pinU1));
  // Serial.println(analogRead(pinU2));
  // Serial.println(analogRead(pinU3));
 
  // Serial.println("------------------------");
 
 
  // //Lire les valeurs de couleur du capteur de couleur
   tcs.getRawData(&rouge, &vert, &bleu, &clear);
 
  Serial.println(rouge);
  Serial.println(vert);
  Serial.println(bleu);
  Serial.println(clear);
  Serial.println("-------------------------------------------------");
 
 
  float son5kHz = analogRead(pin5kHz);
  float diff = analogRead(pinAmbiant)-son5kHz;
 /*
  if(diff<100 && son5kHz>500){
    sonDetecte=true;
  }
 

  if(ROBUS_IsBumper(3)){
    sonDetecte=true;
  }
 
 //Serial.println(analogRead(A0));
 if(sonDetecte){
 
    if((rouge<200 && rouge>170) && (vert<185 && vert>150) && (bleu<200 && bleu>170))
    {
      AllumerLEDRouge();
      EtapeRouge();
      SuiveurDeLigne();
      Serial.println("Carton rose");
    }else if ((rouge<170 && rouge>130) && (vert<240 && vert>190) && (bleu<200 && bleu>170))
      {
      AllumerLEDVerte();
      EtapeVerte();
      SuiveurDeLigne();
        Serial.println("Carton vert");
      }else if((rouge<150 && rouge>120) && (vert<220 && vert>170) && (bleu<240 && bleu>175))
        {
          AllumerLEDBleu();
          EtapeBleue();
          SuiveurDeLigne();
          Serial.println("Carton bleu");
        }else if ((rouge<230 && rouge>200) && (vert<250 && vert>200) && (bleu<190 && bleu>155))
          {
            AllumerLEDJaune();
            EtapeJaune();  
            SuiveurDeLigne();
            Serial.println("Carton jaune");
          }else
          {
          EteindreLesLEDs();
          SuiveurDeLigne();    
          Serial.println("Autre carton");
          }
     delay(10);
    }
   */

 }
 
 
 
 
 