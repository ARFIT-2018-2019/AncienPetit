ssedfsezR///////////////////////////////////////////////////////////////////////////////////////////////
//  Programme Asservissement de la trajectoire et des mouvements du petit robot ARFIT
//  Moteur DC Pololu 12V MP 20,4:1 avec encodeur, 48
//  Encodeur incrémental, Kubler, 12000rpm, Incrémental, 1024, Push-pull, 5 â†’ 24 
//  Arduino DUE avec Dual MC33926 Motor Driver Shield 
//  GONIN Nathan 05/2018
///////////////////////////////////////////////////////////////////////////////////////////////

//Bibliothèque qui gère le timer des asservissements
# include <DueTimer.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

//Bibliothèque de l'application de commande des moteurs (Création d'une PWM)
# include "Applique_Commande.h"

// Définition des intructions de précompilation
# define BAUDRATE 115200     // 115200 bauds
# define TEDATA 20           // Période transmission des données (ms)

// Déclarations pour les moteurs gauche et droit.
# define PWM_MG 6
# define MG_IN1 7
# define MG_IN2 8
# define PWM_MD 9
# define MD_IN1 11
# define MD_IN2 10

// Déclarations des pins pour les encodeurs des moteurs
# define Enc_RMG_VA 22
# define Enc_RMG_VB 23
# define Enc_RMD_VA 26
# define Enc_RMD_VB 27

// Déclarations des pins pour les encodeurs des roues codeuses
# define Enc_RCG_VA 24
# define Enc_RCG_VB 25
# define Enc_RCD_VA 28
# define Enc_RCD_VB 29

// Pour le calcul de la vitesse via moyenne mobile de longueur NmoyenneMobile
# define NmoyenneMobile 10

#define pin1_canon 2
#define pin2_canon 3
#define gnd_canon 4

#define tirette 49
#define interCouleur 47

#define captAvDroit A3
#define captArDroit A4
#define captAvGauche A6
#define captArGauche A5

boolean etatTirette = false;
boolean etatInterCouleur = false;

//Définition des compteurs d'impulsion des encodeurs moteurs et des roues encodeuses
volatile long CountEncodeurMG = 0;
volatile long CountEncodeurMD = 0;
volatile long CountEncodeurCG = 0;
volatile long CountEncodeurCD = 0;
volatile long CountEncodeurCGodo = 0;
volatile long CountEncodeurCDodo = 0;

//Paramètre physique du robot en cm
volatile const float DRoueEnc = 6.2200;
volatile const float DRoueMot = 7.3024;
volatile float EntraxeRoueEnc = 6.4000;
volatile const float EntraxeRoueMot = 12.4000;

// Gestion du temps pour l'envoi des données.
unsigned long TempsCourant        = 0;
unsigned long TempsDernierEnvoi   = 0;

volatile int indiceTicksCodeurMG = 0;
volatile int indiceTicksCodeurMD = 0;
volatile int TabCountCodeurMG[NmoyenneMobile];
volatile int TabCountCodeurMD[NmoyenneMobile];
volatile int DeltaPositionCodeurMG;
volatile int DeltaPositionCodeurMD;
volatile int TabCountCodeurCD[NmoyenneMobile];
volatile int TabCountCodeurCG[NmoyenneMobile];

// Variables globales pour la mise en place de l'asservissement.
volatile float omegaref    = 0.0;   // la consigne de vitesse (rad/s)
volatile float omegaMG       = 0.0;   // La vitesse de rotation (rad/s) du moteur gauche
volatile float omegaMD       = 0.0;   // La vitesse de rotation (rad/s) du moteur droit

const float TECOMMANDE = 10.0  ;   // ms
volatile float dt = TECOMMANDE / 1000.0; // s

volatile float CommandeMG;
volatile float CommandeMD;

volatile float temps = 0.0;  // temps courant en sec

int DataSize  =  100 ;  // Nombre d'écriture à  effectuer sur le port série dès que la commande U est modifiée.
int ChangeCmd = 0 ;     // flag pour indication changement de commande.
long unsigned  i = 0 ;     // Compteur d'écriture effectuée;
double uMG, u1MG, uMD, u1MD;  // création des nouvelles variables necessaire pour la commande 
double eMG, e1MG, eMD, e1MD;
float V,R,Ktheta;    // Création de la variable consigne 

float VitesseTransRot[2]; //Vitesse en translation (cm.s-1) et en rotation (rad.s-1) du robot
float ConsigneVitesseRotMG;  //Vitesse angulaire de chaque roue du robot (en rad.s-1)
float ConsigneVitesseRotMD;
int NbImpulsionInitCD;
int NbImpulsionInitCG;
int NbImpulsionObj;
int EcartImpulsion;
int TransErreurTheta;

//odometrie
volatile float xt = 48.0000;
volatile float yt = 28.0000;
volatile float thetat = 0.0000;
volatile float xtETdt, ytETdt, thetatETdt, DRD, DRG, DC;
volatile int CountEncodeurCGTempsCourant, CountEncodeurCDTempsCourant;
volatile int CountEncodeurCGTempsPrecedent = 0;
volatile int CountEncodeurCDTempsPrecedent = 0;

////////////////////////TRANSLATIONS////////////////
float erreurTransMD = 0;
float erreurTransMG = 0;

//float GainDroit=0.2;
//float GainGauche=0.2;  //0.13

float GainTransDroit=0.03;
float GainTransGauche=0.03;

float erreurTransOrientation = 0;
float SPpulse = 0;

//avancer
float distanceA = 0;
float dprevA = 0;

float gainOrientationTransDroitA = 0.21;
float gainOrientationTransGaucheA = 0.21;

/*float gainOrientationTransDroitA = 0.3;
float gainOrientationTransGaucheA = 0.3;*/

int changementAvancer = 1;

//reculer
float distanceR = 0;
float dprevR = 0;

/*float gainOrientationTransDroitR = 0.3;
float gainOrientationTransGaucheR = 0.3;*/
//0.21
float gainOrientationTransDroitR = 0.25;
float gainOrientationTransGaucheR = 0.25;

int changementReculer = 1;

////////////////////////////ROTATIONS//////////////////
float SPpulseD = 0;
float SPpulseG = 0;

float angle = 0;
float aprev = 0;

float erreurRotMD = 0;
float erreurRotMG = 0;

//float gainRotDroit = 0.3;
//float gainRotGauche = 0.3;
//0.27
float gainRotDroit = 0.32;
float gainRotGauche = 0.32;

const int Precision = 50; //Précision en nombre d'impulsions sur l'encodeur

int changementRotation = 1;

////////////////////////////Aller A//////////////////////
float consigneOrientation = 0;
float distanceCible = 0;
int signe = 0;

float XCIBLE = 0;
float YCIBLE = 0;

int changementAllerA = 0;
char etatAllerA;
char etatReculerVers;
/////////////////////////////CANON//////////////////////
int vitesse_canon = 65; //moins on met plus il tourne vite
float pi = 3.141592654;
float c = 0;
float s = 0;
float CapObjectif = 0;

////////////////////////////////Strategie///////////////////////////////
//boolean E1 = true;
boolean E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14;
boolean E1prev = true;
boolean finE1 = false;
boolean finE2 = false;
boolean finE3 = false;
boolean finE4 = false;
boolean finE5 = false;
boolean finE6 = false;
boolean finE7 = false;
boolean finE8 = false;
boolean finE9 = false;
boolean finE10 = false;
boolean finE11 = false;
boolean finE12 = false;
boolean finE13 = false;

boolean E2prev=false;
boolean E3prev, E4prev, E5prev, E6prev, E7prev, E8prev, E9prev, E10prev, E11prev, E12prev, E13prev, E14prev;
boolean etat = false;

boolean finAvancer12 = false;
boolean finAvancer23 = false;
boolean finAvancer45 = false;
boolean finAvancer89 = false;
boolean finAvancer1011 = false;
boolean finAvancer1112 = false;

boolean finReculer67 = false;
boolean finReculer1011 = false;
boolean finReculer1112 = false;

boolean finServo34 = false;
boolean finServo56 = false;
boolean finServo78 = false;
boolean finServo910 = false;

boolean finLancement1213 = false;
boolean arret13 = false;

boolean chgtEtape = true;

boolean finLancement = false;
boolean cond12, cond23, cond34, cond45, cond56, cond67, cond78, cond89, cond910, cond1011, cond1112, cond1213, cond1301;
//IHM
String couleur = "";

//temps
unsigned long startMillis;
unsigned long currentMillis3=0;
unsigned long currentMillis4=0;
unsigned long currentMillis5=0;
unsigned long currentMillis7=0;
unsigned long currentMillis9=0;
unsigned long currentMillis11=0;
unsigned long currentMillis12=0;
unsigned long currentMillis13=0;

////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(interCouleur, INPUT_PULLUP);
  pinMode(tirette, INPUT_PULLUP);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();

  // Initialisation de la  com série.
  Serial.begin(BAUDRATE);
  Serial1.begin(9600);
  Serial2.begin(9600);
  
  while(digitalRead(tirette)==0)
  {
    /*Serial.print(analogRead(captAvDroit));
    Serial.print("   ");
    Serial.print(analogRead(captArDroit));
    Serial.print("   ");
    Serial.print(analogRead(captAvGauche));
    Serial.print("   ");
    Serial.print(analogRead(captArGauche));
    Serial.println("   ");*/
    if(digitalRead(interCouleur))
    {
      display.setTextSize(4);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("VERT");
      display.display();
      display.clearDisplay();
      couleur = "VERT";
      xt = 48.0000;
      yt = 28.0000;
    }
    else
    {
      display.setTextSize(3);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("ORANGE");
      display.display();
      display.clearDisplay();
      couleur = "ORANGE";
      xt = 48.0000;
      yt = 273.0000;
    } 
  }
  startMillis = millis();
  
  //Moteurs gauche et droit
  pinMode(PWM_MG, OUTPUT);
  pinMode(MG_IN1, OUTPUT);
  pinMode(MG_IN2, OUTPUT);
  pinMode(PWM_MD, OUTPUT);
  pinMode(MD_IN1, OUTPUT);
  pinMode(MD_IN2, OUTPUT);

  //Capteurs de collision
  pinMode(captAvDroit, INPUT);
  pinMode(captArDroit, INPUT);
  pinMode(captAvGauche, INPUT);
  pinMode(captArGauche, INPUT);

  // Définition d'interruptions sur fronts montants des 2 voies A&B de chaque encodeur (moteurs et roues encodeuses).
  attachInterrupt(Enc_RMG_VA, fmMG_A, CHANGE);
  attachInterrupt(Enc_RMG_VB, fmMG_B, CHANGE);
  attachInterrupt(Enc_RMD_VA, fmMD_A, CHANGE);
  attachInterrupt(Enc_RMD_VB, fmMD_B, CHANGE);
  
  attachInterrupt(Enc_RCG_VA, fmCG_A, RISING);
  attachInterrupt(Enc_RCD_VA, fmCD_A, RISING);

  pinMode(pin1_canon, OUTPUT);
  pinMode(pin2_canon, OUTPUT);
  pinMode(gnd_canon, OUTPUT);
  digitalWrite(pin1_canon, HIGH);
  digitalWrite(pin2_canon, HIGH);
  digitalWrite(gnd_canon, LOW);
  
  // Paramètrage et lancement de la tâche périodique: basé sur la librairie "DueTimer.h"
  Timer.getAvailable().attachInterrupt(asservissementMD).setFrequency(1000.0/TECOMMANDE).start();
  Timer1.getAvailable().attachInterrupt(asservissementMG).setFrequency(1000.0/TECOMMANDE).start();
  Timer2.getAvailable().attachInterrupt(asservissement_translaterA).setFrequency(1000/TECOMMANDE);
  Timer3.getAvailable().attachInterrupt(asservissement_rotation).setFrequency(1000/TECOMMANDE);
  Timer4.getAvailable().attachInterrupt(asservissement_translaterR).setFrequency(1000/TECOMMANDE);
  Timer5.getAvailable().attachInterrupt(odometrie).setFrequency(1000.0/TECOMMANDE).start();
}
///fin setup()///////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
  //fin de partie
  while((startMillis+99000) <= millis())
  {
//    SPpulse = 0;
//    SPpulseD = 0;
//    SPpulseG = 0;
    
    CommandeMG=0; 
    CommandeMD=0;
    Applique_Commande(CommandeMD, PWM_MD, MD_IN1, MD_IN2);
    Applique_Commande(CommandeMG, PWM_MG, MG_IN1, MG_IN2);
    Serial2.print('A');
    arret_canon();
    display.setTextSize(4);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("FIN");
    display.display();
    display.clearDisplay();
    /*E1prev = false;
    E2prev = false;
    E3prev = false;
    E4prev = false;
    E5prev = false;
    E6prev = false;
    E7prev = false;
    E8prev = false;
    E9prev = false;
    E10prev = false;
    E11prev = false;
    E12prev = false;
    E13prev = false;
    E1 = false;
    E2 = false;
    E3 = false;
    E4 = false;
    E5 = false;
    E6 = false;
    E7 = false;
    E8 = false;
    E9 = false;
    E10 = false;
    E11 = false;
    E12 = false;
    E13 = false;*/
    Timer.stop();
    Timer1.stop();
    Timer2.stop();
    Timer3.stop();
    Timer4.stop();
    Timer5.stop();
    
  }
  
  /////////Partie
  cond12 = finAvancer12;
  cond23 = finAvancer23;
  cond34 = finServo34;
  cond45 = finAvancer45;
  cond56 = finServo56;
  cond67 = finReculer67;
  cond78 = finServo78;
  cond89 = finAvancer89;
  cond910 = finServo910;
  cond1011 = finReculer1011||finAvancer1011;
  cond1112 = finReculer1112||finAvancer1112;
  cond1213 = finLancement1213;
  cond1301 = finE13;
  
  E1 = (E1prev&&!cond12)||(E13prev&&cond1301);
  E2 = (E2prev&&!cond23)||(E1prev&&cond12);
  E3 = (E3prev&&!cond34)||(E2prev&&cond23);
  E4 = (E4prev&&!cond45)||(E3prev&&cond34);
  E5 = (E5prev&&!cond56)||(E4prev&&cond45);
  E6 = (E6prev&&!cond67)||(E5prev&&cond56);
  E7 = (E7prev&&!cond78)||(E6prev&&cond67);
  E8 = (E8prev&&!cond89)||(E7prev&&cond78);
  E9 = (E9prev&&!cond910)||(E8prev&&cond89);
  E10 = ((E10prev&&!cond1011)||(E9prev&&cond910));
  E11 = ((E11prev&&!cond1112)||(E10prev&&cond1011));
  E12 = ((E12prev&&!cond1213)||(E11prev&&cond1112));
  E13 = ((E13prev&&!cond1301)||(E12prev&&cond1213));
  /*if((startMillis+99000) <= millis())
  {
    while(true)
    {
      display.setTextSize(4);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("FIN");
      display.display();
      display.clearDisplay();
    }
  }
  */
  if(E1)
  {
    //Serial.println("attenteE1");
    if(finE1 == false)
    {   
      consigneOrientation = 0;
      etatAllerA = 'T';
      if(couleur =="VERT")
      {
        XCIBLE = 120;
        YCIBLE = 61;
      }
      else
      {
        XCIBLE = 120;
        YCIBLE = 239;
      }
      //XCIBLE = 58;
      //YCIBLE = 38;
      aller_a(XCIBLE,YCIBLE);
      //Serial.println("E1");
      finE1=true;
    }
    while((analogRead(captAvDroit)>600)||(analogRead(captAvGauche)>600))
    {
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
    } 
  }
  if(E2)
  {
    if(finE2 == false)
    {
      consigneOrientation = 0;
      etatAllerA = 'T';
      if(couleur =="VERT")
      {
        XCIBLE = 162;
        YCIBLE = 20;
      }
      else
      {
        XCIBLE = 162;
        YCIBLE = 280;
      }
      aller_a(XCIBLE,YCIBLE);
      //Serial.println("E2");
      finE2=true;
    }
    while((analogRead(captAvDroit)>600)||(analogRead(captAvGauche)>600))
    {
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
    }
  }
  if(E3)
  {
    if(finE3==false)
    {
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
      Serial2.print('S');
      if(chgtEtape==true)
      {
        chgtEtape = false;
        currentMillis3 = millis();
        //Serial.println("millis");
      }
      //Serial.println("E3");
      if((currentMillis3+1000)<millis())
      {
        chgtEtape = true;
        finServo34 = true;
        //Serial.println("finE3");
        finE3=true;
      }    
    }
  }
  if(E4)
  {
    if(finE4 == false)
    {
    consigneOrientation = 0;
    etatAllerA = 'T';
    if(couleur=="VERT")
    {
      XCIBLE = 191.5;
      YCIBLE = 14.5;
    }
    else
    {
      XCIBLE = 191.5;
      YCIBLE = 283.5;
    }
    aller_a(XCIBLE,YCIBLE);
    //Serial.println("E4");
    finE4=true;
    } 
    if(chgtEtape==true)
    {
      chgtEtape = false;
      currentMillis4 = millis();
    }
    if((currentMillis4+7000)<millis())
    {
      chgtEtape = true;
      finAvancer45 = true;
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
    }   
  }
  if(E5)
  {
    if(finE5==false) 
    {
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
      Serial2.print('B');
      /*if(chgtEtape==true)
      {
        chgtEtape = false;
        currentMillis5 = millis();
      }
      Serial.println("E5");
      if((currentMillis5+800)<millis())
      {
        chgtEtape = true;*/
        finServo56 = true;
        finE5=true;
      //}  
    }
  }
  if(E6)
  {
    if(finE6 == false)//ESSAYER DE METTRE MILLIS POUR LANCER RECULE
    {
      distanceR=15; 
      finE6=true;
    }
  }
  if(E7)
  {
    ConsigneVitesseRotMG=0; 
    ConsigneVitesseRotMD=0;
    Serial2.print('R');
    /*if(chgtEtape==true)
    {
      chgtEtape = false;
      currentMillis7 = millis();
    }
    Serial.println("E7");
    if((currentMillis7+2000)<millis())
    {
      chgtEtape = true;
      finServo78 = true;
    }*/
    chgtEtape = true;
    finServo78 = true;
  }
  if(E8)
  {
    if(finE8 == false)
    {
      consigneOrientation = 0;  
      etatAllerA = 'T';
      if(couleur =="VERT")
      {
        XCIBLE = 120;
        YCIBLE = 61;
      }
      else
      {
        XCIBLE = 120;
        YCIBLE = 239;
      }
      aller_a(XCIBLE,YCIBLE);
      //Serial.println("E8");
      finE8=true;
    }
    while((analogRead(captAvDroit)>600)||(analogRead(captAvGauche)>600))
    {
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
    }
  }
  if(E9)
  {
    if(finE9==false)
    {
      if(couleur=="VERT")
      {
        ConsigneVitesseRotMG=0; 
        ConsigneVitesseRotMD=0;
        Serial2.print('v');
        if(chgtEtape==true)
        {
          chgtEtape = false;
          currentMillis9 = millis();
        }
        //Serial.println("E9");
        if((currentMillis9+500)<millis())
        {
          chgtEtape = true;
          finServo910 = true;
          finE9=true;
        }
      }
      else
      {
        ConsigneVitesseRotMG=0; 
        ConsigneVitesseRotMD=0;
        Serial2.print('o');
        if(chgtEtape==true)
        {
          chgtEtape = false;
          currentMillis9 = millis();
        }
        //Serial.println("E9");
        if((currentMillis9+500)<millis())
        {
          chgtEtape = true;
          finServo910 = true;
          finE9=true;
        }
      }     
    }
  }
  if(E10)
  {
    if(finE10 == false)
    {
      consigneOrientation = 0;
      if(couleur=="VERT")
      {
        etatAllerA = 'T';
        XCIBLE = 90;
        YCIBLE = 40;
        aller_a(XCIBLE,YCIBLE);
      }
      else
      {
        etatReculerVers = 'T';
        XCIBLE = 90;
        YCIBLE = 260;
        reculer_vers(XCIBLE, YCIBLE);
      }
      finE10=true;
      //Serial.println("E10");
    }
    while((analogRead(captAvDroit)>600)||(analogRead(captAvGauche)>600))
    {
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
    }
  }
  if(E11)
  {
    if(finE11==false)
    {
      consigneOrientation = 0;
      etatAllerA = 'T';
      if(couleur=="VERT")
      {
        etatAllerA = 'T';
        XCIBLE = 88;
        YCIBLE = 10;
        aller_a(XCIBLE,YCIBLE);
      }
      else
      {
        etatReculerVers = 'T';
        XCIBLE = 88;
        YCIBLE = 300;
        reculer_vers(XCIBLE,YCIBLE);
      }
      finE11=true;
      //Serial.println("E11");
    }
    if(chgtEtape==true)
    {
      chgtEtape = false;
      currentMillis11 = millis();
    }
    if((currentMillis11+8000)<millis())
    {
      chgtEtape = true;
      finReculer1112 = true;
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
      Timer2.stop();
      Timer3.stop();
      Timer4.stop();
      CommandeMG=0; 
      CommandeMD=0;
      Applique_Commande(CommandeMD, PWM_MD, MD_IN1, MD_IN2);
      Applique_Commande(CommandeMG, PWM_MG, MG_IN1, MG_IN2);
    }
  }
  if(E12)
  {
//    ConsigneVitesseRotMG=0; 
//    ConsigneVitesseRotMD=0;
    
    if(finE12==false)
    {
      depart_canon();
      Timer2.stop();
      Timer3.stop();
      Timer4.stop();
      CommandeMG=0; 
      CommandeMD=0;
      Applique_Commande(CommandeMD, PWM_MD, MD_IN1, MD_IN2);
      Applique_Commande(CommandeMG, PWM_MG, MG_IN1, MG_IN2);
      /*if(chgtEtape==true)
      {
        chgtEtape = false;
        currentMillis12 = millis();
      }
      Serial.println("E12");
      if((currentMillis12+2000)<millis())
      {
        
      } */
      if(couleur=="VERT")
      {
        Serial2.print('V');
      }
      else
      {
        Serial2.print('O');
      }
      
      chgtEtape = true;
      finLancement1213 = true;
      finE12=true;
    }
  }
  if(E13)
  {
    if(finE13==false)
    {
      //ConsigneVitesseRotMG=0; 
      //ConsigneVitesseRotMD=0;
      Timer2.stop();
      Timer3.stop();
      Timer4.stop();
      CommandeMG=0; 
      CommandeMD=0;
      Applique_Commande(CommandeMD, PWM_MD, MD_IN1, MD_IN2);
      Applique_Commande(CommandeMG, PWM_MG, MG_IN1, MG_IN2);
      if(chgtEtape==true)
      {
        chgtEtape = false;
        currentMillis13 = millis();
      }
      //Serial.println("E13");
      if((currentMillis13+35000)<millis())
      {
        //arret_canon();
        //Serial2.print('A');
        chgtEtape = true;
        arret13 = true;
        //finE13=true;
        //ConsigneVitesseRotMG=0; 
        //ConsigneVitesseRotMD=0;
        /*display.setTextSize(3);
        display.setTextColor(WHITE);
         display.setCursor(0, 0);
        display.println("Gagné!");
        display.display();
        display.clearDisplay();*/
      }    
    }
    /*while((startMillis+99000) <= millis())
    {
      display.setTextSize(4);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("FIN");
      display.display();
      display.clearDisplay();
      ConsigneVitesseRotMG=0; 
      ConsigneVitesseRotMD=0;
      arret_canon();
      Serial2.print('A');
    }*/
  }
  E1prev = E1;
  E2prev = E2;
  E3prev = E3;
  E4prev = E4;
  E5prev = E5;
  E6prev = E6;
  E7prev = E7;
  E8prev = E8;
  E9prev = E9;
  E10prev = E10;
  E11prev = E11;
  E12prev = E12;
  E13prev = E13;
  E14prev = E14;

  //-----  Traitement des trames en réception ------------------
  /*while (Serial.available()) {
    char c    = Serial.read();  // Consomme un octet sur le buffer de réception et l'affecte à  c.
    float val = Serial.parseFloat();

    switch (c) {
      //Vitesse linéaire constante du robot en cm.s-1
      case 'S':
        Serial2.print('S');
        break;
        case 'B':
        Serial2.print('B');
        break;
      case 'V':
        V=val;
        ConsigneVitesseRotMG=V;
        ConsigneVitesseRotMD=V;
        //ChangeCmd = 1;
        break;
      case 'A':
        distanceA=val;
        break;
      case 'R':
        distanceR=val;
        break;
      case 'r':
        consigneOrientation = 0;
        etatReculerVers = 'T';
        //XCIBLE=0;
        //YCIBLE=0;
        reculer_vers(XCIBLE,YCIBLE);
        break;
      case 'T':
        angle=val;
        break;
      case 'X':
        XCIBLE=val;
        Serial.println(XCIBLE);
        break;
      case 'Y':
        YCIBLE=val;
        Serial.println(YCIBLE);
        break;
      case 'P':
        Serial.print("x(t)= ");
        Serial.print(xtETdt);
        Serial.print("cm ; y(t)=");
        Serial.print(ytETdt);
        Serial.print("cm ; theta(t)=");
        Serial.print(thetatETdt);
        Serial.print("rad = ");
        Serial.print(thetatETdt*57.29);
        Serial.println("° ;");
        consigneOrientation = 0;
        etatAllerA = 'T';
        //XCIBLE = 37;
        //YCIBLE = -50;
        //Timer6.start();
        aller_a(XCIBLE,YCIBLE);
       // orientation(XCIBLE,YCIBLE);
        break;
      default:;
    }
  }
  // ----Fin du traitement des réceptions rx.


  while (Serial1.available()) {
      char z = Serial1.read();
      Serial.print(z);
  }*/
  if(distanceA!=dprevA)
  {
    changementAvancer = 0;
    CountEncodeurCD=0;
    CountEncodeurCG=0;
    SPpulse = (distanceA*1024)*1.0314685/(3.141592*DRoueEnc);
    Timer2.start();
    dprevA=distanceA;
  }
  if(angle!=aprev)
  {
    changementRotation = 0;
    CountEncodeurCD=0;
    CountEncodeurCG=0;
    SPpulseD = angle * 3.20000 * 0.91;
    SPpulseG = -angle * 3.20000 * 0.91;
    Timer3.start();
    aprev=angle;
  }
  if(distanceR!=dprevR)
  {
    changementReculer = 0;
    CountEncodeurCD=0;
    CountEncodeurCG=0;
    SPpulse = -(distanceR*1024)*1.0314685/(3.141592*DRoueEnc);
    Timer4.start();
    dprevR=distanceR;
  }
//  EcritureData();

  
}
/////fin void loop()//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////Asservissements en vitesse des moteurs////////////////////////////////////////////////////////////
////////////////////////////Moteur droit
void asservissementMD()
{
  for (int i = 0; i < NmoyenneMobile; i++) 
  {
    TabCountCodeurMD[i] += CountEncodeurMD;
  }
  CountEncodeurMD = 0;
  DeltaPositionCodeurMD = TabCountCodeurMD[indiceTicksCodeurMD];
  TabCountCodeurMD[indiceTicksCodeurMD] = 0;
  indiceTicksCodeurMD++;
  if (indiceTicksCodeurMD==NmoyenneMobile) {indiceTicksCodeurMD = 0;}
  omegaMD = ((2.*3.141592 * ((double)DeltaPositionCodeurMD)) / 1869.381818) / (NmoyenneMobile * dt);

  eMD = ConsigneVitesseRotMD - omegaMD;     
  uMD = u1MD + 1.025*eMD - 0.975*e1MD; //Kp=1 Taui=0.200
  u1MD = uMD;     //Permet a chaque itération de mettre la valeur précédente pour le calcul de la valeur actuelle.
  e1MD = eMD;
  CommandeMD = uMD;   // On redéfini u comme la  nouvelle commande et plus U.
  Applique_Commande(CommandeMD, PWM_MD, MD_IN1, MD_IN2);
  temps += dt;
}

////////////////////////////////Moteur Gauche
void asservissementMG()
{
  for (int i = 0; i < NmoyenneMobile; i++) 
  {
    TabCountCodeurMG[i] += CountEncodeurMG;
  }
  CountEncodeurMG = 0;
  DeltaPositionCodeurMG = TabCountCodeurMG[indiceTicksCodeurMG];
  TabCountCodeurMG[indiceTicksCodeurMG] = 0;
  indiceTicksCodeurMG++;
  if (indiceTicksCodeurMG == NmoyenneMobile) {indiceTicksCodeurMG = 0;}
  omegaMG = ((2.*3.141592 * ((double)DeltaPositionCodeurMG)) / 1869.381818) / (NmoyenneMobile * dt); // en rad/s
  eMG = ConsigneVitesseRotMG - omegaMG;    
  uMG = u1MG + 1.025*eMG - 0.975*e1MG; //Kp=1.8 Taui=0.135 
  u1MG = uMG;     //Permet a chaque itération de mettre la valeur précédente pour le calcul de la valeur actuelle.
  e1MG = eMG;
  CommandeMG = uMG; 
  Applique_Commande(CommandeMG, PWM_MG, MG_IN1, MG_IN2);
}
//-------------------------------------------------------------------------------------------------------------------------------------------//

////////////////////////////////////////////////////////Asservissement en déplacements du robot////////////////////////////////////////////////
/////////////////////////////////////Marche avant
void asservissement_translaterA()
{
 erreurTransMD = SPpulse-CountEncodeurCD;
 erreurTransMG = SPpulse-CountEncodeurCG;
 
 erreurTransOrientation = CountEncodeurCG - CountEncodeurCD;
 
 ConsigneVitesseRotMD = GainTransDroit * erreurTransMD + gainOrientationTransDroitA * erreurTransOrientation;
 ConsigneVitesseRotMG = GainTransGauche * erreurTransMG - gainOrientationTransGaucheA * erreurTransOrientation;
 
 if(ConsigneVitesseRotMD > 5){ConsigneVitesseRotMD = 5 + gainOrientationTransDroitA * erreurTransOrientation;} 
 if(ConsigneVitesseRotMG > 5){ConsigneVitesseRotMG = 5 - gainOrientationTransGaucheA * erreurTransOrientation;}
 
 if((erreurTransMD <= 8) && (erreurTransMG <= 8) && (erreurTransMD >= -8) && (erreurTransMG >= -8))
 {
  if(E1){finAvancer12 = true;}
  if(E2){finAvancer23 = true;}
  if(E4){finAvancer45 = true;}
  if(E8){finAvancer89 = true;}
  if(E10){finAvancer1011 = true;}
  if(E11){finAvancer1112 = true;}
  //Serial.println("avancer OK");
  changementAvancer = 1;
  ConsigneVitesseRotMD = 0; 
  ConsigneVitesseRotMG = 0;
  Timer2.stop();
 }
}

//////////////////////////////////////Marche arrière
void asservissement_translaterR()
{
 erreurTransMD = SPpulse-CountEncodeurCD;
 erreurTransMG = SPpulse-CountEncodeurCG;
 
 erreurTransOrientation = CountEncodeurCG - CountEncodeurCD;
 
 ConsigneVitesseRotMD = GainTransDroit * erreurTransMD + gainOrientationTransDroitR * erreurTransOrientation;
 ConsigneVitesseRotMG = GainTransGauche * erreurTransMG - gainOrientationTransGaucheR * erreurTransOrientation;
 
 if(ConsigneVitesseRotMD < -5){ConsigneVitesseRotMD = -5 + gainOrientationTransDroitR * erreurTransOrientation;} 
 if(ConsigneVitesseRotMG < -5){ConsigneVitesseRotMG = -5 - gainOrientationTransGaucheR * erreurTransOrientation;}
  
 if((erreurTransMD <= 8) && (erreurTransMG <= 8) && (erreurTransMD >= -8) && (erreurTransMG >= -8))
 {
  if(E6){finReculer67 = true;}
  if(E10){finReculer1011 = true;}
  if(E11){finReculer1112 = true;}
  //Serial.println("reculer OK");
  changementReculer = 1;
  ConsigneVitesseRotMD = 0; 
  ConsigneVitesseRotMG = 0;
  Timer4.stop();
 }
}
///////////////////////////////////////Virages à droite (-) et à gauche (+)
void asservissement_rotation()
{
  erreurRotMD = SPpulseD - CountEncodeurCD;
  erreurRotMG = SPpulseG - CountEncodeurCG;

  ConsigneVitesseRotMD = gainRotDroit * erreurRotMD;
  ConsigneVitesseRotMG = gainRotGauche * erreurRotMG;

  if(ConsigneVitesseRotMD > 3){ConsigneVitesseRotMD = 3;} 
  if(ConsigneVitesseRotMG > 3){ConsigneVitesseRotMG = 3;}
  if(ConsigneVitesseRotMD < 3){ConsigneVitesseRotMD = -3;} 
  if(ConsigneVitesseRotMG < 3){ConsigneVitesseRotMG = -3;}
    
  if((erreurRotMD <= 10) && (erreurRotMG <= 10) && (erreurRotMD >= -10) && (erreurRotMG >= -10))
  {
    //Serial.println("tourner OK");
    changementRotation = 1;
    ConsigneVitesseRotMD = 0; 
    ConsigneVitesseRotMG = 0;
    Timer3.stop();
    if(etatAllerA=='T')//&&(etatReculerVers=='R'))
    {
      etatAllerA = 'A';
      aller_a(XCIBLE,YCIBLE);
    }
    if(etatReculerVers=='T')
    {
      etatReculerVers = 'R';
      reculer_vers(XCIBLE, YCIBLE);
    }
  }
}
/////////////////////////////////////////////////Aller sur la cible
void aller_a(float xCible, float yCible)
{
  switch (etatAllerA) 
  {
    case 'T':
      //consigneOrientation = (orientation(XCIBLE, YCIBLE)-2*atan(tan(thetat/2)));
      consigneOrientation = (orientation(xCible, yCible)-2*atan(tan(thetat/2)));
      consigneOrientation = 2*atan(tan(consigneOrientation/2))*57.29;
      //Serial.print("consigneOrientation");
      //Serial.println(consigneOrientation);
      changementRotation = 0;
      CountEncodeurCD=0;
      CountEncodeurCG=0;
      SPpulseD = consigneOrientation * 3.20000 * 0.91;
      SPpulseG = -consigneOrientation * 3.20000 * 0.91;
      Timer3.start();
      break;
    case 'A':
      distanceCible = sqrt((xCible-xt)*(xCible-xt)+(yCible-yt)*(yCible-yt));
      changementAvancer = 0;
      //finAvancer = false;
      CountEncodeurCD=0;
      CountEncodeurCG=0;
      SPpulse = (distanceCible*1024)*1.0314685/(3.141592*DRoueEnc);
      Timer2.start();
      //XCIBLE=xt;
      //YCIBLE=yt;
      break;
    default:;
  }
}

void reculer_vers(float xCible, float yCible)
{
  switch (etatReculerVers) 
  {
    case 'T':
      consigneOrientation = (orientation(xCible, yCible)-3.14-2*atan(tan(thetat/2)));
      consigneOrientation = 2*atan(tan(consigneOrientation/2))*57.29;
      //Serial.print("consigneOrientation");
      //Serial.println(consigneOrientation);
      changementRotation = 0;
      CountEncodeurCD=0;
      CountEncodeurCG=0;
      SPpulseD = consigneOrientation * 3.20000 * 0.91;
      SPpulseG = -consigneOrientation * 3.20000 * 0.91;
      Timer3.start();
      break;
    case 'R':
      distanceCible = sqrt((xCible-xt)*(xCible-xt)+(yCible-yt)*(yCible-yt));
      /*Serial.println(xt);
      Serial.println(yt);
      Serial.print("Cible");
      Serial.println(distanceCible);
      Serial.println(xCible);
      Serial.println(yCible);*/
      //changementreculer = 0;
      //finAvancer = false;
      CountEncodeurCD=0;
      CountEncodeurCG=0;
      SPpulse = -(distanceCible*1024)*1.0314685/(3.141592*DRoueEnc);
      Timer4.start();
      //XCIBLE=xt;
      //YCIBLE=yt;
      break;
    default:; 
  }
}

float orientation(int xO, int yO)
{
  c = (xO - xt)/sqrt((xO - xt)*(xO - xt)+(yO - yt)*(yO - yt));
  s = (yO - yt)/sqrt((xO - xt)*(xO - xt)+(yO - yt)*(yO - yt));
  CapObjectif = atan2(s,c);
  return CapObjectif; 
}
//--------------------------------------------------------------------------------------------------------------------------------------------//

//////////////////////////////////////////////////////////////Calcul de la postion du robot/////////////////////////////////////////////////////
void odometrie() {
  // Fonction executée toutes les TECOMMANDE ms sur interruption.
  // Afin de faciliter l'odométrie, veuillez mettre en commentaire la commande des moteurs dans la boucle asservissement
  
  CountEncodeurCGTempsCourant = CountEncodeurCGodo - CountEncodeurCGTempsPrecedent;
  CountEncodeurCDTempsCourant = CountEncodeurCDodo - CountEncodeurCDTempsPrecedent;
  DRG = (3.1416*DRoueEnc*CountEncodeurCGTempsCourant)/1024.0000;
  DRD = (3.1416*DRoueEnc*CountEncodeurCDTempsCourant)/1024.0000;
  DC = (DRG+DRD)*0.5;
  xtETdt = xt + DC*cos(thetat);
  ytETdt = yt + DC*sin(thetat);
  thetatETdt = thetat + ((DRD-DRG)/EntraxeRoueEnc);
  
  /*Serial.print("x(t)= ");
  Serial.print(xtETdt);
  Serial.print("cm ; y(t)=");
  Serial.print(ytETdt);
  Serial.print("cm ; theta(t)=");
  Serial.print(thetatETdt);
  Serial.print("rad = ");
  Serial.print(thetatETdt*57.29);
  Serial.println("° ;");*/
  
  xt=xtETdt;
  yt = ytETdt;
  thetat = thetatETdt;
  CountEncodeurCGTempsPrecedent = CountEncodeurCGodo;
  CountEncodeurCDTempsPrecedent = CountEncodeurCDodo;
}
//--------------------------------------------------------------------------------------------------------------------------------------------//

/////////////////////////////////////////////////////////Fonctions gerant le moteur du canon////////////////////////////////////////////////////
void depart_canon()
{
  for(int p=255; p>vitesse_canon; p--)
  {
    digitalWrite(pin1_canon, HIGH);
    analogWrite(pin2_canon, p);
    delay(10);
  }
}

void arret_canon()
{
  digitalWrite(pin1_canon, HIGH);
  digitalWrite(pin2_canon, HIGH);
}
//--------------------------------------------------------------------------------------------------------------------------------------------//

/////////////////////////////////////////////////////////Fonction gérant les servos dynamixel du robot///////////////////////////////////////////

//////////////////////////////////////////////////////////Envoie de données pour Scilab afin d'asservir le robot//////////////////////////////////
//ATTENTION si EcritureData est mdoifié, le script Scilab permettant l'acquisition des données doit être repris
void EcritureData(void) {
  // Ecriture des données sur le port série toutes le TEDATA millisecondes.
  if (ChangeCmd == 1) {
    TempsCourant  =  millis();

    if ( TempsCourant - TempsDernierEnvoi > TEDATA) {

      Serial.print(temps);
      Serial.print(",");
      Serial.print(CommandeMG);
      Serial.print(",");
      Serial.print(CommandeMD);
      Serial.print(",");
      Serial.print(omegaMG);
      Serial.print(",");
      Serial.print(omegaMD);
      Serial.print(",");
      Serial.print(ConsigneVitesseRotMD);
      Serial.print(",");
      Serial.print(ConsigneVitesseRotMG);
      Serial.print(";");
      Serial.print("\r"); Serial.print("\n");
      TempsDernierEnvoi = TempsCourant;
      i++;
      if (i == DataSize) {ChangeCmd = 0;
      i=0;}     
  }
 }
}
//--------------------------------------------------------------------------------------------------------------//

//////////////////////////////////////////////////////////////////////////////////////
//Définition de 6 fonctions permettant de calculer le nombre de front montant de chaque encodeur
//////////////////////////////////////////////////////////////////////////////////////
void fmMG_B() {
  // Sur interruption sur front montant et descendant d'EncodeurB
  if (digitalRead(Enc_RMG_VA) == digitalRead(Enc_RMG_VB)) {
    CountEncodeurMG++;
  }
  else {
    CountEncodeurMG--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmMG_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RMG_VA) == digitalRead(Enc_RMG_VB)) {
    CountEncodeurMG--;
  }
  else {
    CountEncodeurMG++;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmMD_B() {
  // Sur interruption sur front montant et descendant d'EncodeurB
  if (digitalRead(Enc_RMD_VA) == digitalRead(Enc_RMD_VB)) {
    CountEncodeurMD++;
  }
  else {
    CountEncodeurMD--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmMD_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RMD_VA) == digitalRead(Enc_RMD_VB)) {
    CountEncodeurMD--;
  }
  else {
    CountEncodeurMD++;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmCG_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RCG_VA) == digitalRead(Enc_RCG_VB)) {
    CountEncodeurCG++;
    CountEncodeurCGodo++;
  }
  else {
    CountEncodeurCG--;
    CountEncodeurCGodo--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmCD_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RCD_VA) == digitalRead(Enc_RCD_VB)) {
    CountEncodeurCD++;
    CountEncodeurCDodo++;
  }
  else {
    CountEncodeurCD--;
    CountEncodeurCDodo--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////
