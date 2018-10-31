#include <Arduino.h>
#include "Applique_Commande.h"

float TensionAlim = 12.0; // en volt. La valeur maximale de la commande.

////////////////////////////////////////////////////////////////////////////////////////
// Applique les commandes (-100% to 100%) *Tension Alimentation sur les 2 sorties PWM.
// Les commandes sont normalisÃ©es 0-100% vers 0-255 (PWM codÃ©es sur 8bits) + Direction.
////////////////////////////////////////////////////////////////////////////////////////
void Applique_Commande(float Commande_Moteur_enV, int PWM_M, int M_IN1, int M_IN2) {
  // Gamme: -100 ÃƒÂ  +100% de la TensionAlim
  // Commandes remappÃ©es ultÃ©rieurement : 0 Ã  255 sur les sorties analogiques.

  int Cmd = (int)(255 * (Commande_Moteur_enV / TensionAlim));

  // Saturation
  if (Cmd > 255) {
    Cmd = 255;
  }
  if (Cmd < -255) {
    Cmd = -255;
  }

  // Ecriture effective de la commande
  if (Cmd >= 0)
  { digitalWrite(M_IN1, HIGH);
    digitalWrite(M_IN2, LOW);
    analogWrite(PWM_M, Cmd);
  }
  else {
    digitalWrite(M_IN1, LOW);
    digitalWrite(M_IN2, HIGH);
    analogWrite(PWM_M, -Cmd);
  }
}  // Fin Applique_Commande()
/////////////////////////////////////////////////////////////////////////////////////////



