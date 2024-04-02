/*
  Laboratorio movil
  Libreria del labotario movil: Medidor de Resistencias
  Se puede ocupar estos ejemplos para cambiar el modo de uso del laboratorio movil,
  de modo que se pueda adaptar a los distintos experimentos desarrollados.
  Autor: Juan Carlos Velázquez Díaz
*/
#include <LabMovilF.h>

LabMovilF LAB = LabMovilF();  // Creación del Objeto Laboratorio movil sobre el cual funciona la libreria.

void setup() {
  
  Serial.begin(9600);         // Inicializa la comunición Serial a 9600.
  LAB.LabBegin(1);             // Inicializa los sensores del Laboratorio.

}

void loop() {
                              // La función "Resistencia" realiza la medición constante de la resistencia
  LAB.Resistencia();          // y la imprime en la lcd, monitor y dispositivo bluetooth
  
}
