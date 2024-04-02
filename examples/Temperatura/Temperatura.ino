/*
  Laboratorio movil
  Libreria del labotario movil: Medición de las sondas de temperatura
  Se puede ocupar estos ejemplos para cambiar el modo de uso del laboratorio movil,
  de modo que se pueda adaptar a los distintos experimentos desarrollados.
  Autor: Juan Carlos Velázquez Díaz
*/
#include <LabMovilF.h>

LabMovilF Temp = LabMovilF(); // Creación del Objeto Laboratorio movil sobre el cual funciona la libreria.

void setup() {
  
  Serial.begin(9600);         // Inicializa la comunición Serial a 9600.  
  Temp.LabBegin(1);           // Inicializa los sensores del Laboratorio.
  
}

void loop() {
                              // La función "TempExterna" realiza la medición de cada sonda
  Temp.TempExterna();         // de temperatura y los imprime en la lcd, monitor y dispositivo bluetooth
  delay(300);                 // Espera de 200 milisegundos para la correcta visualización
   
}
