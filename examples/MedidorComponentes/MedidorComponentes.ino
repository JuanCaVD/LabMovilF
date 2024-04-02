/*
  Laboratorio movil
  Libreria del labotario movil: Medidor de componentes 
  Se puede ocupar estos ejemplos para cambiar el modo de uso del laboratorio movil,
  de modo que se pueda adaptar a los distintos experimentos desarrollados.
  Autor: Juan Carlos Velázquez Díaz
*/
#include <LabMovilF.h>

LabMovilF LAB = LabMovilF();  // Creación del Objeto Laboratorio movil sobre el cual funciona la libreria.

void setup() {
  
  Serial.begin(9600);         // Inicializa la comunición Serial a 9600
  LAB.LabBegin(1);
  
}

void loop() {
  
  if(!digitalRead(B_1)){      // Ocupa el B_1 para inicializar la medición.
                              // La función "Medidor_ESR" inicia la comprobación del componente
    LAB.Medidor_ESR();        // imprimiendo el resultado en la lcd y monitor.
    
  }

}
