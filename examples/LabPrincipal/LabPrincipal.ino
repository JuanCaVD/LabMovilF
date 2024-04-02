/*
  Laboratorio movil
  Libreria del labotario movil: Programa principal del laboratorio Movil
  Programa para el correcto funcionamiento del laboratorio movil,que permite la manipulación de 
  cada uno de los sensores de forma manual, permitiendo a usuario manejar el dispositivo sin
  necesidad de conocimiento alguno sobre programación.
  Autor: Juan Carlos Velázquez Díaz
*/
#include <LabMovilF.h> 
#define TiempoRebote 220L 

LabMovilF Lab = LabMovilF();  // Creación del Objeto Laboratorio movil sobre el cual funciona la libreria.

bool InicioM = 1;             // Registro que permite regresar el menú de inicio.
bool ModoStart = 1;           // Registro que permite indicar la elección dentro del menú.
int8_t MenuVal = 126;         // Variable para la selección del menú. 
uint32_t TimerStart = 0;      // Variable para el control del tiempo en el programa

void PinMenu();               // Definición para la función  la interrupción del menú. 
void PinModo();               // Definición para la función  la interrupción de la elección.

void setup() {
  
  Serial.begin(9600);         // Inicializa la comunición Serial a 9600.
  Lab.LabBegin();             // Inicializa los sensores del Laboratorio.
  attachInterrupt(digitalPinToInterrupt(B_Inicio), PinMenu, RISING );
                              // Implementación de la interrución del botón de inicio.
  attachInterrupt(digitalPinToInterrupt(B_Modo), PinModo, RISING );
                              // Implementación de la interrución del botón de modo.
  TimerStart = millis();      // Reinicio del contador del tiempo.
  //while((millis() - TimerStart) < 100){}
                              // Espera de medio segundo.
  Lab.MenuLab();              // Inicio del programa e impresión del menú.
  
}

void loop() {
  
  MenuVal = Lab.MenuLab(MenuVal,InicioM, ModoStart);
                              /* 
                               *  Función que permite iniciar el menu, seleccionar el modo de trabajo
                               *  y cambiar el modo de trabajo. Es la función principal para el correcto 
                               *  funcionamiento del laboratorio de forma autónoma.
                               */
  if(!digitalRead(B_2)){      // Lectura del botón 2
    if((millis() - TimerStart) > TiempoRebote){  // Impide rebotes, falsas lecturas de pulsación
      TimerStart = millis();  // Reinicio del contador del tiempo.
      InicioM = 0;            // Reinicia la variable del menú.
      if(ModoStart){          // Impide el cambio de modo después de seleccionarlo.
                              /* Control de flujo del menú */
        MenuVal = (MenuVal == 126) ? -1 : MenuVal;
        MenuVal = (MenuVal >= 8) ? 0 : MenuVal+1;
      }
    }
  }
  if(!digitalRead(B_1)){      // Lectura del botón 1
    if((millis() - TimerStart) > TiempoRebote){  // Impide rebotes, falsas lecturas de pulsación.
      TimerStart = millis();  // Reinicio del contador del tiempo.
      InicioM = 0;            // Reinicia la variable del menú.
      if(ModoStart){          // Impide el cambio de modo después de seleccionarlo.
                              /* Control de flujo del menú */
        MenuVal = (MenuVal == 126) ? 9 : MenuVal;
        MenuVal = (MenuVal <= 0) ? 8 : MenuVal-1 ;
      }
    }
  }
}

void PinModo(){               // Función para seleccionar el modo de trabajo.
  if((millis() - TimerStart) > TiempoRebote ){ // Impide rebotes, falsas lecturas de pulsación.
    TimerStart = millis();    // Reinicio del contador del tiempo.
    if(!InicioM){
      ModoStart = 0;            // Habilita la toma de datos.
    }
  }
}
void PinMenu(){
  if((millis() - TimerStart) > TiempoRebote ){ // Impide rebotes, falsas lecturas de pulsación.
    TimerStart = millis();    // Reinicio del contador del tiempo.
    ModoStart = 1;            // Deshabilita la toma de datos.
    InicioM = 1;              // Imprime la pantalla de menú.
  }
}
