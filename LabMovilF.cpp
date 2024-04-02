#include <LabMovilF.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

/* -----  Inicia el objeto para el NeoPixel  ----------*/

Adafruit_NeoPixel neo(Num_leds, Pin_neo, NEO_GRBW + NEO_KHZ800);

/* -----  Inicia el objeto para el sensor ds18b20 para medir la temperatura  ----------*/

OneWire Sensores_DS18B20(9);  // Definicion del Sensor de temperaruta DS18B20

/* -----  Inicia el objeto para la comunicacion Serial con el ESP32 y el ESR  ----------*/

SoftwareSerial ESP32(7, 6);

SoftwareSerial ESR(11, 10);

/*-------  Inicia el objeto para el sensor BH1750 para medir la luminosidad  ----------*/

#define __wire_write(d) I2C->write(d)

#define __wire_read() I2C->read()



LabMovilF::LabMovilF(){
	pinMode(PIN_OUT, OUTPUT);
	pinMode(PIN_IN, OUTPUT);
	pinMode(B_Inicio, INPUT);
	pinMode(B_Modo, INPUT);
	pinMode(B_1, INPUT);
	pinMode(B_2, INPUT);
	pinMode(TriggerPin, OUTPUT);
	pinMode(EchoPin, INPUT);
	digitalWrite(TriggerPin,LOW);
	Wire.begin();
	neo.begin();
	neo.setBrightness(Brillo_neo);
	
}

void LabMovilF::LabBegin(bool SaltarMenu_) {
	ads.begin();
	LuxBegin();
	MCP3421begin();
	SetPGA_MCP3421();
	ESP32.begin(38400);
	SaltarMenu = SaltarMenu_;
}

uint8_t LabMovilF::MenuLab(uint8_t MenuVal , bool MenuEstado_ , bool ModoEstado_ ){

	Car_Auxliar = '~';


	if (MenuEstado_) {
		MenuVal = 126;
		Inicia_modo_ = 1;
		neo.setPixelColor(0, neo.Color(0, 0, 0));
		neo.show();
	}
	if ((ModoEstado_ == 0) && Inicia_modo_) {
		ESP32.write('@');
		Inicia_modo_ = 0;
		delay(2);
	}
	if ((!ModoEstado_) || ((millis() - TiempoImpresionMenu) > 220)) {
		TiempoImpresionMenu = millis();
		switch (MenuVal) {
		case 0:
			if (ModoEstado_) {
				ESP32.write('0');
			}
			else {
				TempExterna();
				Var32_Auxiliar_1 = millis();
				while ((millis() - Var32_Auxiliar_1) < 200) {
				}
			}
			break;
		case 1:
			if (ModoEstado_) {
				ESP32.write('1');
			}
			else {
				Var32_Auxiliar_1 = millis();
				while ((millis() - Var32_Auxiliar_1) < 500) {
				}
				luxometro();
			}
			break;
		case 2:
			if (ModoEstado_) {
				ESP32.write('2');
			}
			else {
				Voltaje();
			}
			break;
		case 3:
			if (ModoEstado_) {
				ESP32.write('3');
			}
			else {
				Resistencia();
			}
			break;
		case 4:
			if (ModoEstado_) {
				ESP32.write('4');
			}
			else {
				Capacitancia();
			}
			break;
		case 5:
			if (ModoEstado_) {
				ESP32.write('5');
			}
			else {
				Var32_Auxiliar_1 = millis();
				while ((millis() - Var32_Auxiliar_1) < 350) {
				}
				DistanciaA();
			}
			break;
		case 6:
			if (ModoEstado_) {
				ESP32.write('6');
			}
			else {
				CorrienteD();
			}
			break;
		case 7:
			if (ModoEstado_) {
				ESP32.write('7');
			}
			else {
				Medidor_ESR();
				Var32_Auxiliar_1 = millis();
				while ((millis() - Var32_Auxiliar_1) < 200) {}
				ESP32.end();
				return MenuVal;
			}
			break;
		case 8:
			if (ModoEstado_) {
				ESP32.write('8');
			}
			else {
				neo.setPixelColor(0, neo.Color(32, 178, 170));
				neo.show();
				ESP32.write(Car_Auxliar);
				delay(1);
				ESP32.write('@');
				Var32_Auxiliar_1 = millis();
				while ((millis() - Var32_Auxiliar_1) < 350) {}
				ESP32.end();
				return MenuVal;
			}
			break;
		default:
			ESP32.write('`');
			delay(1);
			break;
		}
		if (ModoEstado_ && !MenuEstado_) {
			delay(1);
			ESP32.write(Car_Auxliar);
		}
	}
	return MenuVal;
}


void LabMovilF::PrintData(uint8_t DataModeSelect, float DataSensor, uint8_t NumData, bool imprime_) {
	Car_Auxliar = '~';
	String DataSensor_;
	ResultadoDeLaLectura = DataSensor;

	DataSensor_ = String(DataModeSelect);
	for (int i = 0; i <= DataSensor_.length(); i++) {
		ESP32.write(DataSensor_.charAt(i));
		delay(1);
	}
	ESP32.print(Car_Auxliar);
	delay(1);
	DataSensor_ = String(DataSensor,3);
	for (int i = 0; i <= DataSensor_.length(); i++) {
		ESP32.write(DataSensor_.charAt(i));
		delay(1);
	}
	ESP32.print(Car_Auxliar);
	delay(1);
	DataSensor_ = String(NumData);
	for (int i = 0; i <= DataSensor_.length(); i++) {
		ESP32.write(DataSensor_.charAt(i));
		delay(1);
	}
	ESP32.print(Car_Auxliar);
	delay(1);
	if (imprime_) {
		switch (DataModeSelect){
		case 0:

			Serial.print(F("Temperatura del sensor "));
			Serial.print(NumData);
			Serial.print(F(": "));
			Serial.print(DataSensor, 2);
			Serial.println(F(" Celsius"));
			break;
		case 1:

			Serial.print(F("Luz: "));
			Serial.print(DataSensor, 2);
			Serial.println(F(" lx"));
			break;
		case 2:

			Serial.print(F("Voltaje: "));
			Serial.print(DataSensor, 5);
			Serial.println(F(" V"));
			break;
		case 3:

			if (NumData == 1) {
				Serial.print(F("Resistencia: "));
				if (DataSensor <= 1000) {
					Serial.print(DataSensor, 2);
					Serial.println(F(" Ω"));
				}
				else if (DataSensor > 1001 && DataSensor <= 1000000) {
					Serial.print(DataSensor / 1000.0, 2);
					Serial.println(F(" KΩ"));
				}
				else if (DataSensor > 1000001) {
					Serial.print(DataSensor / 1000000.0, 2);
					Serial.println(F(" MΩ"));
				}
			}
			break;
		case 4:

			Serial.print(F("Capacitancia: "));
			if (NumData == 1) {
				Serial.print(DataSensor, 2);
				Serial.println(F(" pF"));
			}
			else if (NumData == 2) {
				Serial.print(DataSensor, 2);
				Serial.println(F(" nF"));
			}
			else if (NumData == 3) {
				Serial.print(DataSensor, 2);
				Serial.println(F(" uF"));
			}
			neo.setPixelColor(0, neo.Color(0, 0, 0));
			neo.show();
			break;
		case 5:
			if (DataSensor > 0) {
				Serial.print(F("Distancia: "));
				Serial.print(DataSensor, 2);
				Serial.println(F(" cm"));
			}
			else {
				Serial.println(F("Fuera de rango "));
			}
			break;
		case 6:

			Serial.print(F("Corriente: "));
			Serial.print(DataSensor, 3);
			Serial.println(F(" A"));
			break;

		default:
			break;
		}
	}

}

void LabMovilF::PrintLCD(String MensajeEnviado, byte tipo ) {

	Car_Auxliar = '~';
	ESP32.write(Car_Auxliar);
	delay(1);
	ESP32.write(Car_Auxliar);
	delay(1);
	if (tipo == 0) {
		for (int i = 26; i <= MensajeEnviado.length() - 3; i++) {
			if (MensajeEnviado.charAt(i) == '+') {
				i += 22;
			}
			ESP32.write(MensajeEnviado.charAt(i));
			Serial.write(MensajeEnviado.charAt(i));
			delay(1);
		}
	}
	else {
		for (int i = 0; i <= MensajeEnviado.length(); i++) {
			Serial.write(MensajeEnviado.charAt(i));
			ESP32.write(MensajeEnviado.charAt(i));
			delay(2);
		}
	}
	
	return;

}

/*-------------------- Distancia SR-HC04 ------------------------------------------------*/

void LabMovilF::DistanciaA(bool imprime_) {

	if (SaltarMenu) {
		MenuLab(5, 1, 0);
		delay(1);
		SaltarMenu = 0;
	}
	neo.setPixelColor(0, neo.Color(218, 165, 32));
	neo.show();
	digitalWrite(TriggerPin,HIGH);
	delayMicroseconds(10);
	digitalWrite(TriggerPin, LOW);
	Var32_Auxiliar_1 = pulseIn(EchoPin,HIGH,30000);
	Varfloat_Auxiliar_1 = Var32_Auxiliar_1 * VelocidadDistintoMedio;
	PrintData(5, Varfloat_Auxiliar_1, 0, imprime_);

}

/*-------------------- Corriente ACS712 ------------------------------------------------*/

void LabMovilF::CorrienteD(bool imprime_) {

	if (SaltarMenu) {
		MenuLab(6, 1, 0);
		delay(1);
		SaltarMenu = 0;
	}
	neo.setPixelColor(0, neo.Color(0, 255, 255));
	neo.show();
	ads.setDataRate(RATE_ADS1115_250SPS);
	Var32_Auxiliar_1 = 0;
	Var32_Auxiliar_2 = 0;
	for (int i = 0; i < 50; i++) {
		Var32_Auxiliar_1 += ads.readADC_SingleEnded(0);
		delayMicroseconds(50);
		Var32_Auxiliar_2 += ads.readADC_SingleEnded(2);
	}
	Var32_Auxiliar_1 = Var32_Auxiliar_1 / 50;
	Var32_Auxiliar_2 = Var32_Auxiliar_2 / 50;
	Serial.println(ads.computeVolts(Var32_Auxiliar_1));
	Varfloat_Auxiliar_1 = (ads.computeVolts(Var32_Auxiliar_2) - (ads.computeVolts(Var32_Auxiliar_1) / 2 ) );

	PrintData(6, (Varfloat_Auxiliar_1 / SensibilidadACS712), 0, imprime_);

}

/*--------------------Temepraruta externa DS18B20------------------------------------------------*/

void LabMovilF::TempExterna(bool imprime_) {

	if (SaltarMenu) {
		MenuLab(0, 1, 0);
		delayMicroseconds(1200);
		SaltarMenu = 0;
	}

	neo.setPixelColor(0, neo.Color(255, 0, 0));
	neo.show();

	while (Sensores_DS18B20.search(Array_Auxiliar_1)) {
		Sensores_DS18B20.reset();
		Sensores_DS18B20.select(Array_Auxiliar_1);
		Sensores_DS18B20.write(0x44, 1);
	}
	Varfloat_Auxiliar_1 = millis();
	while ((millis() - Var32_Auxiliar_1 < 800)){}
	Var8_Auxiliar_2 = 1;

	while (Sensores_DS18B20.search(Array_Auxiliar_1)) {
		Sensores_DS18B20.reset();
		Sensores_DS18B20.select(Array_Auxiliar_1);
		Sensores_DS18B20.write(0xBE);         // Lectura de los datos
		for (Var8_Auxiliar_1 = 0; Var8_Auxiliar_1 < 9; Var8_Auxiliar_1++) {  // Se necesitna 9 byte para los datos
			Array_Auxiliar_2[Var8_Auxiliar_1] = Sensores_DS18B20.read();
		}
		Var16_Auxiliar_1 = (Array_Auxiliar_2[1] << 8) | Array_Auxiliar_2[0];
		Var8_Auxiliar_1 = (Array_Auxiliar_2[4] & 0x60);  // a baja resolución, los bits bajos no están definidos, se ponen a cero
		if (Var8_Auxiliar_1 == 0x00) Var16_Auxiliar_1 = Var16_Auxiliar_1 & ~7;  // 9 bit de resolucion, 93.75 ms
		else if (Var8_Auxiliar_1 == 0x20) Var16_Auxiliar_1 = Var16_Auxiliar_1 & ~3; // 10 bit de resolucion, 187.5 ms
		else if (Var8_Auxiliar_1 == 0x40) Var16_Auxiliar_1 = Var16_Auxiliar_1 & ~1; // 11 bit de resolucion, 375 ms
		//// La resolucion de default es de 12 bit, 750 ms para la conversion
		float Varfloat_Auxiliar_1 = (float)Var16_Auxiliar_1 / 16.0;
		PrintData(0, Varfloat_Auxiliar_1, Var8_Auxiliar_2, imprime_);
		Var8_Auxiliar_2++;
	}
	return;
}

/*--------------------  Luminosidad BH1750  ------------------------------------------------*/

bool LabMovilF::LuxBegin(uint8_t modoHB, uint8_t addr) {
	_deviceAddress[0] = addr; // Allows user to change TwoWire instance
	I2C = &Wire;
	ModeBH1750 mode_HB;
	switch (modoHB) {
	case 1:
		mode_HB = CONTINUOUS_HIGH_RES_MODE;
		break;
	case 2:
		mode_HB = CONTINUOUS_HIGH_RES_MODE_2;
		break;
	case 3:
		mode_HB = CONTINUOUS_LOW_RES_MODE;
		break;
	default:
		return false;
	}
	return (configure(mode_HB) && setMTreg(BH1750_DEFAULT_MTREG));
}

bool LabMovilF::configure(ModeBH1750 mode) {
	
	Var8_Auxiliar_1 = 5;  // Se define el resultado de la trasmicion como un resultado fuera de rango
	
	switch (mode) {		// Revisa si el modo de medicion es valido

	case CONTINUOUS_HIGH_RES_MODE:
	case CONTINUOUS_HIGH_RES_MODE_2:
	case CONTINUOUS_LOW_RES_MODE:
		// Eniva el modo de operacion al sensor
		I2C->beginTransmission(_deviceAddress[0]);
		__wire_write((uint8_t)mode);
		Var8_Auxiliar_1 = I2C->endTransmission();
		_delay_ms(10);
		break;
	default:
		// Modo de medicion invalido
		break;
	}
	// Revisa el resultado del codigo
	switch (Var8_Auxiliar_1) {
	case 0:
		BH1750_MODE = mode;
		return true;
	default:
		break;
	}
	return false;
}

bool LabMovilF::setMTreg(byte MTreg) {
	if (MTreg < BH1750_MTREG_MIN || MTreg > BH1750_MTREG_MAX) {
		return false;
	}
	Var8_Auxiliar_1 = 5;
	// Send MTreg and the current mode to the sensor
	//   High bit: 01000_MT[7,6,5]
	//    Low bit: 011_MT[4,3,2,1,0]
	I2C->beginTransmission(_deviceAddress[0]);
	__wire_write((0b01000 << 3) | (MTreg >> 5));
	Var8_Auxiliar_1 = I2C->endTransmission();
	I2C->beginTransmission(_deviceAddress[0]);
	__wire_write((0b011 << 5) | (MTreg & 0b11111));
	Var8_Auxiliar_1 = Var8_Auxiliar_1 | I2C->endTransmission();
	I2C->beginTransmission(_deviceAddress[0]);
	__wire_write(BH1750_MODE);
	Var8_Auxiliar_1 = Var8_Auxiliar_1 | I2C->endTransmission();

	// Wait a few moments to wake up
	_delay_ms(10);

	// Check result code
	switch (Var8_Auxiliar_1) {
	case 0:
		BH1750_MTreg = MTreg;
		return true;
	default:
		break;
	}

	return false;
}

void LabMovilF::luxometro(bool imprime_) {

	if (SaltarMenu) {
		MenuLab(1, 1, 0);
		delay(1);
		SaltarMenu = 0;
	}

	neo.setPixelColor(0, neo.Color(0,255, 0));
	neo.show();

	if (BH1750_MODE == UNCONFIGURED) {
		return -2.0;
	}

	// Measurement result will be stored here
	Varfloat_Auxiliar_1 = -1.0;

	// Read two bytes from the sensor, which are low and high parts of the sensor
	// value
	if (2 == I2C->requestFrom((int)_deviceAddress[0], (int)2)) {
		Var16_Auxiliar_1 = 0;
		Var16_Auxiliar_1 = __wire_read();
		Var16_Auxiliar_1 <<= 8;
		Var16_Auxiliar_1 |= __wire_read();
		Varfloat_Auxiliar_1 = Var16_Auxiliar_1;
	}


	if (Varfloat_Auxiliar_1 != -1.0) {
		// Print raw value if debug enabled

		if (BH1750_MTreg != BH1750_DEFAULT_MTREG) {
			Varfloat_Auxiliar_1 *= (float)((byte)BH1750_DEFAULT_MTREG / (float)BH1750_MTreg);
			// Print MTreg factor if debug enabled
		}
		if (BH1750_MODE == CONTINUOUS_HIGH_RES_MODE_2) {
			Varfloat_Auxiliar_1 /= 2;
		}
		// Convert raw value to lux
		Varfloat_Auxiliar_1 /= BH1750_CONV_FACTOR;
	}

	PrintData(1, Varfloat_Auxiliar_1, 0, imprime_);

}

/*--------------------  Voltaje MCP3421  ------------------------------------------------*/

void LabMovilF::SetPGA_MCP3421(uint8_t PGA ) {

	Var8_Auxiliar_1 = 16;						// Configura el mcp3421 en modo continuo

	switch (PGA) {
	case 1:
		Var8_Auxiliar_1 |= Program_PGA_Gain;	// Configura la ganacia  del mcp3421 en 1
		MultPGA = PGAGain;						//Pone el multiplicador en 1
		break;
	case 2:
		Var8_Auxiliar_1 |= Program_PGA_Gain2;	// Configura la ganacia  del mcp3421 en 2
		MultPGA = PGAGain2;						//Pone el multiplicador en 2
		break;
	case 3:
		Var8_Auxiliar_1 |= Program_PGA_Gain4;	// Configura la ganacia  del mcp3421 en 3
		MultPGA = PGAGain4;						//Pone el multiplicador en 4
		break;
	case 4:
		Var8_Auxiliar_1 |= Program_PGA_Gain8;	// Configura la ganacia  del mcp3421 en 4
		MultPGA = PGAGain8;						//Pone el multiplicador en 8
		break;
	default:
		Var8_Auxiliar_1 |= Program_PGA_Gain;	// Por default se configura la ganacia  del mcp3421 en 1
		MultPGA = PGAGain;						//Pone el multiplicador en 1 por default
		break;
	}

	Var8_Auxiliar_1 |= Program_Data_Rate4;		// Configura el mcp3421 a 18 bits de resolucion

	Wire.beginTransmission(_deviceAddress[1]);	// Inicializa la comunicacion con el mcp3421  
	Wire.write(Var8_Auxiliar_1);				// Manda la configuracion al mcp3421
	Wire.endTransmission();						// Termina la configuracion

}

void LabMovilF::Voltaje(bool imprime_) {

	if (SaltarMenu) {
		MenuLab(2, 1, 0);
		delay(1);
		SaltarMenu = 0;
	}

	neo.setPixelColor(0, neo.Color(0, 0, 255));
	neo.show();

	Varfloat_Auxiliar_1 = volt_18_bits;
	Var32_Auxiliar_2 = 0;
	int8_t Auxiliar;
	Var8_Auxiliar_2 = 1;
	while ((Var8_Auxiliar_2 == 1) || bitRead((Array_Auxiliar_1[3]), 7) == 1) {
		Var8_Auxiliar_1 = 0;
		Var8_Auxiliar_2 = 0;
		Wire.requestFrom(_deviceAddress[1], (uint8_t)4);
		while (Wire.available()) {
			Array_Auxiliar_1[Var8_Auxiliar_1] = Wire.read();
			Var8_Auxiliar_1++;
		}
	}
	Var32_Auxiliar_1 = 0;
	if (bitRead(Array_Auxiliar_1[0], 7)) {
		Var32_Auxiliar_1 = 255;
		Var32_Auxiliar_1 <<= 8;
		Array_Auxiliar_1[0] |= B11111100;
		Var32_Auxiliar_1 |= Array_Auxiliar_1[0];
		Var32_Auxiliar_1 <<= 8;
		Var32_Auxiliar_1 |= Array_Auxiliar_1[1];
		Var32_Auxiliar_1 <<= 8;
		Var32_Auxiliar_1 |= Array_Auxiliar_1[2];
		Var32_Auxiliar_1 = ~Var32_Auxiliar_1 + 1;
		Auxiliar = -1;
	}
	else {
		Var32_Auxiliar_1 = 0;
		Var32_Auxiliar_1 <<= 8;
		Var32_Auxiliar_1 = Array_Auxiliar_1[0];
		Var32_Auxiliar_1 <<= 8;
		Var32_Auxiliar_1 |= Array_Auxiliar_1[1];
		Var32_Auxiliar_1 <<= 8;
		Var32_Auxiliar_1 |= Array_Auxiliar_1[2];
		Auxiliar = 1;
	}

	PrintData(2, ((float)Var32_Auxiliar_1 * Varfloat_Auxiliar_1 * (float)Auxiliar * DiviVoltaje / (float)MultPGA), 0, imprime_);
} 

/*--------------------   Medidor de capacitancia de alta precision   ------------------------------------------------*/

void LabMovilF::Capacitancia(bool imprime_){

	if (SaltarMenu) {
		MenuLab(4, 1, 0);
		delay(1);
		SaltarMenu = 0;
	}

	neo.setPixelColor(0, neo.Color(255, 255, 0));
	neo.show();

	pinMode(PIN_IN, INPUT);
	digitalWrite(PIN_OUT, HIGH);
	Var16_Auxiliar_1 = analogRead(PIN_IN);
	digitalWrite(PIN_OUT, LOW);
	pinMode(PIN_IN, OUTPUT);
	digitalWrite(PIN_IN, LOW);

	while (millis() % 1000 != 0);

	pinMode(PIN_IN, INPUT);
	digitalWrite(PIN_OUT, HIGH);
	Var16_Auxiliar_1 = analogRead(PIN_IN);
	digitalWrite(PIN_OUT, LOW);

	if (Var16_Auxiliar_1 < 1000){  // picofaradios
		pinMode(PIN_IN, OUTPUT);
		Varfloat_Auxiliar_1 = (float)Var16_Auxiliar_1 * CapInternoGND / (float)(1023 - Var16_Auxiliar_1);
		while (micros() % 100000 != 0);
		PrintData(4, Varfloat_Auxiliar_1, 1, imprime_);
		return;
	}

	else{
		pinMode(PIN_IN, OUTPUT);
		delay(1);
		pinMode(PIN_OUT, INPUT_PULLUP);
		Var32_Auxiliar_1 = micros();
		Var32_Auxiliar_3;

		do{
			Var8_Auxiliar_1 = digitalRead(PIN_OUT);
			Var32_Auxiliar_2 = micros();
			Var32_Auxiliar_3 = Var32_Auxiliar_2 > Var32_Auxiliar_1 ? Var32_Auxiliar_2 - Var32_Auxiliar_1 : Var32_Auxiliar_1 - Var32_Auxiliar_2;
		}
		while ((Var8_Auxiliar_1 < 1) && (Var32_Auxiliar_3 < 400000L));

			pinMode(PIN_OUT, INPUT);
			Var16_Auxiliar_1 = analogRead(PIN_OUT);
			digitalWrite(PIN_IN, HIGH);
			delay((Var32_Auxiliar_3 / 1000L) * 5);
			pinMode(PIN_OUT, OUTPUT);
			digitalWrite(PIN_OUT, LOW);
			digitalWrite(PIN_IN, LOW);
			float Varfloat_Auxiliar_1 = -(float)Var32_Auxiliar_3 / Resistencia_PullUp / log(1.0 - (float)Var16_Auxiliar_1 / 1023.0);

		if (Varfloat_Auxiliar_1 > 1000.0){   //microfaradios
			while (millis() % 1000 != 0);
			PrintData(4, Varfloat_Auxiliar_1 / 1000.0, 3, imprime_);
			return;
		}

		else{                                 // Nanofaradios
			while (millis() % 1000 != 0);  
			PrintData(4, Varfloat_Auxiliar_1, 2, imprime_);
			return;
		}
	}
	
}

/*--------------------  Medidor de la resistencia con el ADS1115  ------------------------------------------------*/

void LabMovilF::Resistencia(bool imprime_) {

	if (SaltarMenu) {
		MenuLab(3, 1, 0);
		delay(1);
		SaltarMenu = 0;
	}

	uint8_t Iteracion = 50;
	neo.setPixelColor(0, neo.Color(255, 0, 255));
	neo.show();
	ads.setDataRate(RATE_ADS1115_250SPS);
	Var32_Auxiliar_1 = 0;
	Var32_Auxiliar_2 = 0;
	for (int i = 0 ; i < Iteracion; i++) {
		Var32_Auxiliar_1 += ads.readADC_SingleEnded(0);
		delayMicroseconds(50);
		Var32_Auxiliar_2 += ads.readADC_SingleEnded(1);
	}
	Varfloat_Auxiliar_1 = Var32_Auxiliar_1 / Iteracion;
	Varfloat_Auxiliar_2 = Var32_Auxiliar_2 / Iteracion;
	if (Var32_Auxiliar_2 > Var32_Auxiliar_1) {
		PrintData(7, 0, 0, imprime_);
		return;
	}

	PrintData(3, (float)(Varfloat_Auxiliar_2 * ResistenicaDivisor) / (float)(Varfloat_Auxiliar_1 - Varfloat_Auxiliar_2), 1, imprime_);
	
}

/*--------------------  ESR ATMEGA328  ------------------------------------------------*/

void LabMovilF::Medidor_ESR() {


	ESR.begin(9600);
	neo.setPixelColor(0, neo.Color(255, 255, 255));
	neo.show();

	if (SaltarMenu) {
		MenuLab(5, 1, 0);
		delay(1);
		SaltarMenu = 0;
	}

	Wire.beginTransmission(_deviceAddress[2]);	// Inicializa la comunicacion con el mcp3421  
	Wire.write(127);				// Manda la configuracion al mcp3421
	delay(10);
	Wire.write(255);				// Manda la configuracion al mcp3421
	Wire.endTransmission();

	delay(150);
	String DataESR = "\n    Analizando...\n\n";
	PrintLCD(DataESR, 1);
	DataESR = "";

	Var32_Auxiliar_1 = millis();
	Var8_Auxiliar_2 = 1;
	while ( (digitalRead(B_2)) && (millis() - Var32_Auxiliar_1 < 8000)) {

		if (ESR.available()) {    // Si llega un dato por el puerto BT se envía al monitor serial
			Var8_Auxiliar_1 = ESR.read();
			delay(2);
			while ( Var8_Auxiliar_2 ) {
				Var8_Auxiliar_1 = ESR.read();
				delay(2);
				if (Var8_Auxiliar_1 == 3) {
					Var8_Auxiliar_2 = 0;
					while (!(Var8_Auxiliar_1 == 46)) {
						Var8_Auxiliar_1 = ESR.read();
						delay(1);
					}
				}
			}
			DataESR += String((char)Var8_Auxiliar_1);
			
		}
		
	}
	neo.setPixelColor(0, neo.Color(0, 0, 0));
	neo.show();

	PrintLCD( DataESR, 0);
	ESR.end();
	while (digitalRead(B_1) && digitalRead(B_Inicio));
	return ;
}

/*--------------------  Cofiguracion de ADS1115  ------------------------------------------------*/

bool ads1115::begin(uint8_t i2c_addr, TwoWire* wire) {
	m_i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
	return m_i2c_dev->begin();
}

void ads1115::setGain(adsGain_t gain) { m_gain = gain; }


void ads1115::setDataRate(uint16_t rate) { m_dataRate = rate; }


int16_t ads1115::readADC_SingleEnded(uint8_t channel) {
	if (channel > 3) {
		return 0;
	}

	startADCReading(MUX_BY_CHANNEL[channel]);

	while (!(readRegister(ADS1X15_REG_POINTER_CONFIG) & 0x8000));

	return (int16_t) (readRegister(ADS1X15_REG_POINTER_CONVERT) >> m_bitShift);
}

float ads1115::computeVolts(int16_t counts) {
	// see data sheet Table 3
	float fsRange;
	switch (m_gain) {
	case GAIN_TWOTHIRDS:
		fsRange = 6.144f;
		break;
	case GAIN_ONE:
		fsRange = 4.096f;
		break;
	case GAIN_TWO:
		fsRange = 2.048f;
		break;
	case GAIN_FOUR:
		fsRange = 1.024f;
		break;
	case GAIN_EIGHT:
		fsRange = 0.512f;
		break;
	case GAIN_SIXTEEN:
		fsRange = 0.256f;
		break;
	default:
		fsRange = 0.0f;
	}
	return counts * (fsRange / (32768 >> m_bitShift));
}

void ads1115::startADCReading(uint16_t mux) {
	// Start with default values
	uint16_t config;    // Traditional comparator (default val)

	config |= ADS1X15_REG_CONFIG_MODE_SINGLE;
	config |= m_gain;		// Set PGA/voltage range
	config |= m_dataRate;	// Set data rate
	config |= mux;			// Set channels
	config |= ADS1X15_REG_CONFIG_OS_SINGLE;				// Set 'start single-conversion' bit

	writeRegister(ADS1X15_REG_POINTER_CONFIG, config);	// Write config register to the ADC  
	writeRegister(ADS1X15_REG_POINTER_HITHRESH, 0x8000);		// Set ALERT/RDY to RDY mode.
	writeRegister(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
}

void ads1115::writeRegister(uint8_t reg, uint16_t value) {
	buffer[0] = reg;
	buffer[1] = value >> 8;
	buffer[2] = value & 0xFF;
	m_i2c_dev->write(buffer, 3);
}

uint16_t ads1115::readRegister(uint8_t reg) {
	buffer[0] = reg;
	m_i2c_dev->write(buffer, 1);
	m_i2c_dev->read(buffer, 2);
	return ((buffer[0] << 8) | buffer[1]);
}
