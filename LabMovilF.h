#ifndef _LABMOVILF_H_
#define _LABMOVILF_H_

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>

/*=========================================================================
    NeoPixel SK6812
    -----------------------------------------------------------------------*/

#define Pin_neo 8
#define Num_leds 1
#define Brillo_neo 200

/*=========================================================================
    MCP3421 Lectura del voltaje.
    -----------------------------------------------------------------------*/
#define ID_MCP3421  0x68
#define volt_12_bits 0.001F
#define volt_14_bits 0.00025F
#define volt_16_bits 0.0000625F
#define volt_18_bits 0.000015625F
#define PGAGain 1
#define PGAGain2 2
#define PGAGain4 4
#define PGAGain8 8
#define Program_PGA_Gain 0
#define Program_PGA_Gain2 1
#define Program_PGA_Gain4 2
#define Program_PGA_Gain8 3
#define Program_Data_Rate 0
#define Program_Data_Rate2 4
#define Program_Data_Rate3 8
#define Program_Data_Rate4 12


/*=========================================================================
    BH1750 sensor de luminicencia
    -----------------------------------------------------------------------*/

#define BH1750_DEFAULT_ADDRESS 0x23

typedef enum ModeBH1750 {
    // same as Power Down
    UNCONFIGURED = 0,
    // Measurement at 1 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_HIGH_RES_MODE = 0x10,
    // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_HIGH_RES_MODE_2 = 0x11,
    // Measurement at 4 lux resolution. Measurement time is approx 16ms.
    CONTINUOUS_LOW_RES_MODE = 0x13
};

#define BH1750_POWER_DOWN 0x00  // Estado de bajo comsumo
#define BH1750_POWER_ON 0x01    // En estado de espera
#define BH1750_RESET 0x07       // Reinicia el registro de valores

#define BH1750_DEFAULT_MTREG 69 // Valores del registro MTResg
#define BH1750_MTREG_MIN 31
#define BH1750_MTREG_MAX 254



  /*=========================================================================
      ADS1115
      -----------------------------------------------------------------------*/


#define ADS1115_ADDRESS (0x48)

#define ADS1X15_REG_POINTER_CONVERT (0x00)   ///< Conversion
#define ADS1X15_REG_POINTER_CONFIG (0x01)    ///< Configuration
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02) ///< Low threshold
#define ADS1X15_REG_POINTER_HITHRESH (0x03)  ///< High threshold

#define ADS1X15_REG_CONFIG_OS_SINGLE (0x8000) ///< Write: Set to start a single-conversion

#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3

constexpr uint16_t MUX_BY_CHANNEL[] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0, ///< Single-ended AIN0
    ADS1X15_REG_CONFIG_MUX_SINGLE_1, ///< Single-ended AIN1
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ///< Single-ended AIN2
    ADS1X15_REG_CONFIG_MUX_SINGLE_3  ///< Single-ended AIN3
};                                   ///< MUX config by channel

#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)   ///< PGA Mask
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000) ///< +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200) ///< +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600) ///< +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800) ///< +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00) ///< +/-0.256V range = Gain 16

#define ADS1X15_REG_CONFIG_MODE_SINGLE (0x0100) ///< Power-down single-shot mode (default)


typedef enum {
    GAIN_TWOTHIRDS = ADS1X15_REG_CONFIG_PGA_6_144V,
    GAIN_ONE = ADS1X15_REG_CONFIG_PGA_4_096V,
    GAIN_TWO = ADS1X15_REG_CONFIG_PGA_2_048V,
    GAIN_FOUR = ADS1X15_REG_CONFIG_PGA_1_024V,
    GAIN_EIGHT = ADS1X15_REG_CONFIG_PGA_0_512V,
    GAIN_SIXTEEN = ADS1X15_REG_CONFIG_PGA_0_256V
} adsGain_t;

#define RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
#define RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
#define RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
#define RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
#define RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
#define RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
#define RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
#define RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second

/*============= =====================*/


#define B_Inicio      2       // Boton 1
#define B_Modo        3       // Boton 2
#define B_1         4       // Boton 3
#define B_2         A3      // Boton 4
#define PIN_OUT     A0
#define PIN_IN      A1
//#define ADC_Max_Val 1023L
#define TriggerPin  12
#define EchoPin     13
#define LaserPin    5

class ads1115
{
protected:
    Adafruit_I2CDevice* m_i2c_dev; ///< I2C bus device
    uint8_t m_bitShift = 0;            ///< bit shift amount
    adsGain_t m_gain = GAIN_TWOTHIRDS;              ///< ADC gain   /* +/- 6.144V range (limited to VDD +0.3V max!) */
    uint16_t m_dataRate = RATE_ADS1115_128SPS;           ///< Data rate
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    void startADCReading(uint16_t mux);
    uint8_t buffer[3];
 
public:

    bool begin(uint8_t i2c_addr = ADS1115_ADDRESS, TwoWire* wire = &Wire);
    void startComparator_SingleEnded(uint8_t channel, int16_t threshold);
    int16_t readADC_SingleEnded(uint8_t channel);
    float computeVolts(int16_t counts);
    void setGain(adsGain_t gain);
    void setDataRate(uint16_t rate);

};


class LabMovilF {
private:

    //-------------BH1750------------------------------//

    byte BH1750_MTreg = (byte)BH1750_DEFAULT_MTREG; // Factor de correccion para calcular los lux, valor tipico de 1.2,
    // puede tener un rango entre 0.96 a 1.44. 
    const float BH1750_CONV_FACTOR = 1.2;
    ModeBH1750 BH1750_MODE = UNCONFIGURED;
    float readLightLevel();
    bool configure(ModeBH1750 mode);
    bool setMTreg(byte MTreg);


    ///----MPC3421------///

    uint8_t MultPGA = PGAGain;


    ///----Comunes-----////

    uint16_t ResistenicaDivisor = 9997;
    uint32_t    TiempoImpresionMenu = 0;
    float Resistencia_PullUp = 38;
    float CapInternoGND = 24.48;
    float DiviVoltaje = (24045.0 + 998.8) / 998.8;
    float SensibilidadACS712 = 0.066;
    float VelocidadDistintoMedio = 0.017014;
    ads1115 ads; 
    
    uint8_t _deviceAddress[5] = { 0x23 , 0x68 , 0x3C,0,0}; //(BH1750, MCP3421, PCF8574,--,--)
    TwoWire* I2C;
    

public:

	LabMovilF();
    void LabBegin( bool SaltarMenu_ = 0);
    void PrintData(uint8_t DataModeSelect, float DataSensor, uint8_t NumData = 0, bool imprime_ = 1);
    void PrintLCD(String MensajeEnviado, byte tipo = 1);
    uint8_t MenuLab(uint8_t MenuVal = 126, bool MenuEstado_ = 1, bool ModoEstado_ = 1);
    float ValorSensor() { return ResultadoDeLaLectura; }
    ///----Resistencis////Capacitancia----////
    void Capacitancia(bool imprime_ = 1);
    void Resistencia(bool imprime_ = 1);
    void SetResistenciaPullUp(float Resistencia_PullUp_ ){ Resistencia_PullUp = Resistencia_PullUp_;}
    void SetCapInterno( float CapInternoGND_){ CapInternoGND = CapInternoGND_;}
    ///----DS18B20-----///
    void TempExterna(bool imprime_ = 1);
    ///----HB1750-----///
    void luxometro(bool imprime_ = 1);
    bool LuxBegin(uint8_t modoHB = 1, uint8_t addr = BH1750_DEFAULT_ADDRESS);
    ///----MCP3421----///
    void MCP3421begin(int _ID_MCP3421 = ID_MCP3421) { _deviceAddress[1] = _ID_MCP3421; }
    void SetPGA_MCP3421(uint8_t PGA = 1);
    void Voltaje(bool imprime_ = 1);
    void SetVoltaje(float Divi_Res1, float Divi_Res2) { DiviVoltaje = (Divi_Res2 + Divi_Res1) / Divi_Res1; }
    ///----     Medidor ESR     ----///
    void Medidor_ESR();
    ///----     PCF8574     ----///
    void PCF8574Begin(uint8_t _ID_PCF8574 = 0x3C) { _deviceAddress[2] = _ID_PCF8574; }
    ///----     SR-HC04     ----///
    void SetDistancia(float VelocidadDistintoMedio_) { VelocidadDistintoMedio = VelocidadDistintoMedio_; }
    void DistanciaA(bool imprime_ = 1);
    ///----     ACS712     ----///
    void CorrienteD(bool imprime_ = 1);
    void SetCorrienteD(float SensibilidadACS712_) { SensibilidadACS712 = SensibilidadACS712_; }

	
protected:
    
    bool        SaltarMenu;
    bool        Inicia_modo_;
    uint8_t     Array_Auxiliar_1[8];
    uint8_t     Array_Auxiliar_2[9];
    uint8_t     Var8_Auxiliar_1;
    uint8_t     Var8_Auxiliar_2;
    uint16_t    Var16_Auxiliar_1;
    uint32_t    Var32_Auxiliar_1;
    uint32_t    Var32_Auxiliar_2;
    uint32_t    Var32_Auxiliar_3;
    float       Varfloat_Auxiliar_1;
    float       Varfloat_Auxiliar_2;
    float       ResultadoDeLaLectura;
    char        Car_Auxliar;

};


#endif
