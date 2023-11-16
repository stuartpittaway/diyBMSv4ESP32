#include "Steinhart.h"


int16_t Steinhart::ThermistorToCelcius(uint16_t BCOEFFICIENT, uint16_t RawADC, float ADCScaleMax) {

//We can calculate the  Steinhart-Hart Thermistor Equation based on the B Coefficient of the thermistor
// at 25 degrees C rating
#define NOMINAL_TEMPERATURE 25

//If we get zero its likely the ADC is connected to ground
 if (RawADC>0){
    //https://arduinodiy.wordpress.com/2015/11/10/measuring-temperature-with-ntc-the-steinhart-hart-formula/

    float steinhart = (ADCScaleMax/(float)RawADC - 1.0F);

    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
    steinhart += 1.0F / (NOMINAL_TEMPERATURE + 273.15F); // + (1/To)
    steinhart = 1.0F / steinhart; // Invert
    steinhart -= 273.15F; // convert to oC

    return (int16_t)steinhart;
 }

 return (int16_t)-999;
}

