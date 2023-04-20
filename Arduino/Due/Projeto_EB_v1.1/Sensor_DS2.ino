/*
 * File:   Sensor_DS2.c
 * Author: Gr Eletr Automotiva
 *
 * Created on 24 de Abril de 2022, 19:29
 */

#include "Sensor_DS2.h"



float converte_DS2(int adc_value){
// UA = UV(0.003*p+0.12)
//[(UA/UV)-0.12]/0.003
//333*[(UA/UV)-0.12 ]
//UA = byte_valeu*(5V/1024))
//UV =~ 5V
//333*[(byte_value/1024)-0.12 ]
//byte_value*0.325520833 - 40
//Pres�o em bar
    
    float pressao = ((adc_value * 0.32)-40);
    
    if(pressao < 0){
       return 0; 
    }else {
        return pressao;   
    }
}
