//Function runs during the setup function on the main file. Initiliazes the BMP388
//Evan Grilley - 12/28/2020
void bmpSetup(){
  if (!bmp.begin_I2C()) {
    //error - could not find sensor  
  }

  // Set up oversampling and filder initialization - Placeholder values
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

float altitued(){
  if (! bmp.performReading()) {
    //error - could not perform reading
    return;
  }
  return bmp.readAltitude(seaLevelPressure);
  
}
