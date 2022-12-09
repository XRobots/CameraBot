int thresholdStick (int pos) {
  
    // get zero centre position
    pos = pos - 512;  

    // threshold value for control sticks 
    if (pos > 100) {
      pos = pos -100;
    }
    else if (pos < -100) {
      pos = pos +100;
    }
    else {
      pos = 0;
    }

    return pos;
}



// motion filter to filter motions

float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}
