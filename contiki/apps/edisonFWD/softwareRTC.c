
typedef struct{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} myTime_t;

static myTime_t currentTime = {
    .year= 2000,
    .month = 1,
    .day = 1,
    .hours = 0,
    .minutes = 0,
    .seconds = 0,
};

// set currentTime struct
void setTime(myTime_t* newTime){
    currentTime.year    = newTime->year;
    currentTime.month   = newTime->month;
    currentTime.day     = newTime->day;
    currentTime.hours   = newTime->hours;
    currentTime.minutes = newTime->minutes;
    currentTime.seconds = newTime->seconds;
}

// advance currentTime with sec seconds
void advanceTime(uint8_t sec){
    uint8_t monthAdv = 0;
    currentTime.seconds += sec;
    while (currentTime.seconds >= 60){
        currentTime.seconds -= 60;
        currentTime.minutes += 1;
        if (currentTime.minutes == 60){
            currentTime.minutes = 0;
            currentTime.hours += 1;
            if (currentTime.hours == 24){
                currentTime.hours = 0;
                currentTime.day += 1;
                if (((currentTime.month==1)||(currentTime.month==3)||(currentTime.month==5)||(currentTime.month==7)||(currentTime.month==8)||(currentTime.month==10)||(currentTime.month==12)) && (currentTime.day == 32)){
                    monthAdv = 1;
                }
                else if (((currentTime.month==4)||(currentTime.month==6)||(currentTime.month==9)||(currentTime.month==11)) && (currentTime.day==31)){
                    monthAdv = 1;
                }
                else if (currentTime.month==2){
                    // leap year
                    if ((currentTime.year%4==0) && (currentTime.year%100 != 0)){
                        if (currentTime.day==30){
                            monthAdv = 1;
                        }
                    }
                    else{
                        if (currentTime.day==29){
                            monthAdv = 1;
                        }
                    }
                }
                if (monthAdv==1){
                    currentTime.day = 1;
                    currentTime.month += 1;
                    if (currentTime.month==13){
                        currentTime.month = 1;
                        currentTime.year += 1;
                    }
                }
            }
        }
    }
}
