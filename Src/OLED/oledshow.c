#include "../include/OLED/oledshow.h"
void oled_show(int way)
{

    oled_show_string(0,0,"Mode:",2);
    if(way==1)
    {
        oled_show_string(40,0,"DMP",2);
    }
    else if(way==2)
    {
    oled_show_string(40,0,"Kalman",2);
    }
    else if(way==3)
    {
        oled_show_string(40,0,"Complement",2);
    }
    oled_show_string(0,2,"Angle:",2);
    OLED_ShowAllFloat(40,2,ShowDataCo.Angle_Balance,2,16,1);
    oled_show_string(0,4,"V:",2);
    OLED_ShowAllFloat(12,4,ShowDataCo.Voltage,2,16,0);
    oled_show_string(60,4,"T:",2);
    OLED_ShowAllFloat(72,4,ShowDataCo.Temperature,2,16,0);
    oled_show_string(0,6,"L:",2);
    OLED_ShowAllFloat(12,6,ShowDataCo.LeftWheel_Velocity,2,16,1);
    oled_show_string(60,6,"R:",2);
    OLED_ShowAllFloat(72,6,ShowDataCo.RightWheel_Velocity,2,16,1);
}

// // SHOW::SHOW()
// // {
// //     MX_I2C2_Init();
// //     oled_init();
// // }

