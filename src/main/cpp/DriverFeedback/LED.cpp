
#include <DriverFeedback/LED.h>

    LED::LED(int PWMport){
        m_led = new frc::AddressableLED(PWMport);
        m_led->SetData(m_ledBuffer);
        m_led->Start();
    }

    std::array<int, 3> LED::getColorValues(Colors c){
        switch (c)
        {
        case RED:
            return {255,0,0};
        
        case GREEN:
            return {0,255,0};

        case BLUE:
            return {0,0,255};

        case YELLOW:
            return {255,255,0};   

        case PURPLE:
            return {255,0,255};

        case AZUL:
            return {29,93,236};   
        }
        return {0,0,0};
    }


   void LED::UpdateLEDS(){
        for (int i = 0; i < kLength; i++)
        {
           auto rgb = getColorValues(BLUE); 
           m_ledBuffer[i].SetRGB(rgb[0],rgb[1],rgb[2]);
        }
        
        m_led->SetData(m_ledBuffer);
    }
