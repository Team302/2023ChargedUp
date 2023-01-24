
#include <DriverFeedback/LED.h>

    LED::LED(int PWMport){
        m_led = new frc::AddressableLED(PWMport);
        m_led->SetLength(kLength);
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
            return {204,204,0};   

        case PURPLE:
            return {255,0,255};

        case AZUL:
            return {0,255,255};   
        }
    }

   void LED::LedsOff(){
        for (int i = 0; i < kLength; i++){
            m_ledBuffer[i].SetRGB(0,0,0);
        }
        m_led->SetData(m_ledBuffer);

   }
   int loop = -1;
   bool done = false;
   int timer = 0;
   void LED::ChangingChaserPattern(){
    if(timer>10){
        auto color = getColorValues(PURPLE);
        
        if(loop<kLength){
            loop++;
        }else{
            done = true;
            loop = 0;
        }
        if(!done){
            m_ledBuffer[loop].SetRGB(color[0],color[1],color[2]);
        }else{
            LedsOff();
            done = false;
        }
        timer =0;
    }
    timer++;
   }

   void LED::HalfAndHalfPattern(){
            auto color = getColorValues(PURPLE);
            for (int i = 0; i < kLength/2; i++){
                m_ledBuffer[i].SetRGB(color[0],color[1],color[2]);
        }
         color = getColorValues(YELLOW);
            for (int i = kLength-1; i > kLength/2; i--){
                m_ledBuffer[i].SetRGB(color[0],color[1],color[2]);
            }
        m_led->SetData(m_ledBuffer);
    }


   void LED::UpdateLEDS(){
        ChangingChaserPattern();
        m_led->SetData(m_ledBuffer);
    }
