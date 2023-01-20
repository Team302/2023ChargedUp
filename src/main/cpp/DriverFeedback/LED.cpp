
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
            return {255,255,0};   

        case PURPLE:
            return {255,0,255};

        case AZUL:
            return {29,93,236};   
        }
    }

   void LED::LedsOff(){
        for (int i = 0; i < kLength; i++){
            m_ledBuffer[i].SetRGB(0,0,0);
        }
        m_led->SetData(m_ledBuffer);

   }
   int colorLoop = 0;
   std::array<LED::Colors,(int)LED::MAX_STATE> colorArray;
   int timer = 0;

   void LED::ColorTest(){
        for (int i = 0; i < kLength; i++){
            auto color = getColorValues(YELLOW);
            m_ledBuffer[i].SetRGB(color[0],color[1],color[2]);
            // if (color== getColorValues(BLUE)){
            //     auto color = getColorValues(RED);
            // }
            // else if (color == getColorValues(RED)){
            //     auto color = getColorValues(BLUE);
            // }
        }
    m_led->SetData(m_ledBuffer);
    }
    // if(timer<5){
    //     auto color = getColorValues(colorArray[colorLoop]);
    //     for (int i = 0; i < kLength; i++){
    //         m_ledBuffer[i].SetRGB(color[0],color[1],color[2]);
    //     }
    //     colorLoop+=colorLoop<LED::MAX_STATE ? 1 : -colorLoop;
    
    //     m_led->SetData(m_ledBuffer);
    // }

    //timer++;
   void LED::UpdateLEDS(){
        ColorTest();
    }
