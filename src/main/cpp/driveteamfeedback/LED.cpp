
//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include <driveteamfeedback/LED.h>
#include <span>

void LED::Initialize(int PWMport, int numLeds)
{
    if (m_addressibleLeds == nullptr)
    {
        m_addressibleLeds = new frc::AddressableLED(PWMport);
    }
    m_ledBuffer.resize(numLeds);
    m_addressibleLeds->SetLength(numLeds);

    commitLedData();
    m_addressibleLeds->Start();
}
LED *LED::m_instance = nullptr;

LED::LED()
{
}

LED *LED::GetInstance()
{
    if (LED::m_instance == nullptr)
    {
        LED::m_instance = new LED();
    }
    return LED::m_instance;
}

std::array<int, 3> LED::getColorValues(Colors c)
{
    switch (c)
    {
    case RED:
        return {255, 0, 0};
    case GREEN:
        return {0, 255, 0};
    case BLUE:
        return {0, 0, 255};
    case YELLOW:
        return {255, 160, 0};
    case PURPLE:
        return {75, 0, 130};
    case AZUL:
        return {0, 255, 255};
    case WHITE:
        return {255, 255, 180};
    case BLACK:
        return {0, 0, 0};
    default:
        return {0, 0, 0};
    }
}

void LED::commitLedData()
{
    std::span ledSpan{m_ledBuffer.data(), m_ledBuffer.size()};
    m_addressibleLeds->SetData(ledSpan);
}