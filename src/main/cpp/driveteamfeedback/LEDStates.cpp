
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

#include <driveteamfeedback/LEDStates.h>

void LEDStates::BlinkingPattern(LED::Colors c)
{

    if (timer == 10)
    {
        colorLoop += colorLoop < 1 ? 1 : -colorLoop;
        auto color = colorLoop >= 1 ? m_LED->getColorValues(c) : m_LED->getColorValues(m_LED->BLACK);
        for (int i = 0; i < m_LED->kLength; i++)
        {
            m_LED->m_ledBuffer[i].SetRGB(color[0], color[1], color[2]);
        }
        std::span thisspan{m_LED->m_ledBuffer.data(), m_LED->m_ledBuffer.size()};
        m_LED->m_led->SetData(thisspan);
        timer = 0;
    }
    timer++;
}
void LEDStates::SolidColorPattern(LED::Colors c)
{
    auto color = m_LED->getColorValues(c);
    for (int i = 0; i < m_LED->kLength; i++)
    {
        m_LED->m_ledBuffer[i].SetRGB(color[0], color[1], color[2]);
    }
    std::span thisspan{m_LED->m_ledBuffer.data(), m_LED->m_ledBuffer.size()};
    m_LED->m_led->SetData(thisspan);
}
void LEDStates::ChaserPattern(LED::Colors c)
{
    loopThroughIndividualLEDs += loopThroughIndividualLEDs < m_LED->kLength - 1 ? 1 : -loopThroughIndividualLEDs;

    auto color = colorLoop >= 0 ? m_LED->getColorValues(c) : m_LED->getColorValues(LED::BLACK);

    colorLoop += colorLoop < m_LED->kLength - 1 ? 1 : -((colorLoop * 2) + 1);

    m_LED->m_ledBuffer[loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);
    std::span thisspan{m_LED->m_ledBuffer.data(), m_LED->m_ledBuffer.size()};
    m_LED->m_led->SetData(thisspan);
    timer = 0;
}
void LEDStates::AlternatingBlinkingPattern(LED::Colors c)
{
    if (timer >= 10)
    {
        auto currentColor1 = colorLoop == 0 ? m_LED->getColorValues(c) : m_LED->getColorValues(m_LED->BLACK);
        auto currentColor2 = colorLoop == 1 ? m_LED->getColorValues(c) : m_LED->getColorValues(m_LED->BLACK);

        colorLoop += colorLoop < 1 ? 1 : -colorLoop;

        for (int i = 0; i < m_LED->kLength; i++)
        {
            m_LED->m_ledBuffer[i].SetRGB(currentColor1[0], currentColor1[1], currentColor1[2]);
        }

        for (int i = 0; i < m_LED->kLength / 2 + 1; i++)
        {
            m_LED->m_ledBuffer[i * 2].SetRGB(currentColor2[0], currentColor2[1], currentColor2[2]);
        }
        std::span thisspan{m_LED->m_ledBuffer.data(), m_LED->m_ledBuffer.size()};
        m_LED->m_led->SetData(thisspan);
        timer = 0;
    }
    timer++;
}
void LEDStates::AlternatingBlinkingPattern(LED::Colors c1, LED::Colors c2)
{
    if (timer >= 3)
    {
        auto currentColor1 = colorLoop == 0 ? m_LED->getColorValues(c1) : m_LED->getColorValues(c2);
        auto currentColor2 = colorLoop == 1 ? m_LED->getColorValues(c1) : m_LED->getColorValues(c2);

        colorLoop += colorLoop < 1 ? 1 : -colorLoop;

        for (int i = 0; i < m_LED->kLength; i++)
        {
            m_LED->m_ledBuffer[i].SetRGB(currentColor1[0], currentColor1[1], currentColor1[2]);
        }

        for (int i = 0; i < m_LED->kLength / 2 + 1; i++)
        {
            m_LED->m_ledBuffer[i * 2].SetRGB(currentColor2[0], currentColor2[1], currentColor2[2]);
        }
        std::span thisspan{m_LED->m_ledBuffer.data(), m_LED->m_ledBuffer.size()};
        m_LED->m_led->SetData(thisspan);
        timer = 0;
    }
    timer++;
}
void LEDStates::ClosingInChaserPattern(LED::Colors c)
{
    int halfLength = (m_LED->kLength - 1) / 2;
    loopThroughIndividualLEDs += loopThroughIndividualLEDs < halfLength ? 1 : -loopThroughIndividualLEDs;
    int loopout = (m_LED->kLength - 1) - loopThroughIndividualLEDs;
    auto color = colorLoop >= 0 ? m_LED->getColorValues(c) : m_LED->getColorValues(m_LED->BLACK);
    colorLoop += colorLoop < halfLength ? 1 : -((colorLoop * 2) + 1);
    m_LED->m_ledBuffer[loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);
    m_LED->m_ledBuffer[loopout].SetRGB(color[0], color[1], color[2]);

    std::span thisspan{m_LED->m_ledBuffer.data(), m_LED->m_ledBuffer.size()};
    m_LED->m_led->SetData(thisspan);
    timer = 0;
}
void LEDStates::ResetVariables()
{
    loopThroughIndividualLEDs = -1;
    colorLoop = 0;
    timer = 0;
}

LEDStates *LEDStates::m_instance = nullptr;

LEDStates *LEDStates::GetInstance()
{
    if (LEDStates::m_instance == nullptr)
    {
        LEDStates::m_instance = new LEDStates();
    }
    return LEDStates::m_instance;
}

void LEDStates::LEDsOff()
{
    for (int i = 0; i < m_LED->kLength; i++)
    {
        m_LED->m_ledBuffer[i].SetRGB(0, 0, 0);
    }
    std::span thisspan{m_LED->m_ledBuffer.data(), m_LED->m_ledBuffer.size()};
    m_LED->m_led->SetData(thisspan);
}