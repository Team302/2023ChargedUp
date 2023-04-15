
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
#include <span>

void LEDStates::HalfAndHalfPattern(DragonLeds::Colors currentGamePieceColor, DragonLeds::Colors nextGamePieceColor)
{
    int topIndividualLEDStripLength = 27;
    int botIndividualLEDStripLength = 34;
    int startingPositionIndex = 0;
    int endingPositionIndex = 0;

    int currentGamePieceColorValues[3] = {m_LEDstring->getColorValues(currentGamePieceColor)[0],
                                          m_LEDstring->getColorValues(currentGamePieceColor)[1],
                                          m_LEDstring->getColorValues(currentGamePieceColor)[2]};

    int nextGamePieceColorValues[3] = {m_LEDstring->getColorValues(nextGamePieceColor)[0],
                                       m_LEDstring->getColorValues(nextGamePieceColor)[1],
                                       m_LEDstring->getColorValues(nextGamePieceColor)[2]};

    // sets the whole first bottom LED strip to the next Game Piece color
    endingPositionIndex += botIndividualLEDStripLength;
    for (int i = startingPositionIndex; i < endingPositionIndex - 1; i++)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(nextGamePieceColorValues[0], nextGamePieceColorValues[1], nextGamePieceColorValues[2]);
    }
    for (int i = startingPositionIndex; i < (endingPositionIndex / 2) - 1; i++)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(currentGamePieceColorValues[0], currentGamePieceColorValues[1], currentGamePieceColorValues[2]);
    }
    // overrides half of the bottom first LED strip to the current Game Piece color.
    //
    //
    startingPositionIndex += endingPositionIndex;
    endingPositionIndex += topIndividualLEDStripLength;

    for (int i = startingPositionIndex; i < endingPositionIndex - 1; i++)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(nextGamePieceColorValues[0], nextGamePieceColorValues[1], nextGamePieceColorValues[2]);
    }
    for (int i = startingPositionIndex; i < endingPositionIndex - (topIndividualLEDStripLength / 2) - 1; i++)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(currentGamePieceColorValues[0], currentGamePieceColorValues[1], currentGamePieceColorValues[2]);
    }

    startingPositionIndex += topIndividualLEDStripLength;
    endingPositionIndex += topIndividualLEDStripLength;

    for (int i = startingPositionIndex; i < endingPositionIndex - 1; i++)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(nextGamePieceColorValues[0], nextGamePieceColorValues[1], nextGamePieceColorValues[2]);
    }
    for (int i = startingPositionIndex + topIndividualLEDStripLength; i > endingPositionIndex - (topIndividualLEDStripLength / 2) - 1; i--)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(currentGamePieceColorValues[0], currentGamePieceColorValues[1], currentGamePieceColorValues[2]);
    }

    startingPositionIndex += topIndividualLEDStripLength;
    endingPositionIndex += botIndividualLEDStripLength;

    for (int i = startingPositionIndex; i < endingPositionIndex - 1; i++)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(nextGamePieceColorValues[0], nextGamePieceColorValues[1], nextGamePieceColorValues[2]);
    }
    for (int i = startingPositionIndex + botIndividualLEDStripLength; i > endingPositionIndex - (botIndividualLEDStripLength / 2) - 1; i--)
    {
        m_LEDstring->m_ledBuffer[i].SetRGB(currentGamePieceColorValues[0], currentGamePieceColorValues[1], currentGamePieceColorValues[2]);
    }
}
void LEDStates::BlinkingPattern(DragonLeds::Colors c)
{
    if (timer > 2 * blinkPatternPeriod)
        timer = 0;

    int blinkState = (timer / blinkPatternPeriod) % 2;

    if (blinkState == 0)
        m_LEDstring->setBufferAllLEDsColor(m_LEDstring->getColorValues(c));
    else
        m_LEDstring->setBufferAllLEDsBlack();

    m_LEDstring->commitLedData();

    timer++;
}

void LEDStates::SolidColorPattern(DragonLeds::Colors c)
{
    m_LEDstring->setBufferAllLEDsColor(m_LEDstring->getColorValues(c));
    m_LEDstring->commitLedData();
}

void LEDStates::AlternatingColorBlinkingPattern(DragonLeds::Colors c)
{
    AlternatingColorBlinkingPattern(c, m_LEDstring->BLACK);
}

void LEDStates::AlternatingColorBlinkingPattern(DragonLeds::Colors c1, DragonLeds::Colors c2)
{
    if (timer > 2 * alternatingColorBlinkPatternPeriod)
        timer = 0;

    int blinkState = (timer / blinkPatternPeriod) % 2;

    if (blinkState == 0)
        m_LEDstring->setBufferAllLEDsAlternatingColor(m_LEDstring->getColorValues(c1), m_LEDstring->getColorValues(c2));
    else
        m_LEDstring->setBufferAllLEDsAlternatingColor(m_LEDstring->getColorValues(c2), m_LEDstring->getColorValues(c1));

    m_LEDstring->commitLedData();

    timer++;
}

void LEDStates::ClosingInChaserPattern(DragonLeds::Colors c)
{
    if (timer == 7)
    {
        int halfLength = (m_LEDstring->m_ledBuffer.size() - 1) / 2;
        loopThroughIndividualLEDs += loopThroughIndividualLEDs < halfLength ? 1 : -loopThroughIndividualLEDs;
        int loopout = (m_LEDstring->m_ledBuffer.size() - 1) - loopThroughIndividualLEDs;
        auto color = colorLoop >= 0 ? m_LEDstring->getColorValues(c) : m_LEDstring->getColorValues(m_LEDstring->BLACK);
        colorLoop += colorLoop < halfLength ? 1 : -((colorLoop * 2) + 1);
        m_LEDstring->m_ledBuffer[loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);
        m_LEDstring->m_ledBuffer[loopout].SetRGB(color[0], color[1], color[2]);

        m_LEDstring->commitLedData();
        timer = 0;
    }
    timer++;
}

void LEDStates::ChaserPattern(DragonLeds::Colors c)
{
    loopThroughIndividualLEDs += loopThroughIndividualLEDs < m_LEDstring->m_ledBuffer.size() - 1 ? 1 : -loopThroughIndividualLEDs;

    auto color = colorLoop >= 0 ? m_LEDstring->getColorValues(c) : m_LEDstring->getColorValues(DragonLeds::BLACK);

    colorLoop += colorLoop < m_LEDstring->m_ledBuffer.size() - 1 ? 1 : -((colorLoop * 2) + 1);

    m_LEDstring->m_ledBuffer[loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);

    m_LEDstring->commitLedData();

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

void LEDStates::setLEDsOff()
{
    m_LEDstring->setOff();
}

void LEDStates::setLEDsOn()
{
    m_LEDstring->setOn();
}