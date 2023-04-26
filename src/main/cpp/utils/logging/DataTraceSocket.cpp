//====================================================================================================================================================
/// Copyright 2023 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// FRC includes

// Team 302 includes
#include <utils/logging/DataTraceSocket.h>

// Third Party Includes
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#define PORT 30200

DataTraceSocket::DataTraceSocket() : m_timer()
{
}

void DataTraceSocket::Connect(void)
{
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        printf("DataTrace Socket creation error \n");
    else
    {
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);

        // since the DataTrace server is on a laptop that is getting its IP address through DHCP
        // we need to scan a range of IP addresses starting with 10.3.2.3 to for example 10.3.2.10
        // note that 10.3.2.1 is the compbot radio and 10.3.2.2 is the compbot roborio

        char ipAddressBuffer[20] = {0};

        for (int i = 3; i < 10; i++)
        {
            // Convert IPv4 and IPv6 addresses from text to binary form

            sprintf(ipAddressBuffer, "10.3.2.%d", i);
            printf("Trying to connect to ip address %s \n", ipAddressBuffer);

            if (inet_pton(AF_INET, ipAddressBuffer, &serv_addr.sin_addr) <= 0)
            {
                printf("Invalid address or Address not supported \n");
            }

            if ((status = connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
            {
                printf("Connection Failed \n");
                Disconnect();
            }
            else
            {
                printf("Connection established with %s \n", ipAddressBuffer);
                isConnected = true;
                m_timer.Restart();
                break;
            }
        }
    }
}

void DataTraceSocket::Disconnect(void)
{
    if (client_fd >= 0)
        close(client_fd);

    isConnected = false;
    m_timer.Stop();
}

void DataTraceSocket::SendData(void)
{
    if (isConnected)
        send(client_fd, sendBuffer, strlen(sendBuffer), 0);
}
