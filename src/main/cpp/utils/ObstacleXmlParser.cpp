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

//C++ Includes


//FRC Includes
#include <frc/Filesystem.h>

//Team 302 Includes
#include <utils/ObstacleXmlParser.h>
#include <utils/Logger.h>

//Third party includes
#include <pugixml/pugixml.hpp>

using namespace pugi;
using namespace std;

ObstacleXmlParser* ObstacleXmlParser::m_instance = nullptr;
ObstacleXmlParser* ObstacleXmlParser::GetInstance()
{
	if ( ObstacleXmlParser::m_instance == nullptr )
	{
        ObstacleXmlParser::m_instance = new ObstacleXmlParser();
	}
	return ObstacleXmlParser::m_instance;
}

void ObstacleXmlParser::ParseObstacles()
{
    // set the file to parse
	auto deployDir = frc::filesystem::GetDeployDirectory();
    std::string filename = deployDir + std::string("/obstacles.xml");

    string waypoint = "";
    double xCoordinate = 0.0;
    double yCoordinate = 0.0;

    bool hasError = false;

    try
    {
       // load the xml file into memory (parse it)
        xml_document doc;
        xml_parse_result result = doc.load_file(filename.c_str());

        // if it is good
        if (result)
        {
            // get the root node <robot>
            xml_node parent = doc.root();
            for (xml_node node = parent.first_child(); node; node = node.next_sibling())
            {
                // loop through the direct children of <robot> and call the appropriate parser
                for (xml_node child = node.first_child(); child; child = child.next_sibling())
                {
                    for(xml_attribute attr = child.first_attribute(); attr && !hasError; attr = attr.next_attribute())
                    {
                        if(strcmp(attr.name(), "waypoint") == 0)
                        {
                            waypoint = attr.as_string();
                        }
                        else if (strcmp(attr.name(), "xCoordinate") == 0)
                        {
                            xCoordinate = attr.as_double();
                        }
                        else if (strcmp(attr.name(), "yCoordinate") == 0)
                        {
                            yCoordinate = attr.as_double();
                        }
                        else
                        {
                            string msg = "unknown attribute ";
                            msg += attr.name();
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MotorXmlParser"), string("ParseXML"), msg );
                            hasError = true;
                        }
                    }
                }
            }
        }
        else
        {
            std::string msg = "XML [";
            msg += filename;
            msg += "] parsed with errors, attr value: [";
            msg += doc.child( "prototype" ).attribute( "attr" ).value();
            msg += "]";
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ObstacleXmlParser"), string("ParseXML (1) "), msg );

            msg = "Error description: ";
            msg += result.description();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ObstacleXmlParser"), string("ParseXML (2) "), msg );

            msg = "Error offset: ";
            msg += result.offset;
            msg += " error at ...";
            msg += filename;
            msg += result.offset;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ObstacleXmlParser"), string("ParseXML (3) "), msg );
        }
    }
    catch(const std::exception& e)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ObstacleXmlParser"), string("ParseXML"), string("Error thrown while parsing obstacles.xml") );
    }

    if(!hasError)
    {

    }
}
