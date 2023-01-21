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
#include <utils/Obstacles.h>

//Third party includes
#include <pugixml/pugixml.hpp>

using namespace pugi;

void Obstacles::ParseObstacles()
{
    // set the file to parse
	auto deployDir = frc::filesystem::GetDeployDirectory();
    std::string filename = deployDir + std::string("/obstacles.xml");

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
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML (1) "), msg );

            msg = "Error description: ";
            msg += result.description();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML (2) "), msg );

            msg = "Error offset: ";
            msg += result.offset;
            msg += " error at ...";
            msg += filename;
            msg += result.offset;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML (3) "), msg );
        }
    }
    catch(const std::exception& e)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML"), string("Error thrown while parsing robot.xml") );
    }
}
