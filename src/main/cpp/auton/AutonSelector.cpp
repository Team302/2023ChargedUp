
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
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

// co-Author: notcharlie, creator of dumb code / copy paster of better code

//Includes
#include <string>
#include <vector>
#include <sys/stat.h>
#include <fstream>

#ifdef __linux
#include <dirent.h>
#endif

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Filesystem.h>

//Team302 includes
#include <auton/AutonSelector.h>
#include <utils/Logger.h>
#include <utils/FMSData.h>
using namespace std;

//---------------------------------------------------------------------
// Method: 		<<constructor>>
// Description: This creates this object and reads the auto script (CSV)
//  			files and displays a list on the dashboard.
//---------------------------------------------------------------------
bool HasError = false;
AutonSelector::AutonSelector() : m_xmlFiles(),
								 m_chrgstatchooser() //todo remove

{
	FindXMLFileNames();
	PutChoicesOnDashboard();
}

//---------------------------------------------------------------------
// Method: 		FindXMLFileNames
// Description: This builds up a list of CSV files in the directory and
//				stores them in the m_csvFiles attribute.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::FindXMLFileNames()
{

}

string AutonSelector::GetSelectedAutoFile()
{
	std::string autonfile(frc::filesystem::GetDeployDirectory());
	autonfile += std::string ("/auton/");
	autonfile += GetAlianceColor();
	autonfile += GetStartPos();
	autonfile += GetNumofPiecesinauton();
	autonfile += GetParkOnChargeStation();
	autonfile += std::string (".xml");

	if (FileExists(autonfile) ==false)
	{
	autonfile=frc::filesystem::GetDeployDirectory();
	autonfile += std::string ("/auton/");
	autonfile += GetAlianceColor();
	autonfile += ("COOPThreeP.xml");
	}
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("xml file"), std::string("interesting string Idea"),autonfile);
	return autonfile;
}

	bool  AutonSelector::FileExists(const std::string& name) 
		{
	ifstream f(name.c_str());
	return f.good();
		}
string AutonSelector::GetParkOnChargeStation()
{
	if (m_chrgstatchooser.GetSelected()=="yes")
{
	return std::string("P");
}
 	else//(m_parkonchrgstat.compare("no"));
{
	return std::string("Np");
}

}

string AutonSelector::GetAlianceColor()
{
	if (FMSData::GetInstance()->GetAllianceColor()==frc::DriverStation::Alliance::kRed)
	{
		return std::string("Red");
	
	}
	else if (FMSData::GetInstance()->GetAllianceColor()==frc::DriverStation::Alliance::kBlue)
	{
		return std::string("Blue");
	}
	else
	{
		Benisgoodatspelling = (false);
		HasError =(true);
	}
}
string AutonSelector::GetStartPos()
{
	if (m_startposchooser.GetSelected()=="Gridwall")
	{
	return	std::string("Wall");
	}

	else if (m_startposchooser.GetSelected()=="Gridcoop")
	{
	return	std::string("COOP");
	
	}
	else 
	{
		return std::string("HP");
	}
}

string AutonSelector::GetNumofPiecesinauton()
{
	if (m_numofgamepiecechooser.GetSelected()=="1")
	{
		return std::string("One");
	}

	else if (m_numofgamepiecechooser.GetSelected()=="2")
	{
		return std::string("Two");
	}

	else if (m_numofgamepiecechooser.GetSelected()=="3")
	{
		return std::string("Three");
	}
	else
	{
		return std::string("Four");
	}
}

// color strt pos num of gamepiece 
//---------------------------------------------------------------------
// Method: 		PutChoicesOnDashboard
// Description: This puts the list of files in the m_csvFiles attribute
//				up on the dashboard for selection.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::PutChoicesOnDashboard()
{	//dashboard wont change prkonchrgstat in .xml file
	//choose to park on charging station or not
	m_chrgstatchooser.AddOption("yes", "yes");
	m_chrgstatchooser.AddOption("no", "no");
	frc::SmartDashboard::PutData("prkonchrgstat", &m_chrgstatchooser);

	//# of game pieces
	m_startposchooser.AddOption("Gridcoop","Gridcoop");
	m_startposchooser.AddOption("Gridwall","Gridwall");
	m_startposchooser.AddOption("Gridhp","Gridhp");
	frc::SmartDashboard::PutData("StartPos",&m_startposchooser);

	//what you want to do in auton
	m_numofgamepiecechooser.AddOption("1","1");
	m_numofgamepiecechooser.AddOption("2","2");
	m_numofgamepiecechooser.AddOption("3","3");
	m_numofgamepiecechooser.AddOption("4","4");
	frc::SmartDashboard::PutData("NumOfpcs",&m_numofgamepiecechooser);

}