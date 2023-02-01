
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

#ifdef __linux
#include <dirent.h>
#endif

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Filesystem.h>

//Team302 includes
#include <auton/AutonSelector.h>


using namespace std;

//---------------------------------------------------------------------
// Method: 		<<constructor>>
// Description: This creates this object and reads the auto script (CSV)
//  			files and displays a list on the dashboard.
//---------------------------------------------------------------------
bool HasError = false;
AutonSelector::AutonSelector() : m_xmlFiles(),
								 m_chrgstatchooser()

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
	autonfile += std::string ("/paths/output/");
	autonfile += GetAlianceColor();
	autonfile += GetStartPos();
	autonfile += GetNumofPiecesinauton();
	autonfile += GetParkOnChargeStation();
	return autonfile;
}

string AutonSelector::GetParkOnChargeStation()
{
	if (m_parkonchrgstat.compare("yes"))
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
	if (m_alliancecolor.compare("Red"))
	{
		return std::string("Red");
	
	}
	else //(m_alliancecolor.compare("Blue"))
	{
		return std::string("Blue");
	}
}
string AutonSelector::GetStartPos()
{
	if (m_startpos.compare("Gridwall"))
	{
	return	std::string("Wall");
	}

	else if (m_startpos.compare("Gridcoop"))
	{
	return	std::string("Coop");
	
	}
	else (m_startpos.compare("Gridhp"));
	{
		return std::string("Hp");
	}
}

string AutonSelector::GetNumofPiecesinauton()
{
	if (m_numofgamepiece.compare("1"))
	{
		return std::string("One");
	}

	else if (m_numofgamepiece.compare("2"))
	{
		return std::string("Two");
	}

	else if (m_numofgamepiece.compare("3"))
	{
		return std::string("Three");
	}
	else (m_numofgamepiece.compare("4"));
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
{
	//choose to park on charging station or not
	m_chrgstatchooser.SetDefaultOption("yes", m_parkonchrgstat);
	m_chrgstatchooser.AddOption("no", m_parkonchrgstat);
	frc::SmartDashboard::PutData("prkonchrgstat", &m_chrgstatchooser);
	
	//alliance color
	m_alliancecolorchooser.AddOption("Red", m_alliancecolor);
	m_alliancecolorchooser.AddOption("Blue", m_alliancecolor);
	frc::SmartDashboard::PutData("Alliance color", &m_alliancecolorchooser);

	//# of game pieces
	m_startposchooser.AddOption("Gridcoop",m_startpos);
	m_startposchooser.AddOption("Gridwall",m_startpos);
	m_startposchooser.AddOption("Gridhp",m_startpos);
	frc::SmartDashboard::PutData("StartPos",&m_startposchooser);

	//what you want to do in auton
	m_numofgamepiecechooser.AddOption("1",m_numofgamepiece);
	m_numofgamepiecechooser.AddOption("2",m_numofgamepiece);
	m_numofgamepiecechooser.AddOption("3",m_numofgamepiece);
	m_numofgamepiecechooser.AddOption("4",m_numofgamepiece);
	frc::SmartDashboard::PutData("NumOfpcs",&m_numofgamepiecechooser);

}