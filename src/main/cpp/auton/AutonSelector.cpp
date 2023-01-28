
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
// Method: 		GetSelectedAutoFile
// Description: This returns the selected auton file to run.  If it
//  			returns "Do Nothing", it is indicating no auton should
//				be run.
// Returns:		std::string			auton file to run
//---------------------------------------------------------------------
std::string AutonSelector::GetSelectedAutoFile()
{
	return m_chrgstatchooser.GetSelected();
}

//---------------------------------------------------------------------
// Method: 		FindXMLFileNames
// Description: This builds up a list of CSV files in the directory and
//				stores them in the m_csvFiles attribute.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::FindXMLFileNames()
{
#ifdef __linux__
	//struct dirent* files;

	auto deployDir = frc::filesystem::GetDeployDirectory();
	auto autonDir = deployDir + "/auton/";
	DIR* directory = opendir(autonDir.c_str());

	if (directory != nullptr)
	{
		while (true)
		{
			auto files = readdir(directory);
			if (files == nullptr)
			{
				break;
			}
			else 
			{
				auto filename = string( files->d_name);
				if ( filename != "." && filename != ".." && filename != "auton.dtd" )
				{
					m_xmlFiles.emplace_back(string(files->d_name));
				}

			} 
		}
	}
	else
	{
		// error condition need to handle
	}
#endif
// TODO handle windows so the simulator works
}

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
	m_startposchooser.AddOption("Gridcoop",m_statrpos);
	m_startposchooser.AddOption("Gridwall",m_statrpos);
	m_startposchooser.AddOption("Gridhp",m_statrpos);
	frc::SmartDashboard::PutData("StartPos",&m_startposchooser);

	//what you want to do in auton
	m_numofgamepiecechooser.AddOption("1 gamepiece",m_numofgamepiece);
	m_numofgamepiecechooser.AddOption("2 gamepieces",m_numofgamepiece);
	m_numofgamepiecechooser.AddOption("3 gamepieces",m_numofgamepiece);
	frc::SmartDashboard::PutData("NumOfpcs",&m_numofgamepiecechooser);
}