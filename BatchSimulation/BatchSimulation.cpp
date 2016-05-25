// BatchSimulation.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <jansson.h>
#include <time.h>

#include "CoverageAlgorithm.h"
#include "BoxPlotFile.h"
#include "BatchSimulation.h"

#include <jansson_config.h>
#include <stdlib.h>
#include <stdio.h>
#include <boost\config\compiler\visualc.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <exception>
#include <sstream>

#include <utility>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <stdexcept>
#include <..\SDK\rapidjson\include\rapidjson\rapidjson.h>
#include <..\SDK\rapidjson\include\rapidjson\document.h>
#include <..\SDK\rapidjson\include\rapidjson\stringbuffer.h>
#include <..\SDK\rapidjson\include\rapidjson\filereadstream.h>
#include <..\SDK\rapidjson\include\rapidjson\filewritestream.h>
#include <..\SDK\rapidjson\include\rapidjson\reader.h>
#include <..\SDK\rapidjson\include\rapidjson\writer.h>
#include <..\SDK\rapidjson\include\rapidjson\stream.h>
#include <..\SDK\rapidjson\include\rapidjson\allocators.h>
#include <..\SDK\rapidjson\include\rapidjson\memorybuffer.h>
#include <..\SDK\rapidjson\include\rapidjson\encodedstream.h>
#include <..\SDK\rapidjson\include\rapidjson\memorystream.h>
#include <..\SDK\rapidjson\include\rapidjson\stringbuffer.h>



using namespace rapidjson;
// ...
using namespace Robotics::GameTheory;
using namespace IDS::BaseGeometry;
using namespace std;

#define D_TEST
#define D_STATIC
#define D_EPSILON
#define _TALGORITHM

struct Log
{
	ofstream m_logFile;
	Log(const std::string & name = "end.txt")
	{
		//cout << name ;
		m_logFile.open(name.c_str());
		if (m_logFile.fail())
			throw std::exception("Unable to open log file");
	}

	Log& operator<<(const std::string & str)
	{
		if (m_logFile.is_open())
			m_logFile << str;
		cout << str;
		return *this;
	}

	Log& operator<<(const double & num)
	{
		if (m_logFile.is_open())
			m_logFile << num;
		cout << num;
		return *this;
	}

	Log& operator<<(const int & num)
	{
		if (m_logFile.is_open())
			m_logFile << num;
		cout << num;
		return *this;
	}

	void flush()
	{
		if (m_logFile.is_open())
			m_logFile.flush();
	}

	void close()
	{
		if (m_logFile.is_open())
			m_logFile.close();
	}

	// this is the type of std::cout
	typedef std::basic_ostream<char, std::char_traits<char> > CoutType;

	// this is the function signature of std::endl
	typedef CoutType& (*StandardEndLine)(CoutType&);

	// define an operator<< to take in std::endl
	Log& operator<<(StandardEndLine manip)
	{
		// call the function, but we cannot return it's value
		manip(m_logFile);
		manip(std::cout);

		return *this;
	}
};

//////////////////////////////////////////////////////////////////////////
// String Tokenizer
void tokenize(const std::string& str, const std::string& separators, std::vector<std::string> &tokens)
{
	tokens.clear();

	// skip separators at beginning
	std::string::size_type lastPos = str.find_first_not_of(separators, 0);

	// find first separator
	std::string::size_type pos = str.find_first_of(separators, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));

		// skip separators.  Note the "not_of"
		lastPos = str.find_first_not_of(separators, pos);

		// find next "non-separator"
		pos = str.find_first_of(separators, lastPos);
	}
}

struct SimulationConfig
{
	std::vector<double> StopRate;
	std::vector<int> MonitorUpdateTime;
	std::vector<int> ThiefJump;
	std::vector<double> Epsilon;
	std::vector<int> TimeEnd;
	std::vector<int> Period;
	int TestCase;
} g_config;

void readSimulationConfigFile(Log & _log, std::string const& _filename)
{
	std::ifstream file(_filename);

	std::string l_sep("\t");

	if (file.is_open())
	{
		std::string l_line;
		while (std::getline(file, l_line))
		{
			std::vector<std::string> l_token; // il l_token mette gli elementi delle righe successive via via che scorre il file
			tokenize(l_line, l_sep, l_token); // tokenize respect on separetor l_sep
			if (l_token.empty())
				continue;

			if (l_token[0] == "StopRate")
			{
				_log << "StopRate" << endl;
				for (size_t i = 1; i < l_token.size(); ++i)
				{
					g_config.StopRate.push_back(atof(l_token[i].c_str()));
					_log << g_config.StopRate.back() << endl;
				}
			}
			else if (l_token[0] == "Monitor")
			{
				_log << "Monitor" << endl;
				for (size_t i = 1; i < l_token.size(); ++i)
				{
					g_config.MonitorUpdateTime.push_back(atoi(l_token[i].c_str()));
					_log << g_config.MonitorUpdateTime.back() << "\t";
				}
				_log << endl;
			}
			else if (l_token[0] == "Jump")
			{
				_log << "Jump" << endl;
				for (size_t i = 1; i < l_token.size(); ++i)
				{
					g_config.ThiefJump.push_back(atoi(l_token[i].c_str()));
					_log << g_config.ThiefJump.back() << "\t";
				}
				_log << endl;
			}
			else if (l_token[0] == "Epsilon")
			{
				_log << "Epsilon" << endl;
				for (size_t i = 1; i < l_token.size(); ++i)
				{
					g_config.Epsilon.push_back(atof(l_token[i].c_str()));
					_log << g_config.Epsilon.back() << "\t";
				}
				_log << endl;
			}
			else if (l_token[0] == "Print")
			{
				_log << "EndTime" << endl;
				for (size_t i = 1; i < l_token.size(); ++i)
				{
					g_config.TimeEnd.push_back(atoi(l_token[i].c_str()));
					_log << g_config.TimeEnd.back() << "\t";
				}
				_log << endl;
			}
			else if (l_token[0] == "Period")
			{
				_log << "Period" << endl;
				for (size_t i = 1; i < l_token.size(); ++i)
				{
					g_config.Period.push_back(atoi(l_token[i].c_str()));
					_log << g_config.Period.back() << "\t";
				}
				_log << endl;
			}
			else if (l_token[0] == "TestCase")
			{
				_log << "TestCase" << endl;
				for (size_t i = 1; i < l_token.size(); ++i)
				{
					g_config.TestCase = atoi(l_token[i].c_str());
					_log << g_config.TestCase << endl;
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
std::vector<std::string> getAgentNames(std::string const& _folname)
{
	std::vector<std::string> l_result;

#ifdef _SINK

	l_result.push_back("Scenario_5G_1T_1S.dat");

#ifndef _TEST
	l_result.push_back("Scenario_10G_1T_1S.dat");
	l_result.push_back("Scenario_15G_1T_1S.dat");
	l_result.push_back("Scenario_20G_1T_1S.dat");
#endif

#else
	l_result.push_back("Scenario_5G_1T_multiAgentMODIFIED.dat");

#ifndef _TEST
	l_result.push_back("Scenario_10G_1T_multiAgent.dat");
	l_result.push_back("Scenario_15G_1T_multiAgent.dat");
	l_result.push_back("Scenario_20G_1T_multiAgent.dat");
#endif
#endif
	return l_result;
}

//////////////////////////////////////////////////////////////////////////
std::vector<std::string> getAreaNames(std::string const& _folname)
{
	std::vector<std::string> l_result;
	/// più scenari
	l_result.push_back("External30.txt");
	//l_result.push_back("External50.txt");


#ifndef _TEST
	//	l_result.push_back("External_trig.txt");
	//	l_result.push_back("External_road.txt");
	//	l_result.push_back("External_room.txt");
#endif
	return l_result;
}

/*std::string setFileName() {
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	char buffer[22];
	strftime(buffer, 22, "%G_%m_%e__%H_%M_%S", timeinfo);
	//std::string tmp = "logFile_";
	//tmp.append(buffer);
	//tmp.append(".txt");
	//std::cout << tmp << std::endl;
	return "log.txt";
	//return tmp;
}*/

// Returns the local date/time formatted as 2014-03-19 11:11:52
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);

	// for more information about date/time format
	strftime(buf, sizeof(buf), "Prova_%Y_%m_%d_%H_%M_%S", &tstruct);
	std::string date = string(buf);
	return date;
}




//////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	const std::string date = currentDateTime();
	//getchar();  // wait for keyboard input
	//setFileName();
	Log l_log("log.txt");
	Log l_benefitValue(date + "_benefitvalue.txt");
	Log l_potentialValue(date + "_potentialValue.txt");
	Log l_coverageValue(date + "_coverageValue.txt");


	const char* json = "ConfiguratioFile.json";

	FILE* fp = fopen("ConfigurationFile.json", "rb"); // non-Windows use "r"
	char readBuffer[65536];
	FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	std::cout << readBuffer[0] << endl;
	
	std::string str = "ConfigurationFile.json";
	rapidjson::Document document;
	
	

	std::string l_folname;

	if (argc < 1)
		return -1;
	else if (argc >= 2)
		l_folname = argv[1];


	// richiamo le funzioni sopra definite
	std::vector<std::string> l_AgentFilenames = getAgentNames(l_folname);
	std::vector<std::string> l_AreaFilenames = getAreaNames(l_folname);

	//std::vector<std::string> getConfigurationFileName(l_ConfigurationFile);


#ifdef _EPSILON

#ifndef _TEST
	std::string l_file("config_EPSILON.dat");
#else
	std::string l_file("simple_config_EPSILON.dat");
#endif

#else

#ifdef _STATIC

#ifndef _TEST
	std::string l_file("config_STATIC.dat");
#else 
	std::string l_file("simple_config_STATIC.dat");
#endif

#else

#ifdef _TALGORITHM

#ifndef _TEST
	std::string l_file("config_T.dat");
#else 
	std::string l_file("simple_config_T.dat");
#endif

#else

#ifndef _TEST
	std::string l_file("config.dat");
#else 
	std::string l_file("simple_config.dat");
#endif

#endif

#endif

#endif
	// prende il titolo del file e lo divide, mette i parametri di configurazione all'interno della struttura config,
	// in l_file ci sono le info relative alla configurazione
	readSimulationConfigFile(l_log, l_file);

	for (size_t o = 0; o < l_AgentFilenames.size(); ++o)
	{
		std::string l_AgentFilename = l_AgentFilenames[o];
		std::string l_folder = l_AgentFilename.substr(0, l_AgentFilename.find_last_of("/\\") + 1);
		std::string l_AgentName = l_AgentFilename.substr(l_AgentFilename.find_last_of("/\\") + 1, l_AgentFilename.find_last_of("."));

		l_log << "******************************************" << endl;
		l_log << "Agent File: " << l_AgentName << endl;

		for (size_t l = 0; l < l_AreaFilenames.size(); ++l)// scorre sul file External con 0 in sub liberi e 1 nei sub con ostacolo
		{

			std::string l_AreaFilename = l_AreaFilenames[l];
			std::string l_AreaName = l_AreaFilename.substr(l_AreaFilename.find_last_of("/\\") + 1, l_AreaFilename.find_last_of("."));

			std::string l_name = l_AreaName.substr(0, l_AreaName.find_last_of("."));
			l_name += ("_" + l_AgentName.substr(0, l_AgentName.find_last_of(".")));

			l_log << "-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-" << endl;
			l_log << "Area File: " << l_AreaName << endl;

			try
			{
				// per decidere gli algoritmi: per ora scelgo solo DISL o PIPIP, no PARETO
				for (int l_algorithmType = 0; l_algorithmType < 2; ++l_algorithmType)
				{
					std::string l_algName = (l_algorithmType == 0 ? "DISL" : l_algorithmType == 1 ? "PIPIP" : "PARETO");
					l_log << "---------Algorithm: " << l_algName << endl;
					l_name += "_";
					l_name += l_algName;

					BoxPlotFile l_boxPlot;
					for (size_t l_periodIndex = 0; l_periodIndex < g_config.Period.size(); ++l_periodIndex)
					{
						l_log << "-------Period: " << g_config.Period[l_periodIndex] << endl;

						for (size_t l_monitorUpdateTimeIndex = 0; l_monitorUpdateTimeIndex < g_config.MonitorUpdateTime.size(); ++l_monitorUpdateTimeIndex)
						{
							l_log << "-----Monitor Update: " << g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex] << endl;

							for (size_t l_thiefJumpIndex = 0; l_thiefJumpIndex < g_config.ThiefJump.size(); ++l_thiefJumpIndex)
							{
								l_log << "---Thief Jump: " << g_config.ThiefJump[l_thiefJumpIndex] << endl;

								for (int l_epsilonIndex = 0; l_epsilonIndex < g_config.Epsilon.size(); ++l_epsilonIndex)
								{
									double l_epsilon = g_config.Epsilon[l_epsilonIndex];

									l_log << "-Epsilon: " << g_config.Epsilon[l_epsilonIndex] << endl;

									// preparo i nomi dei file:
									char buffername[1024], buffernameBoxplot[1024], buffernameBoxPlotAscissa[1024];
									sprintf(buffername, "%s_Period_%d_MonitorUpdate_%d_ThiefJump_%d_Epsilon_%f",
										l_name.c_str(),
										g_config.Period[l_periodIndex],
										g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
										g_config.ThiefJump[l_thiefJumpIndex],
										g_config.Epsilon[l_epsilonIndex]);

									sprintf(buffernameBoxplot, "%s_boxplot.txt", buffername);
									//sprintf(buffernameBoxPlotAscissa, "%s_boxplot_ascissa.txt", buffername);

									for (int l_testIndex = 0; l_testIndex < g_config.TestCase; ++l_testIndex)
										// per g_test volte ripeto lo stesso scenario! con punti di partenza diversi per gli agenti
									{
										l_log << "-Case: " << l_testIndex << "..." << endl;

										setLostBattery(g_config.Epsilon[l_epsilonIndex]);

										// cerca il file di cui passo il nome e preleva agenti e area(0 o 1)
										std::shared_ptr<Robotics::GameTheory::CoverageAlgorithm> l_coverage =
											Robotics::GameTheory::CoverageAlgorithm::createFromAreaFile(
												l_AreaFilename,
												l_AgentFilename,
												l_algorithmType,
												g_config.Period[l_periodIndex],
												0.1);

										if (g_config.TimeEnd.size() > 0)
											// Stop algorithm when number of steps reach a given value
										{
											for (size_t l_TimeEndIndex = 0; l_TimeEndIndex < g_config.TimeEnd.size(); ++l_TimeEndIndex)
											{
												if (g_config.TimeEnd[l_TimeEndIndex] == 0)
													continue;

												l_log << "End Time " << g_config.TimeEnd[l_TimeEndIndex] << endl;

												l_coverage->update(
													g_config.TimeEnd[l_TimeEndIndex] - (l_TimeEndIndex == 0 ? 0 : g_config.TimeEnd[l_TimeEndIndex - 1]),
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex]);

												/// print data for BoxPlot:
												double l_potentialIndex = l_coverage->m_stats.getPotentialIndexMediumValue();
												double l_benefitIndex = l_coverage->m_stats.getBenefitIndexMediumValue(); //errore medio
												double l_coverageIndex = l_coverage->getGlobalTrajectoryCoverage();// numero di quadrati coperti
											

												//l_log << "Potential Index ";
												//l_log << l_potentialIndex; 
												//l_log << endl;

												l_potentialValue << l_potentialIndex << endl;

												l_boxPlot.add(
													"Potential Index",
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
													g_config.Epsilon[l_epsilonIndex],
													g_config.Period[l_periodIndex],
													g_config.TimeEnd[l_TimeEndIndex],
													l_potentialIndex);

												//l_log << "Benefit Index ";
												//l_log << l_benefitIndex;
												//l_log << endl;

												l_benefitValue << l_benefitIndex << endl;
												//prova << l_benefitValue << endl;

												l_boxPlot.add(
													"Benefit Index",
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
													g_config.Epsilon[l_epsilonIndex],
													g_config.Period[l_periodIndex],
													g_config.TimeEnd[l_TimeEndIndex],
													l_benefitIndex);

												//l_log << "Coverage Index ";
												//l_log << l_coverageIndex;
												//l_log << endl;

												l_coverageValue << l_coverageIndex << endl;

												l_boxPlot.add(
													"Coverage Index",
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
													g_config.Epsilon[l_epsilonIndex],
													g_config.Period[l_periodIndex],
													g_config.TimeEnd[l_TimeEndIndex],
													l_coverageIndex);
											}
										}
										else if (g_config.StopRate.size() > 0)
											// Stop algorithm when epsilon reach a given value
										{
											double l_rate = 2;
											int k = 0;
											l_coverage->updateMonitor();
											for (auto l_stoprate_index = 0; l_stoprate_index < g_config.StopRate.size(); ++l_stoprate_index)
											{
												int l_numberOfSteps = l_coverage->getNumberOfSteps(g_config.StopRate[l_stoprate_index]);

												l_coverage->update(l_numberOfSteps, g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex], g_config.ThiefJump[l_thiefJumpIndex]);

												double l_steadyValue;
												int l_steadyIndex;
												computeSteadyValue(l_coverage->m_stats.m_benefitIndex, l_steadyValue, l_steadyIndex);

												l_boxPlot.add("BenefitIndex Steady Value",
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
													g_config.Epsilon[l_epsilonIndex],
													g_config.Period[l_periodIndex], l_numberOfSteps,
													l_steadyValue);

												l_boxPlot.add("BenefitIndex Steady Index",
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
													g_config.Epsilon[l_epsilonIndex],
													g_config.Period[l_periodIndex], l_numberOfSteps,
													l_steadyIndex);

												double l_steadyValue_potential;
												int l_steadyIndex_potential;
												computeSteadyValue(l_coverage->m_stats.m_potentialIndex, l_steadyValue_potential, l_steadyIndex_potential);

												/// print data for BoxPlot:
												l_log << "Steady Value " << l_steadyValue_potential << "\t Steady Index " << l_steadyIndex_potential;
												l_log << endl;

												l_boxPlot.add("PotentialIndex Steady Value",
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
													g_config.Epsilon[l_epsilonIndex],
													g_config.Period[l_periodIndex], l_numberOfSteps,
													l_steadyValue_potential);

												l_boxPlot.add("PotentialIndex Steady Index",
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
													g_config.Epsilon[l_epsilonIndex],
													g_config.Period[l_periodIndex], l_numberOfSteps,
													l_steadyIndex_potential);

											}
										}
										else
											// Stop algorithm at the steady configuration
										{
											int l_numberOfSteps = l_coverage->getNumberOfSteps(-1);

											l_coverage->update(l_numberOfSteps, g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex], g_config.ThiefJump[l_thiefJumpIndex]);

											double l_steadyValue;
											int l_steadyIndex;
											computeSteadyValue(l_coverage->m_stats.m_benefitIndex, l_steadyValue, l_steadyIndex);

											l_boxPlot.add("BenefitIndex Steady Value",
												g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
												g_config.ThiefJump[l_thiefJumpIndex],
												g_config.Epsilon[l_epsilonIndex],
												g_config.Period[l_periodIndex], l_numberOfSteps,
												l_steadyValue);

											l_boxPlot.add("BenefitIndex Steady Index",
												g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
												g_config.ThiefJump[l_thiefJumpIndex],
												g_config.Epsilon[l_epsilonIndex],
												g_config.Period[l_periodIndex], l_numberOfSteps,
												l_steadyIndex);

											double l_steadyValue_potential;
											int l_steadyIndex_potential;
											computeSteadyValue(l_coverage->m_stats.m_potentialIndex, l_steadyValue_potential, l_steadyIndex_potential);

											/// print data for BoxPlot:
											l_log << "Steady Value " << l_steadyValue_potential << "\t Steady Index " << l_steadyIndex_potential;
											l_log << endl;

											l_boxPlot.add("PotentialIndex Steady Value",
												g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
												g_config.ThiefJump[l_thiefJumpIndex],
												g_config.Epsilon[l_epsilonIndex],
												g_config.Period[l_periodIndex], l_numberOfSteps,
												l_steadyValue_potential);

											l_boxPlot.add("PotentialIndex Steady Index",
												g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
												g_config.ThiefJump[l_thiefJumpIndex],
												g_config.Epsilon[l_epsilonIndex],
												g_config.Period[l_periodIndex], l_numberOfSteps,
												l_steadyIndex_potential);
										}
									}
								}
							}
						}
					}
					l_log << "Print BoxPlot Data _" << l_name << endl;
					l_log.flush();
					l_boxPlot.printOnNewFile(l_name + "_boxPlot.txt", l_name + "_VEC_boxPlot.txt");
				}
			}
			catch (...)
			{
				l_log << "Unable to process file " << l_name << endl;
				l_log.flush();
			}
		}
	}

	l_log << "Process is ended!" << endl;
	l_log.close();
	l_benefitValue.close();
	l_potentialValue.close();
	l_coverageValue.close();

	system("pause");

	return 0;
}
