// BatchSimulation.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <time.h>

#include "CoverageAlgorithm.h"
#include "BoxPlotFile.h"
#include "BatchSimulation.h"

#include <stdlib.h>
#include <stdio.h>
#include <exception>
#include <sstream>

#include <utility>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <stdexcept>

#include <zmq/zmq.h>
#include <zmq/zmq.hpp>

#include<rapidjson\document.h>
#include<rapidjson\filereadstream.h>


#include "IDSBaseMath\IDSMath.h"

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

/*void readSimulationConfigFile(Log & _log, std::string const& _filename)
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
}*/

///////////////////// NEW : readConfigurationFile /////////////////////
void readSimulationConfigFile(Log & _log, rapidjson::Value& Config_Param) {

	std::vector<double> StopRate;
	std::vector<int> MonitorUpdateTime;
	std::vector<int> ThiefJump;
	std::vector<double> Epsilon_v;
	std::vector<int> TimeEnd_v;
	std::vector<int> Period_v;
	int TestCase_v;

	// Prelevo gli elementi dei vari Array
	if (Config_Param.HasMember("Monitor")) {
		rapidjson::Value& Monitor = Config_Param["Monitor"];

		_log << "Monitor" << endl;
		for (int i = 0; i < Monitor.Size(); i++)
		{
			int tmp = Monitor[i].GetInt();
			cout << Monitor[i].GetInt() << " \t";
			MonitorUpdateTime.push_back(tmp);
		}
		g_config.MonitorUpdateTime = MonitorUpdateTime;
		_log << endl;
	}
	else {
		cout << "Monitor field hasn't been decleared";
	}

	if (Config_Param.HasMember("Jump")) {
		rapidjson::Value& Jump = Config_Param["Jump"];
		_log << "Jump" << endl;
		for (int i = 0; i < Jump.Size(); i++)
		{
			int tmp = Jump[i].GetInt();
			cout << Jump[i].GetInt() << "\t";
			ThiefJump.push_back(tmp);
		}
		g_config.ThiefJump = ThiefJump;
		_log << endl;
	}
	else {
		cout << "Jump field hasn't been decleared" << endl;
	}

	if (Config_Param.HasMember("Epsilon")) {
		rapidjson::Value& Epsilon = Config_Param["Epsilon"];
		_log << "Epsilon";
		for (int i = 0; i < Epsilon.Size(); i++)
		{
			double tmp = Epsilon[i].GetDouble();
			cout << Epsilon[i].GetDouble() << '\t';
			Epsilon_v.push_back(tmp);
		}
		g_config.Epsilon = Epsilon_v;
		_log << endl;
	}
	else {
		cout << "Epsilon field hasn't been decleared" << endl;
	}

	if (Config_Param.HasMember("Print")) {
		rapidjson::Value& Print = Config_Param["Print"];
		_log << "Print" << endl;
		for (int i = 0; i < Print.Size(); i++)
		{
			int tmp = Print[i].GetInt();
			cout << Print[i].GetInt() << "\t";
			g_config.TimeEnd.push_back(tmp);
		}
		_log << endl;
	}
	else {
		cout << "Print field hasn't been decleared" << endl;
	}

	if (Config_Param.HasMember("Period")) {
		rapidjson::Value& Period = Config_Param["Period"];
		_log << "Period" << endl;
		for (int i = 0; i < Period.Size(); i++)
		{
			int tmp = Period[i].GetInt();
			cout << Period[i].GetInt() << "\t";
			Period_v.push_back(tmp);
		}
		g_config.Period = Period_v;
		_log << endl;
	}
	else {
		cout << "Period field hasn't been decleared" << endl;
	}

	if (Config_Param.HasMember("TestCase")) {
		rapidjson::Value& TestCase = Config_Param["TestCase"];
		_log << "TestCase" << endl;
		g_config.TestCase = TestCase.GetInt();
		cout << TestCase.GetInt();
		_log << endl;
	}
	else {
		cout << "TestCase field hasn't been decleared" << endl;
	}
}

////////////////// new ///////////////////////////////////////////
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


//////////////////////////////////////// MAIN ////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	const std::string date = currentDateTime();

	//  Prepare our context and publisher
	zmq::context_t context(1);
	zmq::socket_t publisher(context, ZMQ_PAIR);
	publisher.connect("tcp://localhost:5555");
	
	Log l_log("log.txt");
	Log l_benefitValue(date + "_benefitvalue.txt");
	Log l_potentialValue(date + "_potentialValue.txt");
	Log l_coverageValue(date + "_coverageValue.txt");
	//Log l_positions(date + "_positionsOfRobots");

	// read file.json for set os parameters simulation
	std::string conf_file = "Scenario_5G_1T_multiAgent.json";
    FILE * cf = fopen(conf_file.c_str(), "r" );
    char readBuffer[65536];
    rapidjson::FileReadStream is(cf, readBuffer, sizeof(readBuffer));
    rapidjson::Document document;
    document.ParseStream(is);
    
	if (!document.IsObject()) {
        std::cout << "ERR: error during the parsing of configuration file" << std::endl;
        exit(1);
    }
	// Area
    rapidjson::Value& Area = document["Area"];
	// Agents
	rapidjson::Value& Agents = document["Agents"];
	//Thieves
	rapidjson::Value& Thieves = document["Thieves"];
	//Sinks
	rapidjson::Value& Sinks = document["Sinks"];
	// Neutral_agents
	rapidjson::Value& NeutralAgents = document["NeutralAgents"];
	// Configuration
	rapidjson::Value& Configuration_parameters = document["Configuration"];
	rapidjson::Value& Monitor = Configuration_parameters["Monitor"];
	//cout << Configuration_parameters.HasMember("Monitor");
	/*int a = Monitor[1].GetInt();
	int size = Monitor.Size();
	std::vector<int> MonitorUpdateTime;
	cout << a << size << endl;

	for (int i = 0; i < Monitor.Size(); i++)
	{
		int c = Monitor[i].GetInt();
		MonitorUpdateTime.push_back(c);
		cout << Monitor[i].GetInt() << endl;
	}

	g_config.MonitorUpdateTime = MonitorUpdateTime;*/

    std::string l_folname;
	if (argc < 1)
		return -1;
	else if (argc >= 2)
		l_folname = argv[1];

	std::vector<std::string> l_AgentFilenames;
	l_AgentFilenames.push_back("Scenario_5G_1T_multiAgent.json");

	std::vector<std::string> l_AreaFilenames;
	rapidjson::Value& AreaName = Area["Area_name"];

	// prendo il char che descrive lo scenario. es. Open_sea
	l_AreaFilenames.push_back(AreaName.GetString());


	// prende il titolo del file e lo divide, mette i parametri di configurazione all'interno della struttura config,

	readSimulationConfigFile(l_log, Configuration_parameters);

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
												Area,
												Agents,
												Thieves,
												Sinks,
												NeutralAgents,
												l_algorithmType, // DISL O PIPIP
												g_config.Period[l_periodIndex],
												0.1);

										if (g_config.TimeEnd.size() > 0) {
											// Stop algorithm when number of steps reach a given value
											for (size_t l_TimeEndIndex = 0; l_TimeEndIndex < g_config.TimeEnd.size(); ++l_TimeEndIndex)
											{
												if (g_config.TimeEnd[l_TimeEndIndex] == 0)
													continue;
												
												l_log << "End Time " << g_config.TimeEnd[l_TimeEndIndex] << endl;

												l_coverage->updateViewer(
													g_config.TimeEnd [l_TimeEndIndex] - (l_TimeEndIndex == 0 ? 0 : g_config.TimeEnd[l_TimeEndIndex - 1]),
													g_config.MonitorUpdateTime[l_monitorUpdateTimeIndex],
													g_config.ThiefJump[l_thiefJumpIndex],
                                                    &publisher,
													l_coverage->numberOfSquaresCoveredByGuards()
                                                );

											
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
