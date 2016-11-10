#include "CoverageAlgorithm.h"
#include "CoverageAlgorithm.h"
#include "CoverageUtility.h"
#include "LearningAlgorithm.h"
#include "DISLAlgorithm.h"
#include "PIPIPAlgorithm.h"
#include "ParetoEfficientAlgorithm.h"
#include "CoarseCorrelatedAlgorithm.h"
#include "Probability.h"
#include "DiscretizedArea.h"
//aggiunta
#include "Area.h"
#include "StructuredArea.h"
#include "Agent.h"
#include "Guard.h"
#include "Neutral.h"
#include "Thief.h"
#include "Sink.h"
#include "World.h"

#include "BaseGeometry/MakePoint2D.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <time.h>


#include <Windows.h>
#include <rapidjson\document.h>
#include <rapidjson\filereadstream.h>

#define _ZMQ
using namespace std;

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
		//cout << str;
		return *this;
	}

	Log& operator<<(const double & num)
	{
		if (m_logFile.is_open())
			m_logFile << num;
		//cout << num;
		return *this;
	}

	Log& operator<<(const int & num)
	{
		if (m_logFile.is_open())
			m_logFile << num;
		//cout << num;
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

//aggiunto da C. Li Destri
struct vector_pos
{
	double x;
	double y;
	double theta;
} vector_pos;

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;
using namespace IDS::BaseGeometry;

const double g_CameraRadius = 7.;

//////////////////////////////////////////////////////////////////////////
void Robotics::GameTheory::setLostBattery(double _lostBattery)
{
	LOSTBATTERY_PERSTEP = _lostBattery;
}

//////////////////////////////////////////////////////////////////////////
Robotics::GameTheory::CoverageAlgorithm::CoverageAlgorithm(
	const std::set< std::shared_ptr<Agent> >& _agent, 
	std::shared_ptr<Area> _space, 
	int _type) 
	: m_world(nullptr)
	, m_learning(nullptr)
	//, m_time(0)
	, m_stats()
	, m_count(0)
{
	m_world = std::make_shared<World>(_agent, _space);

	std::shared_ptr<LearningAlgorithm> l_learning = nullptr;
	if(_type == 0)
		l_learning = std::make_shared<DISLAlgorithm>(m_world->getSpace());
	else if(_type == 1)
		l_learning = std::make_shared<PIPIPAlgorithm>(m_world->getSpace());
	else if(_type == 2)
		l_learning = std::make_shared<ParetoEfficientAlgorithm>(m_world->getSpace());
	else if(_type == 3)
		l_learning = std::make_shared<CoarseCorrelatedAlgorithm>(m_world->getSpace());
	else 
		throw std::exception("Error on selection of the Learning Algorithm.");

	this->setLearningAlgorithm(l_learning);

	m_stats.reset();
}

//////////////////////////////////////////////////////////////////////////
Robotics::GameTheory::CoverageAlgorithm::CoverageAlgorithm(
	const std::set< std::shared_ptr<Agent> >& _agent, 
	std::shared_ptr<DiscretizedArea> _space, 
	int _type) 
	: m_world(nullptr)
	, m_learning(nullptr)
	//, m_time(0)
	, m_stats()
	, m_count(0)
{
	m_world = std::make_shared<World>(_agent, _space);

	std::shared_ptr<LearningAlgorithm> l_learning = nullptr;
	if(_type == 0)
		l_learning = std::make_shared<DISLAlgorithm>(m_world->getSpace());
	else if(_type == 1)
		l_learning = std::make_shared<PIPIPAlgorithm>(m_world->getSpace());
	else if(_type == 2)
		l_learning = std::make_shared<ParetoEfficientAlgorithm>(m_world->getSpace());
	else if(_type == 3)
		l_learning = std::make_shared<CoarseCorrelatedAlgorithm>(m_world->getSpace());
	else 
		throw std::exception("Error on selection of the Learning Algorithm.");

	this->setLearningAlgorithm(l_learning);

	m_stats.reset();
}

//////////////////////////////////////////////////////////////////////////
void Robotics::GameTheory::CoverageAlgorithm::setLearningAlgorithm(std::shared_ptr<LearningAlgorithm> _learning)
{
	m_learning = _learning;
	m_learning->setGuards(m_world->getGuards());
}

//////////////////////////////////////////////////////////////////////////
void Robotics::GameTheory::CoverageAlgorithm::Reset()
{
	m_count = 0;
	m_stats.reset();
}

//////////////////////////////////////////////////////////////////////////
void Robotics::GameTheory::CoverageAlgorithm::updateMonitor()
{
	m_learning->resetValue();
	m_learning->monitoringThieves(m_world->getThieves());
	m_learning->monitoringSinks(m_world->getSinks());
}

#ifdef _ZMQ
//////////////////////////////////////////////////////////////////////////
bool Robotics::GameTheory::CoverageAlgorithm::updateViewer(int _nStep, int _monitorUpdateTime, int _thiefJump, zmq::socket_t *publisher, bool _continuousUpdate)
{
	bool res = true;

	if (m_count == 0)
		m_stats.reset();

	for (int i = 0; i < _nStep; ++i) // il valore di print
	{
		if (m_learning)
		{
			m_learning->updateTime();
			m_learning->resetCounter();

			if (m_count == 0 || (_monitorUpdateTime > 0 && !(m_count % _monitorUpdateTime))) // muove theives ogni M
			{
				this->updateMonitor();
				//std::cout << "Qui " << _monitorUpdateTime << endl;
				
				//prelevo le posizioni dei thieves
				std::vector<v_pos> temp1 = this->getThievesPosition1();
				for (int h = 0; h < temp1.size(); h++)
				{
					zmq::message_t message1(50);
					std::ostringstream stringStream;
					stringStream << "T," << h << "," << temp1[h].x << "," << temp1[h].y;
					std::string copyOfStr = stringStream.str();
					zmq::message_t msg1(copyOfStr.size());
					memcpy(msg1.data(), copyOfStr.c_str(), copyOfStr.size());
					publisher->send(msg1);
				}
			}

			res = m_learning->forwardOneStep();
			
			if (!res)
				return false;
			// prelevo le posizioni dei robot
			if (res /*&& (m_count % 5) == 0*/)
			{
				std::vector<v_pos> temp = m_learning->getGuardsPosition1();
				for (int z = 0; z < temp.size(); z++)
				{
					
					zmq::message_t message(50);
					std::ostringstream stringStream;
					stringStream << "A," << z << "," << temp[z].x << "," << temp[z].y << "," << temp[z].theta;
					//std::cout << " sono qui x: " << temp[z].x << " y: " << temp[z].y << "theta: " << temp[z].theta <<endl;
					std::string copyOfStr = stringStream.str();
					zmq::message_t msg(copyOfStr.size());
					memcpy(msg.data(), copyOfStr.c_str(), copyOfStr.size());
					publisher->send(msg);
				}
				
				// Potenziale di gioco: deriva dal benefit splittato per il n° robot che guardano la stessa regione
				double potential = m_learning->getPotentialValue();
				//cout << "Potential Value " << potential << endl;

				// team Benefit
				double benefitSquadra = m_learning->getBenefitValue();
				//cout << "Benefit Squadra: " << benefitSquadra << endl;

				// Benefit of single player
				std::set<std::shared_ptr<Guard>> gruppoGuardie = m_world->getGuards();
				int num = -1;
				/*for (std::set<std::shared_ptr<Guard>>::iterator it = gruppoGuardie.begin(); it != gruppoGuardie.end(); ++it) {
					std::shared_ptr<Guard> agent = *it;
					++num;
					cout << "robot " << num << "esimo" << " payoff = " << agent->getCurrentPayoff() << endl;
				}*/
				
				// Coverage Index
				int IndexOfCoverage = m_world->getSpace()->numberOfSquaresCoveredByGuards();
				//cout << "SquareCovered: " << IndexOfCoverage << endl;
				
				zmq::message_t message2(50);
				std::ostringstream stringStream;
				stringStream << "B," << benefitSquadra;
				std::string copyOfStr = stringStream.str();
				zmq::message_t msg2(copyOfStr.size());
				memcpy(msg2.data(), copyOfStr.c_str(), copyOfStr.size());
				publisher->send(msg2);

				zmq::message_t message3(50);
				std::ostringstream stringStream_p;
				stringStream_p << "P," << potential;
				std::string copyOfStr_p = stringStream_p.str();
				zmq::message_t msg3(copyOfStr_p.size());
				memcpy(msg3.data(), copyOfStr_p.c_str(), copyOfStr_p.size());
				publisher->send(msg3);

				zmq::message_t message4(50);
				std::ostringstream stringStream_c;
				stringStream_c << "C," << IndexOfCoverage;
				std::string copyOfStr_c = stringStream_c.str();
				zmq::message_t msg4(copyOfStr_c.size());
				memcpy(msg4.data(), copyOfStr_c.c_str(), copyOfStr_c.size());
				publisher->send(msg4);
			}// chiude learning
		}
			if (m_count == 0)
				this->wakeUpAgentIfSecurityIsLow();

			++m_count;
			if (_monitorUpdateTime > 0 && !(m_count % _monitorUpdateTime))
				// ... il monitor sta fermo ma il ladro si muove.
				m_world->moveThieves(_thiefJump);

			this->wakeUpAgentIfSecurityIsLow();

			if (_continuousUpdate || i == _nStep - 1)
				m_stats.addValues(
					m_learning->getTime(),
					this->numberOfSquaresCoveredByGuards(),
					m_learning->getPotentialValue(),
					m_learning->getBenefitValue(),
					this->getMaximumPotentialValue(),
					this->getSteadyNonCoopertativeBenefitValue(),
					m_learning->getExplorationRate(),
					m_learning->getBatteryValue());
	}
	return res;
}
#endif

const std::string date = currentDateTime();
//Log l_benefit(date + "_benefitSingle.txt"); // di tutti sommata 
//Log l_potential(date + "_potentialSingle.txt"); // splitto
//Log l_coverage(date + "_coverageSingle.txt"); //

//////////////////////////////////////////////////////////////////
bool Robotics::GameTheory::CoverageAlgorithm::update(int _nStep, int _monitorUpdateTime, int _thiefJump, bool _continuousUpdate)
{
	bool res = true;
	const std::string date = currentDateTime();
	

	if (m_count == 0)
		m_stats.reset();

	for (int i = 0; i < _nStep; ++i)
	{
		if (m_learning)
		{
			m_learning->updateTime();
			m_learning->resetCounter();

			if (m_count == 0 || (_monitorUpdateTime > 0 && !(m_count % _monitorUpdateTime)))
			{
				this->updateMonitor();
			}

			res = m_learning->forwardOneStep();
			if (!res)
				return false;
		}

		double benefit_Value = m_learning->getBenefitValue();
		double potential_Value = m_learning->getPotentialValue();
		double coveredSquares = m_world->getSpace()->numberOfSquaresCoveredByGuards();

		/*l_benefit << benefit_Value << endl;
		l_potential << potential_Value << endl;
		l_coverage << coveredSquares << endl;*/

		if (m_count == 0)
			this->wakeUpAgentIfSecurityIsLow();

		++m_count;
		if (_monitorUpdateTime > 0 && !(m_count % _monitorUpdateTime))
			// ... il monitor sta fermo ma il ladro si muove.
			m_world->moveThieves(_thiefJump);

		this->wakeUpAgentIfSecurityIsLow();

		if (_continuousUpdate || i == _nStep - 1)
			m_stats.addValues(
				m_learning->getTime(),
				this->numberOfSquaresCoveredByGuards(),
				m_learning->getPotentialValue(),
				m_learning->getBenefitValue(),
				this->getMaximumPotentialValue(),
				this->getSteadyNonCoopertativeBenefitValue(),
				m_learning->getExplorationRate(),
				m_learning->getBatteryValue());
	}
	return res;
}

//#if 0
//
////////////////////////////////////////////////////////////////////////////
//void Robotics::GameTheory::CoverageAlgorithm::update(int nStep, int _monitorUpdateTime, int _thiefJump)
//{
//#if 0
//	m_space->setRandomSquareValue();this->wakeUpAgentIfSecurityIsLow();
//#else
//	if(nStep == 0)
//	{
//		if(m_learning)
//		{
//			m_learning->resetCounter();
//
//			if(!( m_count % _monitorUpdateTime ))
//			{
//				m_learning->resetValue();
//				m_learning->monitoringThieves(m_agent);
//			}
//			m_learning->forwardOneStep();
//		}
//
//		if(m_count == 0)
//			this->wakeUpAgentIfSecurityIsLow();
//
//		double l_potValue = m_learning->getPotentialValue();
//		m_potValues.push_back(l_potValue);
//		double l_benefitValue = m_learning->getBenefitValue();
//		m_benefitValues.push_back(l_benefitValue);
//
//		double l_maxThiefValue= this->getMaximumBenefitValue();
//		m_maxThiefValue.push_back(l_maxThiefValue);
//		int l_numsquare = this->numberOfSquaresCoveredByGuards();
//		m_squares.push_back(l_numsquare);
//		int l_time = this->getTime();
//		m_times.push_back(l_time);
//	}
//	else
//	{
//		if(m_count == 0)
//		{
//			m_potValues.clear();
//			m_benefitValues.clear();
//			m_maxThiefValue.clear();
//			m_squares.clear();
//			m_times.clear();
//		}
//
//		for(int i = 0; i < nStep; ++i)
//		{
//			if(m_learning)
//			{
//				++m_time;
//				m_learning->updateTime();
//				m_learning->resetCounter();
//
//				if(!( m_count % _monitorUpdateTime ))
//				{
//					//m_learning->resetTime();
//					m_learning->resetValue();
//
//					//m_learning->updateTime();
//					m_learning->monitoringThieves(m_agent);
//				}
//				
//				m_learning->forwardOneStep();
//			}
//
//			if(m_count == 0)
//				this->wakeUpAgentIfSecurityIsLow();
//
//			++m_count;
//			if( !( m_count % _monitorUpdateTime ) ) // ... il monitor sta fermo.
//				m_world->moveThieves(_thiefJump);
//
//			this->wakeUpAgentIfSecurityIsLow();
//
//			double l_potValue = m_learning->getPotentialValue();
//			m_potValues.push_back(l_potValue);
//			double l_benefitValue = m_learning->getBenefitValue();
//			m_benefitValues.push_back(l_benefitValue);
//
//			double l_maxThiefValue= this->getMaximumBenefitValue();
//			m_maxThiefValue.push_back(l_maxThiefValue);
//			int l_numsquare = this->numberOfSquaresCoveredByGuards();
//			m_squares.push_back(l_numsquare);
//			int l_time = this->getTime();
//			m_times.push_back(l_time);
//		}
//	}
//	return;
//#endif
//}
//

//////////////////////////////////////////////////////////////////////////
void Robotics::GameTheory::CoverageAlgorithm::wakeUpAgentIfSecurityIsLow()
{
	return m_world->wakeUpAgentIfSecurityIsLow();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::Initialize()
{
	//this->randomInitializeAllAgent();

	m_learning->initialize();
}

//////////////////////////////////////////////////////////////////////////
void Robotics::GameTheory::CoverageAlgorithm::randomInitializeAllAgent()
{
	m_world->randomInitializeGuards();
	m_world->randomInitializeNeutrals();
	m_world->randomInitializeThief();
}

//////////////////////////////////////////////////////////////////////////
int CoverageAlgorithm::getNumberOfAgent()
{
	return m_world->getNumberOfAgent();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::removeAllThieves()
{
	return m_world->removeAllThieves();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::removeAllSinks()
{
	return m_world->removeAllSinks();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::setPositionOfSink(AgentPosition const& pos, SinkPtr _agent)
{
	bool found = false;
	/// Put agent in the space
	std::set< SinkPtr > l_agents = m_world->getSinks();
	for(auto it = l_agents.begin(); it != l_agents.end(); ++it)
	{
		SinkPtr l_agent = *it;
		if(_agent != l_agent)
			continue;

		found = true;

		// set position of the agent:
		l_agent->setCurrentPosition(pos);
	}

	if(!found)
	{
		if(!_agent)
			_agent = std::make_shared<Sink>(m_world->getNumberOfAgent(), pos);

		m_world->addSink( _agent );
	}
}


//////////////////////////////////////////////////////////////////////////
// cerca l'intruso
void CoverageAlgorithm::setPositionOfThief(AgentPosition const& pos, ThiefPtr _agent)
{
	bool found = false;
	/// Put agent in the space
	std::set< ThiefPtr > l_agents = m_world->getThieves();
	for(auto it = l_agents.begin(); it != l_agents.end(); ++it) //
	{
		ThiefPtr l_agent = *it;
		//if(!l_agent->isThief())
		if(_agent != l_agent)
			continue;

		found = true;

		// compute position of agent and camera:
		//AgentPosition pos = m_space->getRandomPosition();
		// set position of the agent:
		l_agent->setCurrentPosition(pos);
	}

	if(!found) // se non trova l'intruso
	{
		if(!_agent)
			_agent = std::make_shared<Thief>(m_world->getNumberOfAgent(), pos);
			
		m_world->addThief( _agent );

		//AgentPosition posiz_thief = _agent->getCurrentPosition();
	}
}



//////////////////////////////////////////////////////////////////////////
std::vector< std::shared_ptr<Square> > CoverageAlgorithm::getSquares() const
{
	return m_world->getSpace()->getSquares();
}

//////////////////////////////////////////////////////////////////////////
bool CoverageAlgorithm::areaContains(const IDS::BaseGeometry::Point2D & _thiefStartingPt) const
{
	return true;
}

//////////////////////////////////////////////////////////////////////////
SquarePtr CoverageAlgorithm::findSquare(IDS::BaseGeometry::Point2D const& point) const
{
	return m_world->getSpace()->getSquare(point);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::getGuardsPosition(std::vector<AgentPosition> & _pos)
{
	return m_learning->getGuardsPosition(_pos);
}



///////////// calo /////////////
std::vector<v_pos> CoverageAlgorithm::getGuardsPosition1()
{
	return m_learning->getGuardsPosition1();
}

// calo //
/*std::vector<AgentPosition> CoverageAlgorithm::getGuardsPosition1()
{
	std::vector<AgentPosition> _pos;
	_pos.clear();
	_pos.reserve(m_guards.size());
	for (set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		GuardPtr l_agent = *it;
		if (l_agent->isGuard())
		{
			_pos.push_back(l_agent->getCurrentPosition());
		}
	}
}*/

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::getGuardsCoverage( std::vector< std::vector<IDS::BaseGeometry::Point2D> > & _areas)
{
	return m_learning->getGuardsCoverage(_areas);
}

//////////////////////////////////////////////////////////////////////////
int CoverageAlgorithm::getGlobalTrajectoryCoverage()
{
	return m_learning->getGlobalTrajectoryCoverage();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::getGuardsSquare(std::vector<std::pair<SquarePtr, AgentActionIndex>> & _pos)
{
	return m_learning->getGuardsSquare(_pos);
}

//////////////////////////////////////////////////////////////////////////
int CoverageAlgorithm::numberOfSquaresCoveredByGuards() const
{
	return m_world->getSpace()->numberOfSquaresCoveredByGuards();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::getSinksPosition(std::vector<AgentPosition> & _pos)
{
	return m_world->getSinksPosition(_pos);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::getSinksSquare(std::vector<std::pair<std::shared_ptr<Square>,int>> & _pos)
{
	return m_world->getSinksSquare(_pos);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::getSinksCoverage( std::vector< std::vector<IDS::BaseGeometry::Point2D> > & _areas)
{
	return m_world->getSinksCoverage(_areas);
}

//////////////////////////////////////////////////////////////////////////
int CoverageAlgorithm::getNumberOfSteps(double _stopRate)
{
	return m_learning->getNumberOfSteps(_stopRate);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printPotential(std::string const& _filename, bool printOnFile)
{
	return m_stats.printPotential(_filename, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printBenefit(std::string const& _filename, bool printOnFile)
{
	return m_stats.printBenefit(_filename, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printPotentialIndex(std::string const& name, bool printOnFile)
{
	return m_stats.printPotentialIndex(name, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printBenefitIndex(std::string const& name, bool printOnFile)
{
	return m_stats.printBenefitIndex(name, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printPotentialIndexVersusExplorationRate(std::string const& name, bool printOnFile)
{
	return m_stats.printPotentialIndexVersusExplorationRate(name, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printExplorationRate(std::string const& name, bool printOnFile)
{
	return m_stats.printExplorationRate(name, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printNewPerformanceIndex(std::string const& name, bool _print)
{
	return m_stats.printBenefitIndex(name, _print);
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printNewPerformanceIndexVersusExplorationRate(std::string const& name, bool printOnFile)
{
	return m_stats.printPotentialIndexVersusExplorationRate(name, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
std::string CoverageAlgorithm::getExplorationRateStr()
{
	return m_learning->getExplorationRateStr();
}

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getExplorationRate()
{
	return m_learning->getExplorationRate();
}

//////////////////////////////////////////////////////////////////////////
std::string CoverageAlgorithm::getBatteryValueStr()
{
	return m_learning->getBatteryValueStr();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::getThievesPosition(std::vector<AgentPosition> & _pos)
{
	_pos.clear();

	std::set< ThiefPtr > l_thieves = m_world->getThieves();
	for(auto it = l_thieves.begin(); it != l_thieves.end(); ++it)
	{
		_pos.push_back( (*it)->getCurrentPosition() );
	}
}
////////////// aggiunta da C. Lidestri ///////////
std::vector<v_pos> CoverageAlgorithm::getThievesPosition1()
{
	v_pos vettore1;
	std::vector<v_pos> _pos;
	_pos.clear();
	std::set< ThiefPtr > l_thieves = m_world->getThieves();
	for (auto it = l_thieves.begin(); it != l_thieves.end(); ++it)
	{
		vettore1.x = (*it)->getCurrentPosition().getPoint2D().coord(0);
		vettore1.y = (*it)->getCurrentPosition().getPoint2D().coord(1);
		vettore1.theta = 0;
		_pos.push_back(vettore1);
	}
	return _pos;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
struct AgentDriver 
{
	// Position and orientation
	IDS::BaseGeometry::Point2D position;
	double heading;
	// Sensors Features
	double FarRadius;
	double NearRadius;
	double Orientation;
	double Angle;
	// Robot identificator
	int id;
	// Agents type
	enum Type
	{
		THIEF = -1,
		NEUTRAL = 0,
		GUARD = 1,
		SINK = 2
	} type;
};

//////////////////////////////////////////////////////////////////////////
/*void importFromFile(std::string const & _filename, std::vector<AgentDriver> & _agents)
{
	std::ifstream iFile(_filename);	// input.txt has integers, one per line

	while (!iFile.eof())
	{
		// questi li posso togliere
		int numOfXXX;
		/*iFile >> numOfXXX; //vertices
		for(int i = 0; i < numOfXXX; ++i)
		{
			double x, y;
			iFile >> x; //leggo da file e metto in x e y
			iFile >> y;
			_boundary.push_back( makePoint(IDSReal2D(x,y), EucMetric) );
		}*/
		/*iFile >> numOfXXX;	// guards
		for(int i = 0; i < numOfXXX; ++i)
		{
			double x, y, heading;
			iFile >> x;
			iFile >> y;
			iFile >> heading;
			AgentDriver driver;
			driver.position = makePoint(IDSReal2D(x,y), EucMetric);
			driver.heading = heading;
			driver.type = AgentDriver::GUARD;
			_agents.push_back(driver);
		}
		iFile >> numOfXXX; // Thieves
		for(int i = 0; i < numOfXXX; ++i)
		{
			double x, y, heading;
			iFile >> x;
			iFile >> y;
			AgentDriver driver;
			AgentDriver thief;
			driver.position = makePoint(IDSReal2D(x,y), EucMetric);
			driver.heading = heading;
			thief.position = makePoint(IDSReal2D(x, y), EucMetric);
			driver.type = AgentDriver::THIEF;
			_agents.push_back(driver);

		}
		iFile >> numOfXXX; // Sinks
		for(int i = 0; i < numOfXXX; ++i)
		{
			double x, y;
			iFile >> x;
			iFile >> y;
			AgentDriver driver;
			driver.position = makePoint(IDSReal2D(x,y), EucMetric);
			driver.type = AgentDriver::SINK;
			_agents.push_back(driver);
		}
		iFile >> numOfXXX; // Neutral (agenti neutri che dovrebbero essere 0)
		for(int i = 0; i < numOfXXX; ++i)
		{
			double x, y;
			iFile >> x;
			iFile >> y;
			AgentDriver driver;
			driver.position = makePoint(IDSReal2D(x,y), EucMetric);
			driver.type = AgentDriver::NEUTRAL;
			_agents.push_back(driver);
		}
	}
	iFile.close();
}*/

//////////////////////////////////////////////////////////////////////////////////////////////////
void importAgentsFromFile(rapidjson::Value & _agents, std::vector<AgentDriver> & agentsDriver,
	rapidjson::Value & _Neutralagents, std::vector<AgentDriver> & NeutralagentsDriver,
	rapidjson::Value & _Sink, std::vector<AgentDriver> & Sinks)
{
	// Agents

	AgentDriver driver;
	int id = 0;
	//////////Agents///////////
	if (!(_agents.HasMember("coord")))
		cout << "ERR: Nautral Agents has not coord" << endl;
	// check
	if (!(_agents.HasMember("sensors")))
		cout << "ERR: sensors field of Agents hasn't been decleared" << endl;

	rapidjson::Value& Agents_coord = _agents["coord"];
	rapidjson::Value& Agents_sensors = _agents["sensors"];

	// position and orientation
	if (Agents_coord.Size() == Agents_sensors.Size())
	{
		for (int i = 0; i < Agents_coord.Size(); i++)
		{
			driver.position = makePoint(IDSReal2D(Agents_coord[i].GetArray()[0].GetDouble(), Agents_coord[i].GetArray()[1].GetDouble()), EucMetric);
			driver.heading = Agents_coord[i].GetArray()[2].GetDouble();
			// Sensor Features
			driver.type = AgentDriver::GUARD;
			driver.FarRadius = Agents_sensors[i].GetArray()[0].GetDouble();
			driver.NearRadius = Agents_sensors[i].GetArray()[1].GetDouble();
			driver.Orientation = Agents_sensors[i].GetArray()[2].GetDouble();
			driver.Angle = Agents_sensors[i].GetArray()[3].GetDouble();
			driver.id = id++;
			agentsDriver.push_back(driver);
		}
	}
	else {
		cout << "ERR: coord and sensors must have same sizes" << endl;
	}

	///// Neutral ////////
	if (!(_Neutralagents.HasMember("coord")))
		cout << "ERR: coord field of array hasn't been decleared" << endl;
	// check
	if (!(_Neutralagents.HasMember("sensors")))
		cout << "ERR: sensors field of Agents hasn't been decleared" << endl;

	rapidjson::Value& NeutralAgents_coord = _Neutralagents["coord"];
	rapidjson::Value& NeutralAgents_sensors = _Neutralagents["sensors"];

	// position and orientation
	if (NeutralAgents_coord.Size() == NeutralAgents_sensors.Size())
	{
		for (int i = 0; i < NeutralAgents_coord.Size(); i++)
		{
			driver.position = makePoint(IDSReal2D(NeutralAgents_coord[i].GetArray()[0].GetDouble(), NeutralAgents_coord[i].GetArray()[1].GetDouble()), EucMetric);
			driver.heading = NeutralAgents_coord[i].GetArray()[2].GetDouble();
			// Sensor Features
			driver.type = AgentDriver::GUARD;
			driver.FarRadius = NeutralAgents_sensors[i].GetArray()[0].GetDouble();
			driver.NearRadius = NeutralAgents_sensors[i].GetArray()[1].GetDouble();
			driver.Orientation = NeutralAgents_sensors[i].GetArray()[2].GetDouble();
			driver.Angle = NeutralAgents_sensors[i].GetArray()[3].GetDouble();
			driver.id = id++;
			NeutralagentsDriver.push_back(driver);
		}
	}
	else {
		cout << "ERR: coord and sensors must have same sizes" << endl;
	}

	////// Sinks ///////
	if (!(_Sink.HasMember("coord")))
		cout << "Err: Sinks have not coordinate" << endl;
	
		rapidjson::Value& Sinks_coord = _Sink["coord"];

	for (rapidjson::Value::ConstValueIterator itr = Sinks_coord.Begin(); itr != Sinks_coord.End(); ++itr)
	{
		driver.position = makePoint(IDSReal2D(itr->GetArray()[0].GetDouble(), itr->GetArray()[1].GetDouble()), EucMetric);
	}

}

//////////////////////////////////////////////////////////////////////////////////////////////////
void importThievesFromFile(rapidjson::Value & _thieves, std::vector<AgentDriver> & _thiefDriver)
{
	AgentDriver driver;
	rapidjson::Value& thieves_coord = _thieves["coord"];

	for (int i = 0; i < thieves_coord.Size(); i++)
	{
			driver.position = makePoint(IDSReal2D(thieves_coord[i].GetArray()[0].GetDouble(), thieves_coord[i].GetArray()[1].GetDouble()), EucMetric);
			driver.heading = thieves_coord[i].GetArray()[2].GetDouble();
			driver.type = AgentDriver::THIEF;
			_thiefDriver.push_back(driver);
	}
	
}

//////////////////////////////////////////////////////////////////////////
/*std::shared_ptr<CoverageAlgorithm> Robotics::GameTheory::CoverageAlgorithm::createFromFile(std::string const & _filename, int _type, int _period)
{
	std::vector<IDS::BaseGeometry::Point2D> l_bound;
	std::vector<AgentDriver> l_agentDriver;

	// set of positions, orientations and sensors features for each robot in l_agentDriver vector
	importFromFile(_filename, l_agentDriver);

	/// Create Coverage Algorithm:
	std::shared_ptr<Area> l_space = std::make_shared<StructuredArea>(l_bound);

	// carica tutti gli genti di tipo guardia nel vector l_agents
	//int l_id = -1;
	/*std::set< std::shared_ptr<Agent> >l_agents;

	for(size_t i = 0; i < l_agentDriver.size(); ++i)
	{
		if(l_agentDriver[i].type != AgentDriver::GUARD)
			continue;
		//++l_id;
		AgentPosition l_pos(l_agentDriver[i].position, l_agentDriver[i].heading, CameraPosition(l_space->getDistance() / 15.) );

		std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, l_id, l_pos, _period, _type == 2? 1 : 2);

		l_agents.insert(l_agent);
	}

	std::shared_ptr<CoverageAlgorithm> l_algorithm = std::make_shared<CoverageAlgorithm>(l_agents, l_space, _type);

	for(size_t i = 0; i < l_agentDriver.size(); ++i)
	{
		if(l_agentDriver[i].type != AgentDriver::THIEF)
			continue;
		// aggiunto random angle
		AgentPosition l_pos(l_space->randomPosition(), 0.0, CameraPosition(l_space->getDistance() / 15., 0, IDSMath::Pi, IDSMath::Pi*(110/180) ) );
		Sleep(100);
		
		ThiefPtr l_agent = std::make_shared<Thief>(l_algorithm->getNumberOfAgent(), l_pos/*l_agentDriver[i].position*/ //);
	/*	l_algorithm->setPositionOfThief(l_pos, l_agent);
	}

	for(size_t i = 0; i < l_agentDriver.size(); ++i)
	{
		if(l_agentDriver[i].type != AgentDriver::SINK)
			continue;

		AgentPosition l_pos( l_space->randomPosition(), 0.0, CameraPosition(l_space->getDistance() / 15.) );
		Sleep(100);

		SinkPtr l_agent = std::make_shared<Sink>(l_algorithm->getNumberOfAgent(), l_pos /*l_agentDriver[i].position*/ //);
		//l_algorithm->setPositionOfSink(l_pos, l_agent);
	//}

	//return l_algorithm;
//}

////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<CoverageAlgorithm> Robotics::GameTheory::CoverageAlgorithm::createFromAreaFile(
	rapidjson::Value& Area,
	rapidjson::Value& _Agents,
	rapidjson::Value& _Thieves,
	rapidjson::Value& _Sinks,
	rapidjson::Value& _NeutralAgents,
	int _type,
	int _periodIndex,
	double _epsilon)
{
	
	std::vector<IDS::BaseGeometry::Point2D> l_bound;
	std::vector<AgentDriver> l_agentDriver;
	std::vector<AgentDriver> l_NeutralAgentsDriver;
	std::vector<AgentDriver> l_Sinks;
	std::vector<AgentDriver> l_thievesDriver;

	// in l_agentDriver mette le coordinate lette da _agentFile (Guards, Thieves, Sinks, Neutral)
	importAgentsFromFile(_Agents, l_agentDriver, _NeutralAgents, l_NeutralAgentsDriver, _Sinks, l_Sinks);
	importThievesFromFile(_Thieves, l_thievesDriver);

	/// Create Coverage Algorithm:
	std::shared_ptr<DiscretizedArea> l_space = std::make_shared<DiscretizedArea>(Area);// areaFile che viene costruita
	std::set< std::shared_ptr<Agent>> l_agents; 


	// Agents setting parameters to create Algorithm
	for (size_t i = 0; i < l_agentDriver.size(); i++)
	{
		AgentDriver tmp = l_agentDriver[i];
		/*cout << " x " << tmp.position.coord(0) << endl;
		cout << " y " << tmp.position.coord(1) << endl;
		cout << " prova " << tmp.heading << endl;
		cout << " prova " << tmp.Orientation << endl;*/
		AgentPosition l_pos(tmp.position, tmp.heading, CameraPosition(tmp.FarRadius, tmp.NearRadius, tmp.Orientation, tmp.Angle) );
		
		std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, tmp.id, l_pos, _periodIndex, _type == 2 ? 1 : 2);
		l_agents.insert(l_agent);
	}


	//ATTENZIONE NeutralAgents setting parameters to create Algorithm (prima non venivano considerati !)
	/*for (size_t i = 0; i < l_NeutralAgentsDriver.size(); i++)
	{
		AgentDriver tmp = l_NeutralAgentsDriver[i];
		int id = l_agentDriver.size()+i;*/ // per non sovrappore gli id dati agli agents
		/*AgentPosition l_pos(tmp.position, tmp.heading, CameraPosition(tmp.FarRadius, tmp.NearRadius, tmp.Orientation, tmp.Angle) );

		std::shared_ptr<Agent> l_agent = std::make_shared<Neutral>(1, id, l_pos, _periodIndex, _type == 2 ? 1 : 2);
		l_agents.insert(l_agent);
	}*/
	std::shared_ptr<CoverageAlgorithm> l_algorithm = std::make_shared<CoverageAlgorithm>(l_agents, l_space, _type);
	l_algorithm->setExperimentalRate(_epsilon);
	

	// SINKS setting parameters to create Algorithm
	for (size_t i = 0; i < l_Sinks.size(); i++)
	{
		AgentDriver tmp = l_Sinks[i];
		AgentPosition l_pos(tmp.position, 0.0, CameraPosition());

		SinkPtr l_agent = std::make_shared<Sink>(l_algorithm->getNumberOfAgent(), l_pos);
		l_algorithm->setPositionOfSink(l_pos, l_agent);
	}

	// Thieves setting parameters to create Algorithm
	for (size_t i = 0; i < l_thievesDriver.size(); i++)
	{
		AgentDriver tmp = l_thievesDriver[i];
		AgentPosition l_pos( tmp.position, tmp.heading, CameraPosition() );

		ThiefPtr l_agent = std::make_shared<Thief>(l_algorithm->getNumberOfAgent(), l_pos);
		l_algorithm->setPositionOfThief(l_agent->getCurrentPosition(), l_agent);
	}
	return l_algorithm;
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::setExperimentalRate(double _epsilon)
{
	m_learning->setExperimentalRate(_epsilon);
}

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getMaximumPotentialValue()
{
	return m_world->getMaximumValue();
}

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getSteadyNonCoopertativeBenefitValue()
{
	std::set<ThiefPtr> l_thieves = m_world->getThieves();
	return double(l_thieves.size()) * std::log( double(m_world->getGuards().size()) ) * g_maxValue/g_maxValuePossible;
}

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getMaximumBenefitValue()
{
	// CONTROLLARE
	std::set<ThiefPtr> l_thieves = m_world->getThieves();
	return double(l_thieves.size()) * 100.;// * m_world->getSpace()->getNumberOfValidSquare();
}


double CoverageAlgorithm::getPotentialIndexMediumValue()
{
	return m_stats.getPotentialIndexMediumValue();
}

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getBenefitIndexMediumValue()
{
	return m_stats.getBenefitIndexMediumValue();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printArea(const std::string & filename)
{
	std::ofstream outFile;
	outFile.open(filename);
	if(outFile.is_open())
	{
		for(int row = 0; row < m_world->getSpace()->getNumRow(); ++row )
		{
			for(int col = 0; col < m_world->getSpace()->getNumCol(); ++col)
			{
				SquarePtr l_square = m_world->getSpace()->getSquare(row,col);
				if(l_square->isValid())
					outFile << "0";
				else
					outFile << "1";
			}
			outFile << endl;
		}
	}
}

#pragma region CONFIGURATION

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getTrajectoryPotentialIndex()
{
	double l_potValue = m_learning->getPotentialValue();
	double l_nonCooperativeSteadyValue = this->getSteadyNonCoopertativeBenefitValue();
	std::cout << "l_potvalue" << l_potValue << std::endl;
	return l_potValue/l_nonCooperativeSteadyValue;
}

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getTrajectoryBenefitIndex()
{
	double l_maxBenefitValue = this->getMaximumPotentialValue();
	double l_benefitValue = m_learning->getBenefitValue();

	return ( l_maxBenefitValue-l_benefitValue ) / l_maxBenefitValue;
}

//////////////////////////////////////////////////////////////////////////
double CoverageAlgorithm::getTrajectoryCoverage()
{
	return m_learning->getGlobalTrajectoryCoverage();
}

//////////////////////////////////////////////////////////////////////////
void CoverageAlgorithm::printPhoto(std::string const& _outputFileName)
{
	std::ofstream l_file;
	l_file.open(_outputFileName);
	if (!l_file.is_open())
		return; 

	m_world->saveConfiguration(l_file);

	double l_potentialIndex = this->getTrajectoryPotentialIndex();
	l_file << "Potential Index " << l_potentialIndex << endl;
	double l_benefitIndex = this->getTrajectoryBenefitIndex();
	l_file << "Benefit Index " << l_benefitIndex << endl;
	double l_coverageIndex = this->getTrajectoryCoverage();
	l_file << "Coverage Index " << l_coverageIndex << endl;
	double l_explorationRate = m_learning->computeExplorationRate();
	l_file << "Esploration Rate " << l_explorationRate << endl;

	l_file.close();
	
	return;
}

#pragma endregion
