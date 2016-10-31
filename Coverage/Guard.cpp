#include "Guard.h"
#include "DiscretizedArea.h"
#include "Probability.h"
#include "CoverageUtility.h"
#include <utility> 
#include <utility>

//#define _PRINT

using namespace Robotics::GameTheory;
using namespace IDS::BaseGeometry;

double LOSTBATTERY_PERSTEP = .1;

Guard::Guard( int _teamID, int _id, AgentPosition _position, int _trajectoryLength, int _memorySpace ) 
	: Agent(_id, _position)
	, m_teamID(_teamID)
	, m_currentTrajectoryPayoff(0.)
	, m_currentPayoff(0.)
	, m_currentTrajectory()
	, m_maxTrajectoryLength(_trajectoryLength)
	, m_memory(_memorySpace)
	, m_coverage()
	, m_oldCoverage()
	, m_currentMood(Content)
	, m_current_battery(MAXIMUM_BATTERY)
	, m_minimum_battery(MINIMUM_BATTERY)
	, m_maximum_battery(MAXIMUM_BATTERY)
{}

Guard::~Guard()
{}



// return vector with all AgentPosition of feasible actions
//////////////////////////////////////////////////////////////////////////
std::vector<AgentPosition> Guard::getFeasibleActions( std::shared_ptr<DiscretizedArea> _space ) const
{
	AreaCoordinate l_currCoord = _space->getCoordinate( m_currentPosition.getPoint2D() );
	l_currCoord.heading = m_currentPosition.m_heading;

	// getStandardApproachableValidSquares deve dipendere anche da heading che ho aggiunto
	std::vector<AreaCoordinate> l_squares = _space->getStandardApproachableValidSquares(l_currCoord);
	
	// ho tolto la funzione sotto perchè sono già incluse dalla precedente
	//_space->addSpecialApproachableValidSquares(l_currCoord, l_squares); // aggiunge le diagonali in l_sqaure

	std::vector<AgentPosition> l_result;
	for(size_t i = 0; i < l_squares.size(); ++i )
	{
		l_result.push_back( AgentPosition(_space->getPosition(l_squares[i]), l_squares[i].heading, m_currentPosition.m_camera) );
	}

	return l_result;
}

//////////////////////////////////////////////////////////////////////////
int Guard::getBestTrajectoryIndex(bool _best)
{
	//m_memory.computeBestWorstTrajectories();
	return _best ? m_memory.m_best : m_memory.m_worst;
}

//////////////////////////////////////////////////////////////////////////
void RemoveBestPositionFromFeasible(std::vector<AgentPosition> & _feasible, AgentPosition const& _bestPosition)
{
	Point2D l_posBest = _bestPosition.getPoint2D();

	if(l_posBest.isValid())
	{
		for(size_t j = 0; j < _feasible.size();)
		{
			Point2D l_posFeas = _feasible[j].getPoint2D();
			bool found = false;
			if(l_posFeas.distance(l_posBest) < 1.e-1)
			{
				found = true;
				break;
			}
			if(found)
				_feasible.erase(_feasible.begin()+j);
			else
				++j;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void Guard::removeBestPositionFromFeasible(std::vector<AgentPosition> & _feasible)
{
	MemoryGuardTrajectory l_best;
	if( !m_memory.bestTrajectory(l_best))
		return;

	if(!(m_currentTrajectory==l_best.m_memTrajectory))
		return;

	if( m_currentTrajectory.isLast(m_maxTrajectoryLength) )
	{
		RemoveBestPositionFromFeasible(_feasible, l_best.m_memTrajectory.getLastPosition());
	}
}

//////////////////////////////////////////////////////////////////////////
std::set<std::shared_ptr<Guard> > Guard::getCommunicableAgents(std::set< std::shared_ptr<Guard> > const& _guards) const
{
	std::set<std::shared_ptr<Guard> > l_result;
	for(std::set<std::shared_ptr<Guard> >::const_iterator it = _guards.begin(); it != _guards.end(); ++it)
	{
		std::shared_ptr<Guard> l_agent = *it;
		if( this->equals(l_agent) )
			continue;

		if( this->m_currentPosition.communicable( l_agent ) )
		{
			l_result.insert(l_agent);
		}
	}
	return l_result;
}

//////////////////////////////////////////////////////////////////////////
double Guard::computeCurrentCosts() const
{
	return m_currentPosition.computeCosts();
}

//////////////////////////////////////////////////////////////////////////
void Guard::updateMemoryTrajectories()
{
	m_memory.add(MemoryGuardTrajectory(m_currentTrajectory, m_currentTrajectoryPayoff, m_currentMood));
	m_memory.computeBestWorstTrajectories();
}

#pragma region PARETO

Mood Guard::computeMood(double _explorationRate)
{
	int l_index = getBestTrajectoryIndex(true);
	
	if( l_index >= 0 &&
		m_memory.m_elems[l_index].equals(m_currentTrajectory, m_currentTrajectoryPayoff) && 
		m_memory.m_elems[l_index].m_mood == Content)
	{
		return Content;
	}

	double l_explorationRate = pow( _explorationRate, 1 - m_currentTrajectoryPayoff );
	bool l_agentHasToExperiments = agentHasToExperiments(l_explorationRate);
	if(l_agentHasToExperiments)
		return Content;
	else
		return Discontent;
}

#pragma endregion


#pragma region DISLALGORITHM

///
bool Guard::isRunning() const
{
	const int len = m_currentTrajectory.size();
	return len > 0 && len < m_maxTrajectoryLength;
	//return m_currentTrajectory.size() < m_maxTrajectoryLength; 
}

///
void Guard::reset(double _explorationRate)
{
	if( !(m_currentTrajectory.size() == 0) )
	{
		Mood l_mood = computeMood(_explorationRate);

		updateMemoryTrajectories();

		setCurrentMood(l_mood);
	}
	m_currentTrajectory.clear();
	m_currentTrajectoryPayoff = 0.;

	m_oldCoverage = m_coverage;
	m_coverage.clear();

	int l_period = computePeriod();
	if(l_period != m_maxTrajectoryLength)
	{
		resetMemory();
		updatePeriod(l_period);
	}
}

///
void Guard::startExperiment(double _explorationRate)
{
	reset(_explorationRate);
	m_exploring = -1;
}

///
void Guard::followBestTrajectory(double _explorationRate, bool best)
{
	reset(_explorationRate);
	m_exploring = getBestTrajectoryIndex(best);
}

double DeleteDecErr1(double _heading) {

	double value;

	if (_heading >= 6.20 || (_heading >= 0 && _heading <= 0.2))
		value = 0;
	if (_heading < 0.9 && _heading > 0.6)// ok 0.785
		value = 0.78;
	if (_heading > 1.4 && _heading < 1.7) //1.57
		value = 1.57;
	if (_heading < 2.4 && _heading > 2.2) // ok 2.35
		value = 2.35;
	if (_heading < 3.2 && _heading > 3) // 3.14
		value = 3.14;
	if (_heading < 4 && _heading > 3.8) // 3.92 
		value = 3.92;
	if (_heading < 4.8 && _heading > 4.6) // 3.14
		value = 4.71;
	if (_heading > 5.3 && _heading < 5.6) // ok 5.49
		value = 5.49;

	return value;
}

double Mod2Pig(double angle) {
	if (angle < 0)	return DeleteDecErr1(angle + IDSMath::TwoPi);
	if (angle >= IDSMath::TwoPi)	return DeleteDecErr1(angle - IDSMath::TwoPi);
	if (angle >= 0 && angle < IDSMath::TwoPi)	return DeleteDecErr1(angle);
	else std::cout << " values out of range " << std::endl;
}

AgentPosition Guard::rotateRandomly(int number, AgentPosition _agentPosition, double _angle) //number = 2 perche ho 2 possibilita antiorario o orario
{
	int value = rand() % number;
	//std::cout << value << std::endl;
	if (value == 1)
	{
		_agentPosition.m_heading = _agentPosition.m_heading - _angle;
		return _agentPosition;
	}
	if (value == 0)
	{
		_agentPosition.m_heading = _agentPosition.m_heading + _angle;
		return _agentPosition;
	}
}

AgentPosition Guard::chooseRandomRotation(int number, AgentPosition _agentPosition, double _angle1, double _angle2) {
	// _angle1 clockwise, _angle2 counterclockwise
	int value = rand() % number;
	//std::cout << value << std::endl;
	if (value == 1)
	{
		_agentPosition.m_heading = _agentPosition.m_heading + _angle1;
		return _agentPosition;
	}
	if (value == 0)
	{
		_agentPosition.m_heading = _agentPosition.m_heading - _angle2;
		return _agentPosition;
	}
}

AgentPosition Guard::SquareFree(std::shared_ptr<DiscretizedArea> _space, AgentPosition _position) {
	AreaCoordinate Coordinate = _space->getCoordinate(_position.getPoint2D());
	SquarePtr square = _space->getSquare(Coordinate);
	// controllo che sia libero la square
	if (square->isValid())
		return _position; // (indexBest, indexNext)
	if (!square->isValid())
	{
		return m_currentPosition;
	}
}

///
void Guard::selectNextAction(std::shared_ptr<DiscretizedArea> _space)
{
	// Seleziona la nuova posizione sulla base se Experiments o No

	switch (m_exploring)
	{
	case -1:
		// a random position is selected (ABCDEFG)
		//std::cout << "controllo " << selectNextFeasiblePosition(_space).getPoint2D().coord(0) << " " << selectNextFeasiblePosition(_space).getPoint2D().coord(1) << std::endl;
		this->setNextPosition(SquareFree(_space, selectNextFeasiblePosition(_space))); // mette in m_nextPosition la Position selezionata tra le fattbili

		break;
	default:
		// a position from past ones is selected
		AgentPosition nextAgentPosition = m_memory.getNextPosition(m_exploring, m_currentTrajectory.size());
		// take the coordinates current and next position
		double x_curr = m_currentPosition.getPoint2D().coord(0);
		double y_curr = m_currentPosition.getPoint2D().coord(1);
		double x_prox = nextAgentPosition.getPoint2D().coord(0);
		double y_prox = nextAgentPosition.getPoint2D().coord(1);
		// caso in cui i vincoli lo cosentono
		if (x_curr == x_prox && y_curr == y_prox) 
		{
			#ifdef _PRINT
				//std::cout << "feasible action " << std::endl;
			#endif // _PRINT
			AreaCoordinate Coordinate = _space->getCoordinate(nextAgentPosition.getPoint2D());
			SquarePtr square = _space->getSquare(Coordinate);
			// controllo che sia libero la square
			if(square->isValid())
				this->setNextPosition(nextAgentPosition); // (indexBest, indexNext)
			if (!square->isValid())
			{
				nextAgentPosition = m_currentPosition;
				this->setNextPosition(nextAgentPosition);
			}
		}
		// since previous last two actions cannot be choose with olny rotation (kinematic constraint) We have to check coordinate 
		// of next position chosed by algorithm and we have to rotate the robot towards the desidered next position first.
		// after rotation if the  best position is still that the robot will be able to reach it according to its new direction.
		if (x_curr != x_prox || y_curr != y_prox)
		{
			if (x_prox < x_curr) // verso ovest
			{
				if (y_prox == y_curr)	// lungo x a sinistra
				{
					nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi);
					#ifdef _PRINT
						std::cout << "lungo x a sx " << std::endl;
					#endif
				}
				if (y_prox > y_curr)	// obliquo alto vs sinistra
				{
					if (m_currentPosition.m_heading >= 2.30 && m_currentPosition.m_heading <= 2.40) // in questa posizione ci arrivo con 2.35 di orientazione
					{
						nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi); // antioraria di Pi
					#ifdef _PRINT
						std::cout << "obliquo alto vs sx" << std::endl;
					#endif
					}
					if (m_currentPosition.m_heading < 2.30 && m_currentPosition.m_heading > 2.40) 
					{
					#ifdef _PRINT
						std::cout << "non dovrebbe mai capitare" << std::endl;
					#endif
					}
				}
				if (y_prox < y_curr)	// obliquo basso vs sx
				{
					if (m_currentPosition.m_heading >= 0.7 && m_currentPosition.m_heading <= 0.85) 
					{
						nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi); // antioraria di Pi
					#ifdef _PRINT
						std::cout << "obliquo basso a sx " << std::endl;
					#endif
					}
					if (m_currentPosition.m_heading < 0.7 && m_currentPosition.m_heading > 0.85)
					{
					#ifdef _PRINT
						std::cout << "non dovrebbe mai capitare" << std::endl;
					#endif
					}
				}
			}
			if (x_prox > x_curr) // verso est
			{
				if (y_prox == y_curr)	// lungo x vs destra
				{
					nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi);
				#ifdef _PRINT
					std::cout << "lungo x vs destra" << std::endl;

				#endif
				}
				if (y_prox > y_curr)	//obliquo alto verso destra
				{
					if (m_currentPosition.m_heading >= 3.87 && m_currentPosition.m_heading <= 3.97)
					{
						nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi); // antioraria di Pi
					#ifdef _PRINT
						std::cout << "obliquo alto vs destra" << std::endl;
					#endif
					}
					if (m_currentPosition.m_heading < 3.87 && m_currentPosition.m_heading > 3.97)
					{
					#ifdef _PRINT
						std::cout << "non dovrebbe mai capitare" << std::endl;
					#endif
					}
				}
				if (y_prox < y_curr)	//obliquo basso verso destra
				{
					if (m_currentPosition.m_heading >= 5.40 && m_currentPosition.m_heading <= 5.55) {
						nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi); // antioraria di Pi
					#ifdef _PRINT
						std::cout << "obliquo basso vs destra" << std::endl;
					#endif
					}
					if (m_currentPosition.m_heading < 5.40 && m_currentPosition.m_heading > 5.55)
					{
					#ifdef _PRINT
						std::cout << "non dovrebbe mai capitare" << std::endl;
					#endif
					}
				}
			}
			if (x_prox == x_curr)
			{
				if (y_prox > y_curr)	//lungo y verso alto
				{
					nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi);
				#ifdef _PRINT
					std::cout << "lungo y vs alto" << std::endl;
				#endif
				}
				if (y_prox < y_curr)	//lungo y verso basso
				{
					nextAgentPosition = rotateRandomly(2, m_currentPosition, IDSMath::Pi);
				#ifdef _PRINT
					std::cout << "lungo y vs basso" << std::endl;
				#endif
				}
			}

			AreaCoordinate Coordinate = _space->getCoordinate(nextAgentPosition.getPoint2D());
			SquarePtr square = _space->getSquare(Coordinate);
			// controllo che sia libero la square
			if (square->isValid())
				this->setNextPosition(SquareFree(_space, nextAgentPosition)); // (indexBest, indexNext)
			if (square->isValid()) 
			{
				nextAgentPosition = m_currentPosition;
				this->setNextPosition(SquareFree(_space, nextAgentPosition)); // (indexBest, indexNext)
			}
			//this->setNextPosition(SquareFree(_space, nextAgentPosition));
		}// chiude il primo if
	}// switch
}

///
void Guard::moveToNextPosition()
{
	m_currentPayoff = 0.;
	Agent::moveToNextPosition();

	updateBattery(-LOSTBATTERY_PERSTEP);
}

//////////////////////////////////////////////////////////////////////////
std::set<std::shared_ptr<Square> > Guard::getVisibleSquares( std::shared_ptr<DiscretizedArea> _space )
{
	std::set<std::shared_ptr<Square> > result;
	std::vector<AreaCoordinate> l_coord = m_currentPosition.getCoverage(_space);

	for(size_t i = 0; i < l_coord.size(); ++i)
	{
		result.insert(_space->getSquare(l_coord[i]));

	}
	collectVisitedSquare(result);
	return result;
}

//////////////////////////////////////////////////////////////////////////
AgentPosition Guard::selectNextFeasiblePositionWithoutConstraint(std::shared_ptr<DiscretizedArea> _space)
{
	return Agent::selectRandomFeasibleAction(_space);
}

//////////////////////////////////////////////////////////////////////////
AgentPosition Guard::selectNextFeasiblePositionWithoutConstraint(std::shared_ptr<DiscretizedArea> _space, std::set<int> &_alreadyTested)
{
	AgentPosition result;
	// usa getstandardapproachableValidSquare con heading
	std::vector<AgentPosition> l_feasible = this->getFeasibleActions(_space); 

	std::vector< std::pair<AgentPosition, int> > l_notControlledFeasibleActions;
	for(size_t i = 0; i < l_feasible.size(); ++i) // scorre su tutte le azioni fattibili con heading
	{
		if( _alreadyTested.find(i) != _alreadyTested.end() )  //se trova un already tested uguale all'ultimo elemento(azione)
			continue;

		if( m_currentTrajectory.contains(l_feasible[i]) )
		{
			//std::cout << "An action already chosen has been selected!" << std::endl;
			continue;
		}
		// se nessuno dei due casi
		l_notControlledFeasibleActions.push_back( std::make_pair<>(l_feasible[i], i) );//AgentPosition, int
	}

	if (l_notControlledFeasibleActions.empty())
	{
		/*std::cout << "curr pos: x " << m_currentPosition.getPoint2D().coord(0) << "y " << m_currentPosition.getPoint2D().coord(1) << std::endl;
		std::cout << "next pos: x " << m_currentPosition.getPoint2D().coord(0) << "y " << m_currentPosition.getPoint2D().coord(1) << std::endl;
		std::cout << "heading " << m_currentPosition.m_heading << std::endl;*/
		return m_currentPosition;
	}
	else if(_space->isThereASink())
	{
		std::vector<int> l_distanceNearestSink = _space->distanceFromNearestSink(l_notControlledFeasibleActions);

		double l_mindist = IDSMath::Infinity;
		int l_mindistIndex = -1;

		for(size_t i = 0; i < l_distanceNearestSink.size(); ++i)
		{
			if ( l_distanceNearestSink[i] < l_mindist )
			{
				l_mindist = l_distanceNearestSink[i]; 
				l_mindistIndex = i;
			}
		}

		_alreadyTested.insert(l_notControlledFeasibleActions[l_mindistIndex].second);

		AgentPosition l_selectedPosition = l_notControlledFeasibleActions[l_mindistIndex].first;
		/*std::cout << "curr pos: x " << m_currentPosition.getPoint2D().coord(0) << "y " << m_currentPosition.getPoint2D().coord(1) << std::endl;
		std::cout << "next pos: x " << l_selectedPosition.getPoint2D().coord(0) << "y " << l_selectedPosition.getPoint2D().coord(1) << std::endl;
		std::cout << "heading " << l_selectedPosition.m_heading << std::endl;*/
		return l_selectedPosition;
	}
	else
	{
		//this->removeBestTrajectoryFromFeasible(l_feasible);

		int l_value = getRandomValue( int( l_notControlledFeasibleActions.size() ) );
		_alreadyTested.insert(l_notControlledFeasibleActions[l_value].second);

		AgentPosition l_selectedPosition = l_notControlledFeasibleActions[l_value].first;;
		/*std::cout << "curr pos: x " << m_currentPosition.getPoint2D().coord(0) << "y " << m_currentPosition.getPoint2D().coord(1) << std::endl;
		std::cout << "next pos: x " << l_selectedPosition.getPoint2D().coord(0) << "y " << l_selectedPosition.getPoint2D().coord(1) << std::endl;
		std::cout << "heading " << l_selectedPosition.m_heading << std::endl;*/
		return l_selectedPosition;
	}
}

//////////////////////////////////////////////////////////////////////////
AgentPosition Guard::selectNextFeasiblePosition(std::shared_ptr<DiscretizedArea> _space)
{
	// Feasible action è un azione di una traiettoria ancora da definire ma che deve rispettare i vincoli degli algoritmi dinamici.
	// Pesco il riquadro finchè non trovo un elemento accettabile!

	if(!this->isRunning()) // la sua traiettoria non ha più di un passo o è al massimo consentito(all'inizio)
	{
		AgentPosition l_selectedPosition = this->selectNextFeasiblePositionWithoutConstraint(_space); // seleziona una posizione random tra le possibili

		/*std::cout << "curr pos: x = " << m_currentPosition.getPoint2D().coord(0) << "y = " << m_currentPosition.getPoint2D().coord(1) << " heading: " << m_currentPosition.m_heading << std::endl;
		std::cout << "next pos selected: x = " << l_selectedPosition.getPoint2D().coord(0) << "y = " << l_selectedPosition.getPoint2D().coord(1) << " heading: " << l_selectedPosition.m_heading << std::endl;*/

		return l_selectedPosition;
	}
	
	AgentPosition l_selectedPosition = this->getCurrentPosition();
	SquarePtr l_source = _space->getSquare(m_currentTrajectory.getPosition(0).getPoint2D()); // point of the first element of m_elem

	int l_nodeDistance = 0;
	int k = 0;
	std::set<int> l_alreadyTested;
	do
	{
		l_selectedPosition = this->selectNextFeasiblePositionWithoutConstraint(_space, l_alreadyTested); // AgentPosition
		
		//if( (k < 8) && (m_currentTrajectory.contains(l_selectedPosition)) )
		//	continue;

		/*std::cout << "curr pos: x = " << m_currentPosition.getPoint2D().coord(0) << "y = " << m_currentPosition.getPoint2D().coord(1) << " heading: " << m_currentPosition.m_heading << std::endl;
		std::cout << "next pos selected: x = " << l_selectedPosition.getPoint2D().coord(0) << "y = " << l_selectedPosition.getPoint2D().coord(1) << " heading: " << l_selectedPosition.m_heading << std::endl;*/

		SquarePtr l_selected = _space->getSquare( l_selectedPosition.getPoint2D() );

		l_nodeDistance = _space->getDistance(l_source, l_selected); // distanza percorsa dall' agente
		k++;
	} 
	while(m_maxTrajectoryLength - m_currentTrajectory.size() < l_nodeDistance && k < 9);

	return l_selectedPosition;
}

void Guard::receiveMessage( std::set<std::shared_ptr<Square> > const& _visible)
{
	return; // CONTROLLARE
}

//////////////////////////////////////////////////////////////////////////
void Guard::setCurrentPayoff(double _benefit)
{
	m_currentPayoff = _benefit;

	m_currentTrajectory.add(m_currentPosition);
	m_currentTrajectoryPayoff += m_currentPayoff;
}

//////////////////////////////////////////////////////////////////////////
void Guard::setCurrentMood(Mood _state)
{
	m_currentMood = _state;
}

//////////////////////////////////////////////////////////////////////////
int Guard::actualActionIndex()
{
	return m_currentTrajectory.size();
}

//////////////////////////////////////////////////////////////////////////
int Guard::totalActions()
{
	return m_maxTrajectoryLength;
}

//////////////////////////////////////////////////////////////////////////
std::set< std::shared_ptr<Square> > Guard::getTrajectoryCoverage() const
{
	return m_oldCoverage;
}

//////////////////////////////////////////////////////////////////////////
void Guard::collectVisitedSquare(std::set<SquarePtr>const& _squares)
{
	m_coverage.insert(_squares.begin(), _squares.end());
}

//////////////////////////////////////////////////////////////////////////
void Guard::updateBattery(double value)
{
	m_current_battery += value;

	if(m_current_battery < 0)
		m_current_battery = 0;

	if(m_current_battery > m_maximum_battery)
		m_current_battery = m_maximum_battery;
}

//////////////////////////////////////////////////////////////////////////
void Guard::updatePeriod(int value)
{
	m_maxTrajectoryLength = value;
}

const double MAXIMUM_PERIOD = std::max(double(Robotics::GameTheory::DISCRETIZATION_COL), double(Robotics::GameTheory::DISCRETIZATION_ROW));

//////////////////////////////////////////////////////////////////////////
int Guard::computePeriod()
{
	int l_period = (m_minimum_battery - m_current_battery) / m_minimum_battery * MAXIMUM_PERIOD;
	if ( l_period < 1 )
		l_period = 1;
	
	return l_period;
}

//////////////////////////////////////////////////////////////////////////
double Guard::computeBatteryCosts(std::shared_ptr<DiscretizedArea> _space)
{
	const double l_gain = 1.;
	double l_distance = _space->getDistanceFromNearestSink(m_currentPosition.getPoint2D());
	
	double l_param = l_distance / std::max(double(Robotics::GameTheory::DISCRETIZATION_COL), double(Robotics::GameTheory::DISCRETIZATION_ROW));

	return l_gain * ( m_maxTrajectoryLength - 1 ) * l_param;
}

//////////////////////////////////////////////////////////////////////////
void Guard::resetMemory()
{
	m_exploring=-1;
	m_memory.reset();
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
AgentPosition MemoryGuardTrajectories::getNextPosition(int _indexBest, int _indexNext)
{
	return m_elems.at(_indexBest).m_memTrajectory.getPosition(_indexNext);
}

//////////////////////////////////////////////////////////////////////////
double MemoryGuardTrajectories::getDeltaMemoryBenefit()
{
	if(m_elems.size() == 0)
		return 0.;

	//computeBestWorstTrajectories();

	return (m_best > m_worst ? -1. : 1.) * (m_elems[m_best].m_payoff - m_elems[m_worst].m_payoff); // calcola il delta del PIPIP
}

//////////////////////////////////////////////////////////////////////////
void MemoryGuardTrajectories::reset()
{
	m_best = -1;
	m_worst = -1;
	m_elems.clear();
}

#pragma endregion