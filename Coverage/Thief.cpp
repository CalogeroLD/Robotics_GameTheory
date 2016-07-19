
#include "Agent.h"
#include "Area.h"
#include "DiscretizedArea.h"
#include "Guard.h"
#include "Thief.h"
#include "Sink.h"
#include "Probability.h"
#include "World.h"
#include "CoverageAlgorithm.h"
#include "LearningAlgorithm.h"

#include <fstream>

#include <memory>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace IDS;
using namespace IDS::BaseGeometry;

std::vector<AgentPosition> Thief::getFeasibleActionsThief(std::shared_ptr<DiscretizedArea> _space, AgentPosition _thief) const
{
	AreaCoordinate l_thiefCoord = _space->getCoordinate(_thief.getPoint2D()); // prende punto camera
	std::vector<AreaCoordinate> l_squares = _space->getStandardApproachableValidSquaresThief(l_thiefCoord); // prende 8 punti adiacenti ABCDEFG
																											// aggiunge le diagonali al robot
	_space->addSpecialApproachableValidSquares(l_thiefCoord, l_squares);
	std::vector<AgentPosition> l_result;
	for (size_t i = 0; i < l_squares.size(); ++i)
	{
		l_result.push_back(AgentPosition(_space->getPosition(l_squares[i]), l_squares[i].heading, m_currentPosition.m_camera));
	}
	return l_result; // return vector of all ThievesPosition possible
}

AgentPosition Thief::selectRandomFeasibleActionThief(std::shared_ptr<DiscretizedArea> _space, AgentPosition _thief)
{
	std::vector<AgentPosition> l_feasible = this->getFeasibleActionsThief(_space, _thief); // tutte le posizioni possibili in base a dove mi trovo
	if (l_feasible.empty()) // if is empty return currentPosition
		return m_currentPosition;
	else
	{
		int l_value = getRandomValue(int(l_feasible.size())); // ne prende una a caso in posizione l_value
		return l_feasible[l_value];
	}
}