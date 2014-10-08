#include "World.h"
#include "Agent.h"
#include "Neutral.h"
#include "Thief.h"
#include "DiscretizedArea.h"
#include "CoverageUtility.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;
using namespace IDS::BaseGeometry;

//////////////////////////////////////////////////////////////////////////
World::World(std::set< std::shared_ptr<Agent> > _agent, std::shared_ptr<DiscretizedArea> _space) 
	: m_space(_space)
	, m_agent(_agent)
{
	for(int i = 0; i < MAX_NUMBER_NEUTRAL_AGENT; ++i)
	{
		Point2D point;
		_space->getRandomPosition(point);
		m_agent.insert( std::make_shared<Neutral>( m_agent.size(), point) );
	}
}

//////////////////////////////////////////////////////////////////////////
World::World(std::set< std::shared_ptr<Agent> > _agent, std::shared_ptr<Area> _space) 
	: m_space(nullptr)
	, m_agent(_agent)
{
	m_space = _space->discretize();

	for(int i = 0; i < MAX_NUMBER_NEUTRAL_AGENT; ++i)
	{
		m_agent.insert( std::make_shared<Neutral>( m_agent.size(), _space->randomPosition() ) );
	}
}

//////////////////////////////////////////////////////////////////////////
std::set< std::shared_ptr<Guard> > World::getGuards() const
{
	std::set< std::shared_ptr<Guard> > result;
	for(auto it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if(l_agent->isGuard())
			result.insert(l_agent->toGuard());
	}
	return result;
}

//////////////////////////////////////////////////////////////////////////
std::set< std::shared_ptr<Thief> > World::getThieves() const
{
	std::set< std::shared_ptr<Thief> > result;
	for(auto it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if(l_agent->isThief())
			result.insert(l_agent->toThief());
	}
	return result;
}

//////////////////////////////////////////////////////////////////////////
void World::removeAllThieves()
{
	std::set< std::shared_ptr<Thief> > result = getThieves();

	for(auto it = result.begin(); it != result.end(); ++it)
	{
		m_agent.erase(*it);
	}
	return;
}

//////////////////////////////////////////////////////////////////////////
std::set< std::shared_ptr<Agent> > World::getNeutrals() const
{
	std::set< std::shared_ptr<Agent> > result;
	for(auto it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if(!l_agent->isGuard() && !l_agent->isThief())
			result.insert(l_agent);
	}
	return result;
}

//////////////////////////////////////////////////////////////////////////
void World::moveThieves(int _thiefJump)
{
	for(auto it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if(l_agent->isActive())
		{
			if(l_agent->isThief())
			{
				for(int j = 0; j < _thiefJump; ++j)
				{
					l_agent->setNextPosition(l_agent->selectRandomFeasibleAction(m_space));
					l_agent->moveToNextPosition();
				}
			}
			else if( l_agent->isOutOfInterest(m_space) )
				l_agent->sleep();
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void World::wakeUpAgentIfSecurityIsLow()
{
	for(std::set< AgentPtr >::iterator it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if(!l_agent->isActive() && m_space->isSecurityLow())
		{
			l_agent->wakeUp();
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void World::randomInitializeGuards()
{
	/// Put agent in the space
	for(std::set< std::shared_ptr<Agent> >::iterator it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if(!l_agent->isGuard())
			continue;

		// compute position of agent and camera:
		Point2D l_point;
		if(m_space->getRandomPosition(l_point))
			// set position of the agent:
			l_agent->setCurrentPosition( AgentPosition( l_point, CameraPosition( ((m_space->getXStep() +  m_space->getYStep()) / 2) * 3 ) ) );
	}
}

//////////////////////////////////////////////////////////////////////////
void World::randomInitializeNeutrals()
{
	/// Put agent in the space
	for(std::set< std::shared_ptr<Agent> >::iterator it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if( l_agent->isThief() || l_agent->isGuard() )
			continue;

		Point2D l_point;
		if(m_space->getRandomPosition(l_point))
			// set position of the agent:
			l_agent->setCurrentPosition( AgentPosition( l_point, CameraPosition( ) ) );
	}
}

//////////////////////////////////////////////////////////////////////////
void World::randomInitializeThief()
{
	/// Put agent in the space
	for(std::set< std::shared_ptr<Agent> >::iterator it = m_agent.begin(); it != m_agent.end(); ++it)
	{
		AgentPtr l_agent = *it;
		if(!l_agent->isThief())
			continue;

		Point2D l_point;
		if(m_space->getRandomPosition(l_point))
			// set position of the agent:
			l_agent->setCurrentPosition( AgentPosition( l_point, CameraPosition( ) ) );
	}
}

//////////////////////////////////////////////////////////////////////////
int World::getNumberOfAgent()
{
	return m_agent.size();
}

///
void World::addThief(std::shared_ptr<Thief> _thief)
{
	m_agent.insert(_thief);
}

//////////////////////////////////////////////////////////////////////////
double World::getMaximumValue()
{
	std::set<ThiefPtr> l_thieves = this->getThieves();
	for(auto it = l_thieves.begin(); it != l_thieves.end(); ++it)
	{
		return this->getSpace()->getThiefMaxValue( (*it)->getCurrentPosition() );
	}
	return 0.;
}
