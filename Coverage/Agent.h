///////////////////////////////////////////////////////////
//  Agent.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef AGENT_H
#define AGENT_H
#pragma once

//	Coverage
#include "CoverageExport.h"

// GeometricKernel
#include "BaseGeometry/Point2D.h"
#include "BaseGeometry/Shape2D.h"
#include "BaseGeometry/Line2D.h"
#include "BaseGeometry/Arc2D.h"
#include "BaseGeometry/MakeArc2D.h"

//	IDSBaseMath
#include "IDSBasemath/IDSMath.h"

#include <memory>
#include <set>

// set member function added on 24 April0 2016
// get member functions added on March 2016

struct v_pos
{
	double x;
	double y;
	double theta;
};

namespace Robotics 
{
	namespace GameTheory 
	{
		class DiscretizedArea;
		struct AreaCoordinate;
		class Area;
		class Agent;
		class Sink;
		class Square;

		//////////////////////////////////////////////////////////////////////////
		class COVERAGE_API CameraPosition
		{
		protected:
			/// Major radius.
			double m_farRadius;
			/// Minor Radius.
			double m_nearRadius;
			/// Orientation.
			double m_orientation; // [0, 2*Pi)
			/// Angle of view.
			double m_angle;

		public:
			CameraPosition(double _farRadius = 0., double _nearRadius = 0., double _orientation = 0., double _angle = IDSMath::TwoPi/4) 
				: m_farRadius(_farRadius), m_nearRadius(_nearRadius), m_orientation(_orientation), m_angle(_angle) {}


			//std::vector<AreaCoordinate> getCoverage(AreaCoordinate _center, double heading, std::shared_ptr<DiscretizedArea> _area) const;
			
			std::vector<IDS::BaseGeometry::Point2D> getVisibleArcPoints(AreaCoordinate _center, std::shared_ptr<DiscretizedArea> _area) const;
			//aggiunta
			//std::vector<Point2D> getVisibleArcPoints(AreaCoordinate _center, std::shared_ptr<DiscretizedArea> _area) const;

			double getFarRadius() const {return m_farRadius;}
			//aggiunti
			double getNearRadius() const { return m_nearRadius;}
			double getOrientation() const { return m_orientation;}
			double getAngle() const { return m_angle;}

			// set member functions added
			void setOrientation(double _orientation) { m_orientation = _orientation; }

			IDS::BaseGeometry::Shape2D getVisibleArea(IDS::BaseGeometry::Point2D const& point) const;

			std::vector<AreaCoordinate> getCoverage(AreaCoordinate _center, std::shared_ptr<DiscretizedArea> _area) const;

			IDS::BaseGeometry::Arc2D getVisibleArcArea(IDS::BaseGeometry::Line2D const &points, double const &radius, double const &angle) const;
			
			IDS::BaseGeometry::Shape2D getVisibleNearArea(IDS::BaseGeometry::Point2D const& point) const; //aggiunta
			///////// aggiunta
			IDS::BaseGeometry::Shape2D getVisibleArea(IDS::BaseGeometry::Point2D const& point, std::shared_ptr<DiscretizedArea> _area) const;
			///////


			double computeCosts() const {return 0.;}

			bool operator==(CameraPosition const& other) const;
			bool operator!=(CameraPosition const& other) const;
		};

		//////////////////////////////////////////////////////////////////////////
		class COVERAGE_API AgentPosition
		{
		protected:
			/// The position of the agent.
			IDS::BaseGeometry::Point2D m_point;
			// The heading of the agent
			double m_heading;

			/// The camera orientation.
			CameraPosition m_camera;
		

		public:
			AgentPosition() {}; //costruttore

			AgentPosition(IDS::BaseGeometry::Point2D const& point) : m_point(point), m_heading(0), m_camera() {}
		
			AgentPosition(IDS::BaseGeometry::Point2D const& point, double const& heading) : m_point(point), m_heading(heading), m_camera() {}

			AgentPosition(IDS::BaseGeometry::Point2D const& point, double const& heading, CameraPosition _camera) : m_point(point), m_heading(heading), m_camera(_camera) 
			{
				this->setOrientationCamera(heading);
				//_camera.setOrientation(heading);
			}


			double getHeading() { return m_heading; }

			void setHeading(AgentPosition _agentPosition, double heading) { _agentPosition.m_heading = heading; }
			// agg
			void setOrientationCamera(double const& heading) { m_camera.setOrientation(heading); }
			//auto ProbabilityOfDetection(std::shared_ptr<DiscretizedArea> area, AreaCoordinate p_r, AreaCoordinate p_t);

			/// Update the counter of the lattice visible from that position
			void updateCounter(std::shared_ptr<DiscretizedArea> area);

			/// Get Point2D
			IDS::BaseGeometry::Point2D getPoint2D() const {return m_point;}
			CameraPosition getCamera() const { return m_camera; }

			//IDS::BaseGeometry::Arc2D getVisibleArcArea(IDS::BaseGeometry::Line2D const &line, double const &radius, double const &angle) const;

			/// True if other is in communication with this position
			bool communicable(std::shared_ptr<Agent> _other) const;

			/// is the center of the square visible in that position and that camera?
			bool visible(std::shared_ptr<Square> _area) const;
			
			/////////////////////aggiunta//////////////
			bool visible(std::shared_ptr<Square> _area, std::shared_ptr<DiscretizedArea> _space) const;

			/// Compute Camera Costs
			double computeCosts() const;

			std::vector<AreaCoordinate> getCoverage(std::shared_ptr<DiscretizedArea> _space ) const;

			IDS::BaseGeometry::Shape2D getVisibleArea() const;
			//////
			
			IDS::BaseGeometry::Shape2D getVisibleArea(std::shared_ptr<DiscretizedArea> _space);
			std::vector<IDS::BaseGeometry::Point2D> getVisibleArcPointsnull(std::shared_ptr<DiscretizedArea> _space, std::shared_ptr<DiscretizedArea> _area);

			////
			std::vector<IDS::BaseGeometry::Point2D> getVisibleArcPoints();
			

			std::vector<IDS::BaseGeometry::Point2D> getVisibleArcPointsnull();

			bool operator==(AgentPosition const& other) const;
			bool operator!=(AgentPosition const& other) const;

			friend class COVERAGE_API Agent;
			friend class COVERAGE_API Guard;
			friend class COVERAGE_API Thief;
			friend class COVERAGE_API Sink;
		};

		//////////////////////////////////////////////////////////////////////////
		class COVERAGE_API Agent : public std::enable_shared_from_this<Agent>
		{
		protected:
			/// Agent Identifier
			int m_id;

			AgentPosition m_oldPosition;
			AgentPosition m_currentPosition;
			AgentPosition m_nextPosition;

			mutable enum Status 
			{
				ACTIVE,
				DISABLE,
				STANDBY,
				WAKEUP
			} m_status;

		public:

			Agent(int _id, AgentPosition _position)
				: m_id(_id)
				, m_currentPosition(_position)
				, m_nextPosition()
					, m_status(ACTIVE)
			{}

			~Agent() {}

			//aggiunta

			/// Get the position of the agent.


			inline AgentPosition getCurrentPosition() const {return m_currentPosition;}
			inline AgentPosition getNextPosition() const { return m_nextPosition; }

			/// Set Current Position
			void setCurrentPosition(AgentPosition const& _pos);

			/// Set Current Position
			void setNextPosition(AgentPosition const& _pos);
			// orientation of camera is coincident with the heading of robot
			//void setOrientation(double& heading) { m_currentPosition.m_camera.setOrientation(heading); }

			// ultime due posizioni (aggiunta)
			//double getHeadingRobot(IDS::BaseGeometry::Point2D _point);


			/// True if the Agent is active, false otherwise.
			bool isActive();

			/// True if the agent is next to be put on Stand-By, false otherwise.
			bool isOutOfInterest( std::shared_ptr<DiscretizedArea> space) const;

			/// Set the agent on standBy status. 
			void sleep();

			/// Set the agent on wakeUp status. 
			void wakeUp();

			virtual bool isThief() const {return false;}
			
			virtual bool isGuard() const {return false;}

			virtual bool isSink() const {return false;}

			virtual bool isNeutral() const {return false;}

			std::shared_ptr<Thief> toThief();

			std::shared_ptr<Guard> toGuard();

			std::shared_ptr<Sink> toSink();

			virtual void moveToNextPosition();

			virtual std::vector<AgentPosition> getFeasibleActions( std::shared_ptr<DiscretizedArea> _space ) const;
			//virtual std::vector<AgentPosition> getFeasibleActionsThief(std::shared_ptr<DiscretizedArea> _space) const;


			bool getRandomFeasibleAction(std::vector<AgentPosition> const& _feasible, AgentPosition & _pos) const;
			// aggiunta
			AgentPosition selectRandomFeasibleAction(std::shared_ptr<DiscretizedArea> _space, AgentPosition _thief);

			AgentPosition selectRandomFeasibleAction(std::shared_ptr<DiscretizedArea> _space);

			// aggiunta
			std::vector<AgentPosition> getFeasibleActionsThief(std::shared_ptr<DiscretizedArea> _space, AgentPosition _thief) const;
			AgentPosition selectRandomFeasibleActionThief(std::shared_ptr<DiscretizedArea> _space, AgentPosition _thief);
			//AgentPosition selectRandomFeasibleActionThief(std::shared_ptr<DiscretizedArea> _space);

			bool equals(std::shared_ptr<Agent>) const;

			inline IDS::BaseGeometry::Shape2D getVisibleArea() const {return m_currentPosition.getVisibleArea();}
			//aggiunto
			//inline IDS::BaseGeometry::Arc2D getVisibleArcArea() const { return m_currentPosition.getVisibleArcArea(); }

			CameraPosition getCurrentCamera() { return m_currentPosition.m_camera; }

		protected:
			Status getStatus() const;
			void setStatus(Status stat);

			//std::vector<AgentPosition> getFeasibleActions(std::shared_ptr<DiscretizedArea> _space);

		};

		typedef std::shared_ptr<Agent> AgentPtr;
		//aggiunto
		typedef std::shared_ptr<Guard> GuardPtr;


		class COVERAGE_API AgentActionIndex
		{
		public:
			int m_elem;
			int m_total;

			AgentActionIndex( int _elem, int _total ) : m_elem(_elem), m_total(_total) {}
		};
	}
}
#endif