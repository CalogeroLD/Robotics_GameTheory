#include "Agent.h"
#include "Agent.h"
#include "Agent.h"
#include "Guard.h"
#include "DiscretizedArea.h"

#include "BaseGeometry/Shape2D.h"
#include "BaseGeometry/Arc2D.h"
#include "BaseGeometry/Point2D.h"
#include "BaseGeometry/MakeArc2D.h"
#include "BaseGeometry/Line2D.h"
#include "BaseGeometry\Segment2D.h"


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace IDS;
using namespace IDS::BaseGeometry;

//////////////////////////////////////////////////////////////////////////
// MemoryGuardTrajectories
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
void MemoryGuardTrajectories::add(MemoryGuardTrajectory const& _elem)
{
	m_elems.insert(m_elems.begin(), _elem);

	if(m_elems.size() > size_t(m_maxSize))
		m_elems.erase(m_elems.begin()+m_maxSize);
}

//////////////////////////////////////////////////////////////////////////
void MemoryGuardTrajectories::computeBestWorstTrajectories()
{
	double l_bestPayoff = -IDSMath::Infinity;
	double l_worstPayoff = IDSMath::Infinity;
	for(size_t i = 0; i < m_elems.size(); ++i)
	{
		double l_currentPayoff = m_elems[i].m_payoff;

		if(l_currentPayoff > l_bestPayoff)
		{
			l_bestPayoff = l_currentPayoff;
			m_best= i;
		}

		if(l_currentPayoff < l_worstPayoff)
		{
			l_worstPayoff = l_currentPayoff;
			m_worst = i;
		}
	}
	return;
}

//////////////////////////////////////////////////////////////////////////
//	GuardTrajectoryPosition
//////////////////////////////////////////////////////////////////////////
bool GuardTrajectoryPosition::operator==(GuardTrajectoryPosition const& other) const
{
	return m_position == other.m_position && m_index == other.m_index;
}

bool GuardTrajectoryPosition::operator!=(GuardTrajectoryPosition const& other) const
{
	return !( *this == other );
}

//////////////////////////////////////////////////////////////////////////
//	GuardTrajectory
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
bool GuardTrajectory::contains(AgentPosition _nextPos)
{
	for(size_t i = 0; i < m_elems.size(); ++i)
	{
		if( m_elems[i].m_position.getPoint2D().equals(_nextPos.getPoint2D()) )
			return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////
bool GuardTrajectory::operator==(GuardTrajectory const& other) const
{
	if(m_elems.size() != other.m_elems.size())
		return false;

	for(size_t i = 0; i < m_elems.size(); ++i)
	{
		if(!(m_elems[i] == other.m_elems[i]) )
			return false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
bool GuardTrajectory::operator!=(GuardTrajectory const& other) const
{
	return !(*this == other);
}

//////////////////////////////////////////////////////////////////////////
//	AgentPosition
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::operator==(AgentPosition const& other) const
{
	return m_point.equals(other.m_point) && m_camera == other.m_camera;
}

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::operator!=(AgentPosition const& other) const
{
	return !(*this == other);
}

//////////////////////////////////////////////////////////////////////////
void AgentPosition::updateCounter(std::shared_ptr<DiscretizedArea> area)
{
	AreaCoordinate l_coord = area->getCoordinate(m_point);

	area->getSquare( l_coord.row, l_coord.col )->increaseCounter();

	// Calcolo la copertura in base alla camera:
	// per ora conta solo il raggio massimo e non cala con la distanza!
	
	std::vector<AreaCoordinate> l_cover = m_camera.getCoverage(l_coord, area);

	for(size_t i = 0; i < l_cover.size(); ++i)
	{
		std::shared_ptr<Square> l_square = area->getSquare( l_cover[i].row, l_cover[i].col );
		if(l_square)
			l_square->increaseCounter();
	}
}

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::communicable(std::shared_ptr<Agent> _otherAgent) const
{
	return m_point.distance( _otherAgent->getCurrentPosition().getPoint2D() ) < 2 * m_camera.getFarRadius() + IDSMath::TOLERANCE;
}

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::visible(std::shared_ptr<Square> _area) const
{
	Shape2D sh = m_camera.getVisibleArea(m_point);
	if(sh.isValid())
		return sh.contains( _area->getBoundingBox().center() );
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////
double AgentPosition::computeCosts() const
{
	return m_camera.computeCosts();
}

//////////////////////////////////////////////////////////////////////////
std::vector<AreaCoordinate> AgentPosition::getCoverage(std::shared_ptr<DiscretizedArea> _space ) const
{
	//double head = m_camera.getOrientation();
	//std::cout << "head:" << head << endl;
	AreaCoordinate l_center = _space->getCoordinate(m_point);
	return m_camera.getCoverage(l_center, _space); // ho aggiunto argomenti
}

//////////////////////////////////////////////////////////////////////////
IDS::BaseGeometry::Shape2D AgentPosition::getVisibleArea() const
{
	return m_camera.getVisibleArea(m_point);

}


//////////////////////////////////////////////////////////////////////////
//	CameraPosition
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
bool CameraPosition::operator==(CameraPosition const& other) const
{
	if(fabs(m_farRadius-other.m_farRadius) > IDSMath::TOLERANCE)
		return false;
	else if(fabs(m_nearRadius-other.m_nearRadius) > IDSMath::TOLERANCE)
		return false;
	else if(fabs(m_orientation-other.m_orientation) > IDSMath::TOLERANCE)
		return false;
	else if(fabs(m_angle-other.m_angle) > IDSMath::TOLERANCE)
		return false;
	return true;
}

//////////////////////////////////////////////////////////////////////////
bool CameraPosition::operator!=(CameraPosition const& other) const
{
	return !(*this==other);
}

void printArray(std::vector<AreaCoordinate> v, int c_row, int c_col) {
	for (int i = 0; i < v.size(); ++i) {
		double x = v.at(i).row;
		double y = v.at(i).col;
		double dist = sqrt(pow((x - c_row), 2) + pow((y - c_col), 2));
		std::cout << "col: " << v.at(i).col << " \t row: " << v.at(i).row << "\t prob: " << v.at(i).p << std::endl;//<< "\n d : " << dist << std::endl;
	}
	std::cout << "centro col: " << c_col << " \t  centro row: " << c_row << "\t prob: " << std::endl;//<< "\n d : " << dist << std::endl;
	std::cout << "size : " << v.size() << endl;
}


///////////////////////////////////////////////////////////////////////
std::vector<AreaCoordinate> CameraPosition::getCoverage(AreaCoordinate _center, std::shared_ptr<DiscretizedArea> _area) const
{
	Point2D l_pt = _area->getPosition(_center);
	Shape2D l_sensorArea = this->getVisibleArea(l_pt);

	if (!l_sensorArea.isValid())
		return std::vector<AreaCoordinate>();

	int l_rowDelta = int(floor(m_farRadius / _area->getYStep())) + 1;
	int l_colDelta = int(floor(m_farRadius / _area->getXStep())) + 1;

	/*std::cout << "l_rowDelta :" << l_rowDelta << endl;
	std::cout << "l_colDelta :" << l_colDelta << endl;
	std::cout << "heading :" << m_orientation << endl;
	std::cout << "farRadius :" << m_farRadius << endl;
	std::cout << "nearRadius :" << m_nearRadius << endl;*/

	std::vector<AreaCoordinate> result;
	AreaCoordinate l_elem;
	for (int i = -l_rowDelta; i <= l_rowDelta; ++i)
	{
		int row = _center.row + i;
		if (row < 0 || row > _area->getNumRow())
			continue;
		for (int j = -l_colDelta; j <= l_colDelta; ++j)
		{
			int col = _center.col + j;
			if (col < 0 || col > _area->getNumRow())
				continue;

			SquarePtr l_square = _area->getSquare(row, col);
			if (!l_square)
				continue;

			Point2D l_center = l_square->getBoundingBox().center();
			if (l_sensorArea.contains(l_center) )
			{
				l_elem.row = row;
				l_elem.col = col;
				result.push_back(l_elem);
			}
		}
	}

	if (result.size() == 0) {
		result.push_back(_center);
	}
	//printArray(result, _center.row, _center.col);
	return result;

}
////////////////////////////////////////////////////////////////////////////////////////////
/*BaseGeometry::Shape2D CameraPosition::getVisibleNearArea(BaseGeometry::Point2D const& point) const
try
{
	std::exception e;

	std::cout << e.what() << endl;
	if (fabs(m_nearRadius) < IDSMath::TOLERANCE)
		return Shape2D();

	Curve2D l_external = makeCircle(point, m_nearRadius, false);
	return Shape2D(std::vector<Curve2D>(1, l_external));
}
catch (...)
{
	return Shape2D();
}*/

//////////////////////////////////////////////////////////////////////////
BaseGeometry::Shape2D CameraPosition::getVisibleArea(BaseGeometry::Point2D const& point) const
try
{
	std::exception e;
	if (fabs(m_farRadius) < IDSMath::TOLERANCE)
		return Shape2D();

	if (fabs(m_angle) > IDSMath::TwoPi - IDSMath::TOLERANCE)
	{
		Curve2D l_external = makeCircle(point, m_farRadius, false);
		return Shape2D(std::vector<Curve2D>(1, l_external));
	}
	
	Line2D l_center_line = BaseGeometry::makeLine(point, IDSMath::toDeg(m_orientation) );
	Line2D l_left_line = l_center_line.rotate(IDSMath::toDeg(-m_angle) / 2.);
	Line2D l_right_line = l_center_line.rotate(IDSMath::toDeg(m_angle) / 2.);

	Point2D l_near_left = l_left_line.pointFromOrigin(m_nearRadius);
	Point2D l_near_right = l_right_line.pointFromOrigin(m_nearRadius);
	Point2D l_far_left = l_left_line.pointFromOrigin(m_farRadius);
	Point2D l_far_right = l_right_line.pointFromOrigin(m_farRadius);
	/*cout << "far" <<  m_farRadius << endl;
	cout << "near " << m_nearRadius << endl;
	cout << "head" << m_orientation << endl;*/

	std::vector<Curve2D> l_boundary;
	Segment2D l_first = BaseGeometry::makeSegment(l_near_right, l_far_right);
	l_boundary.push_back(l_first);
	Arc2D l_second = BaseGeometry::makeArc(point, l_far_right, l_far_left, false);
	l_boundary.push_back(l_second);
	Segment2D l_third = BaseGeometry::makeSegment(l_far_left, l_near_left);
	l_boundary.push_back(l_third);

	if (m_nearRadius > IDSMath::TOLERANCE)
	{
		Arc2D l_fourth = BaseGeometry::makeArc(point, l_near_left, l_near_right, true);
		l_boundary.push_back(l_fourth);
	}

	return Shape2D(l_boundary);
}
catch (...)
{
	return Shape2D();
}

//////////////////////////////////////////////////////////////////////////
/*BaseGeometry::Shape2D CameraPosition::getVisibleArea(BaseGeometry::Point2D const& point, std::shared_ptr<DiscretizedArea> _area) const
	try
{
	std::exception e;
	if(fabs(m_farRadius) < IDSMath::TOLERANCE)
		return Shape2D();

	//AreaCoordinate _center = _area->getCoordinate(point);

	Line2D l_center_line = BaseGeometry::makeLine(point, m_orientation);

	Line2D l_left_line = l_center_line.rotate(m_angle / 2.);
	Line2D l_right_line = l_center_line.rotate(-m_angle / 2.);

	Point2D l_near_left = l_left_line.pointFromOrigin(m_nearRadius);
	Point2D l_near_right = l_right_line.pointFromOrigin(m_nearRadius);
	Point2D l_far_left = l_left_line.pointFromOrigin(m_farRadius);
	Point2D l_far_right = l_right_line.pointFromOrigin(m_farRadius);

	std::vector<Curve2D> l_boundary;
	Segment2D l_first = BaseGeometry::makeSegment(l_near_right, l_far_right);
	l_boundary.push_back(l_first);
	Arc2D l_second = BaseGeometry::makeArc(point, l_far_right, l_far_left, false);
	l_boundary.push_back(l_second);
	Segment2D l_third = BaseGeometry::makeSegment(l_far_left, l_near_left);
	l_boundary.push_back(l_third);
	Arc2D l_fourth = BaseGeometry::makeArc(point, l_near_left, l_near_right, true);
	l_boundary.push_back(l_fourth);

	//std::vector<BaseGeometry::Point2D> vectorOfPoints = this->getVisibleArcPoints(_center, _area);
	//Curve2D l_external = makeCircle( point, m_farRadius, false);
	return Shape2D(l_boundary);

	//std::cout << e.what() << endl;
	//return Shape2D(vectorOfPoints, true);
}
catch (...)
{
	return Shape2D();
}*/



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
/*						MemoryGuardTrajectory							*/
//////////////////////////////////////////////////////////////////////////

bool MemoryGuardTrajectory::equals(MemoryGuardTrajectory const& _other) const
{
	if(this->m_memTrajectory != _other.m_memTrajectory)
		return false;

	if(fabs( this->m_payoff - _other.m_payoff ) > IDSMath::TOLERANCE )
		return false;

	return true;
}

bool MemoryGuardTrajectory::equals(GuardTrajectory const& _trajectory, double _payoff) const
{
	if(this->m_memTrajectory != _trajectory)
		return false;

	if(fabs( this->m_payoff - _payoff ) > IDSMath::TOLERANCE )
		return false;

	return true;
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
