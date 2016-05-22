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
	
	std::vector<AreaCoordinate> l_cover = m_camera.getCoverage(l_coord, m_heading, area);

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
	return m_camera.getCoverage(l_center, m_heading, _space); // ho aggiunto argomenti
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
}

double modulo2pi(double angle) {

	if (angle < 0)	return angle = angle + IDSMath::TwoPi;
	if (angle > IDSMath::TwoPi)	 return angle = angle - IDSMath::TwoPi;
	if (angle >= 0 && angle <= IDSMath::TwoPi)	return angle;

}

double modulo360(double angle) {

	if (angle < 0)	return angle = angle + 360;
	if (angle > 360)	 return angle = angle - 360;
	if (angle >= 0 && angle <= 360)	return angle;
}

/*if (alpha_line > 360 && turned_f == false && turned_l == true) {
alpha_line += 360;
alpha_finish += 360;
turned_f = true;
}*/
/*if (alpha_line < 0 && turned_l == false){
alpha_line += 360;
alpha_finish += 360;
turned_l = true;
}*/

// sample the point of line with Rstep sensor resolution
/*if (alpha_line >= 180 && alpha_line <= 360) // sin positivo
{
x = x0 + d*cos(alpha);
y = y0 + d*sin(alpha);
}
if (alpha_line >= 0 && alpha_line < 180) // sin negativo
{
x = x0 + d*cos(alpha);
y = y0 + d*sin(alpha);
}
if(alpha_line < 0 && alpha_line >= -180)
{
x = x0 + d*cos(alpha);
y = y0 + d*sin(alpha);
}
if (alpha_line < -180 && alpha_line >= -360)
{
x = x0 + d*cos(alpha);
y = y0 - d*sin(alpha);
}*/

//////////////////////////////////////////////////////////////////////////
std::vector<AreaCoordinate> CameraPosition::getCoverage(AreaCoordinate _center, double heading, std::shared_ptr<DiscretizedArea> _area) const
{
	int l_rowDelta = int(floor(m_farRadius / _area->getYStep())) + 1;
	int l_colDelta = int(floor(m_nearRadius / _area->getXStep())) + 1;
	double oo = m_nearRadius;

	std::vector<AreaCoordinate> result;
	std::vector<AreaCoordinate> tmp;
	AreaCoordinate element;

	// center(x0, y0)
	int x0, y0, alpha_line, Rmin, Rmax, Rstep;
	x0 = _center.col;
	y0 = _center.row;
	Rmin = 2; //m_nearRadius/_area->getYSTep()+_area->getXStep();
	Rmax = l_colDelta+1; //int(l_colDelta + l_rowDelta / 2);
	Rstep = 1; //Discretized radar resolution

	int d, x, y;
	float alpha;
	// in this case heading of robot is considered coincident with the heading of camera
	double h = heading; // heading che va preso con un range da 0 a 360 ( 0 = 360, considerato una volta sola)
	double fov = m_angle; // feed of view
	double alpha_begin = h - fov / 2;
	double alpha_finish = h + fov / 2;
	double angle_resolution = 2; // degrees

	try
	{
		Point2D l_pt = _area->getPosition(_center);
		Line2D line = makeLine(l_pt, h);
		Arc2D l_Arco = this->getVisibleArcArea(line, m_farRadius, m_angle);
		
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		system("pause");
	}
	
	//for (alpha_line = 270; alpha_line < 360; alpha_line = alpha_line + 2.5)
	for(alpha_line = alpha_begin; alpha_line < alpha_finish; alpha_line = alpha_line + angle_resolution )
	{
		for (d = Rmin; d <= Rmax; d += Rstep) //line scan
		{
			alpha = alpha_line * IDSMath::Pi / 180; // angle in rads
			
			x = x0 + d*cos(-alpha);
			y = y0 + d*sin(-alpha);

			// check if square goes outside area
			if (x <= 0 || x >= _area->getNumCol())
				continue;
			if (y <= 0 || y >= _area->getNumRow())
				continue;

				element.col = x;
				element.row = y;
				tmp.push_back(element);

				// find double element from the second step
				bool found = false;
				if (tmp.size() > 1)
				{
					for (int i = 0; i < tmp.size() -1 && found == false; i++)
					{
						if (tmp.at(i).col == x && tmp.at(i).row == y)
							found = true;
					}
				}
				// insert when double element has not been found
				if (found == false)	result.push_back(element);
				// mod 2*pi
				if (alpha_line == 360)	alpha_line = 0;
		}
	}
	//result = this->getVisibleArcPoints(_center, _area);
	//printArray(result, _center.row, _center.col);
	return result;
}

//// potrebbe essere usata al posto di gerVisibleArea in LearningAlgorithm.cpp manca la ,linescan del bordo arco ////
std::vector<IDS::BaseGeometry::Point2D> CameraPosition::getVisibleArcPoints(AreaCoordinate _center, std::shared_ptr<DiscretizedArea> _area) const
{
	int l_rowDelta = int(floor(m_farRadius / _area->getYStep())) + 1;
	int l_colDelta = int(floor(m_farRadius / _area->getXStep())) + 1;

	AreaCoordinate element;
	std::vector<AreaCoordinate> result;
	std::vector<AreaCoordinate> tmp;

	// center(x0, y0)
	int x0, y0, alpha_line, Rmin, Rmax, Rstep, d;
	x0 = _center.col;
	y0 = _center.row;
	Rmin = 2; //m_nearRadius/_area->getYSTep()+_area->getXStep();
	Rmax = l_colDelta + 1; //int(l_colDelta + l_rowDelta / 2);
	Rstep = 1; //Discretized radar resolution

	int x, y;
	double alpha, h, fov, alpha_begin, alpha_finish, angle_resolution;

	h = m_orientation; // heading che va preso con un range da 0 a 360 ( 0 = 360, considerato una volta sola)
	fov = m_angle; // feed of view
	alpha_begin = h - fov / 2;
	alpha_finish = h + fov / 2;
	angle_resolution = 2; // degrees
	
	for (alpha_line = alpha_begin; alpha_line < alpha_finish; alpha_line = alpha_line + angle_resolution)
	{
		alpha = alpha_line * IDSMath::Pi / 180; // angle in rads
		// It samples the points of bounds
		x = x0 + Rmax*cos(-alpha);
		y = y0 + Rmax*sin(-alpha);
		// check if the points go out the discretized area
		if (x <= 0 || x >= _area->getNumCol())
			continue;
		if (y <= 0 || y >= _area->getNumRow())
			continue;

		element.col = x;
		element.row = y;
		tmp.push_back(element);

		// find double element from the second step till the last one
		bool found = false;
		if (tmp.size() > 1)
		{
			for (int i = 0; i < tmp.size() - 1 && found == false; i++)
			{
				if (tmp.at(i).col == x && tmp.at(i).row == y)
					found = true;
			}
		}
		// insert when double element has not been found
		if (found == false)	result.push_back(element);
		// mod 2*pi
		if (alpha_line == 360)	alpha_line = 0;
	}
	// conrversion from AreaCcordinate to Point2D inspired to getCoordinate function
	std::vector<Point2D> points;
	for (int i = 0; i < result.size(); i++)
	{
		Point2D p = _area->getCoordinatePoint2D(result.at(i));
		points.push_back(p);
	}
	return points;
}

///////////////////////////////////////////////////////////////////// come prendo i punti dall'arco Arc2d?
BaseGeometry::Arc2D CameraPosition::getVisibleArcArea(BaseGeometry::Line2D const& line, double const& radius, double const& angle) const
try
{
	if (fabs(m_farRadius) < IDSMath::TOLERANCE)
		return Arc2D();

	Arc2D l_external = makeArc(line, radius, angle);
	return l_external;
}
catch (...)
{
	return Arc2D();
}

BaseGeometry::Shape2D CameraPosition::getVisibleNearArea(BaseGeometry::Point2D const& point) const
try
{
	if (fabs(m_nearRadius) < IDSMath::TOLERANCE)
		return Shape2D();

	Curve2D l_external = makeCircle(point, m_nearRadius, false);
	return Shape2D(std::vector<Curve2D>(1, l_external));
}
catch (...)
{
	return Shape2D();
}
//////////////////////////////////////////////////////////////////////////
BaseGeometry::Shape2D CameraPosition::getVisibleArea(BaseGeometry::Point2D const& point) const
	try
{
	if(fabs(m_farRadius) < IDSMath::TOLERANCE)
		return Shape2D();

	Curve2D l_external = makeCircle( point, m_farRadius, false);
	return Shape2D( std::vector<Curve2D>(1,l_external) );
}
catch (...)
{
	return Shape2D();
}



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
