#include "DiscretizedArea.h"
#include "CoverageUtility.h"
#include "StructuredArea.h"
#include "UnStructuredArea.h"
#include "Agent.h"
#include "Graph.h"

#include "BaseGeometry/Line2D.h"
#include "BaseGeometry/Point2D.h"
#include "BaseGeometry/Shape2D.h"
#include "BaseGeometry/MakePoint2D.h"
#include "BaseGeometry/Box2D.h"
#include "BaseGeometry/BaseGeometry.h"

#include "BaseGeometry/BaseGeometryTypes.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <fstream>
#include <iostream>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace IDS;
using namespace IDS::BaseGeometry;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////////		DISCRETIZED AREA		//////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
Graph DiscretizedArea::getGraph() const
{
	Graph graph;
	return graph;
}

//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::updateSquaresCounter(AgentPtr agent)
{
	AgentPosition l_position = agent->getCurrentPosition();
	l_position.updateCounter( this->shared_from_this() );
}

//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::resetCounter()
{
	for(size_t i = 0; i < m_lattice.size(); ++i)
		m_lattice[i]->resetCounter();
}

//////////////////////////////////////////////////////////////////////////
SquarePtr DiscretizedArea::getSquare(AreaCoordinate _coord) const
{
	return getSquare(_coord.row, _coord.col);
}

//////////////////////////////////////////////////////////////////////////
SquarePtr DiscretizedArea::getSquare(int row, int col) const
{
	if( row >= m_numRow || row < 0 ||
		col >= m_numCol || col < 0 )
		return nullptr;

	return m_lattice.at(row * m_numCol + col);//accede all'elemento del vettore con (row, col) se esiste
}

//////////////////////////////////////////////////////////////////////////
bool DiscretizedArea::getRandomPosition(IDS::BaseGeometry::Point2D & _point) const
{
	/// Compute Random Position:
	srand ( (unsigned int) time(NULL) );

	int k = 0;
	do
	{
		int l_colSecret = rand() % m_numCol;
		int l_rowSecret = rand() % m_numRow;

		SquarePtr l_square	= this->getSquare(l_rowSecret, l_colSecret);

		if(l_square->isValid())
		{
			_point = l_square->getBoundingBox().center();
			return true;
		}
		else
			continue;

		++k;
	} 
	while (k < 100);

	return false;
}

//////////////////////////////////////////////////////////////////////////
bool DiscretizedArea::isOut(AgentPosition const& _pos) const
{
	Point2D l_point = _pos.getPoint2D();
	AreaCoordinate l_coord = this->getCoordinate( l_point );
	SquarePtr l_square = this->getSquare(l_coord);
	if(!l_square)
		return false;
	return !l_square->isValid();
}

//////////////////////////////////////////////////////////////////////////
std::vector<SquarePtr> DiscretizedArea::getSquares() const
{
	return m_lattice;
}

//////////////////////////////////////////////////////////////////////////
// String Tokenizer
void tokenize_(const std::string& str, const std::string& separators, std::vector<std::string> &tokens)
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

//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::addEdges()
{
	for(size_t i = 0; i < m_lattice.size(); ++i)
	{
		int row = i / m_numCol;
		int col = i % m_numCol;

		if( row >= m_numRow || row < 0 ||
			col >= m_numCol || col < 0 )
			//	out of bound
			continue;

		SquarePtr l_square = m_lattice.at(row * m_numCol + col);

		if(!l_square->isValid())
			continue;

		lemon::ListGraph::Node l_currentNode = l_square->getNode();

		SquarePtr l_sq = this->getSquare(row, col-1);
		if( l_sq && l_sq->isValid() )
			m_listGraph->addEdge( l_currentNode, l_sq->getNode() );

		l_sq = this->getSquare(row-1, col-1);
		if( l_sq && l_sq->isValid() )
			m_listGraph->addEdge( l_currentNode, l_sq->getNode() );

		l_sq = this->getSquare(row-1, col);
		if( l_sq && l_sq->isValid() )
			m_listGraph->addEdge( l_currentNode, l_sq->getNode() );

		l_sq = this->getSquare(row-1, col+1);
		if( l_sq && l_sq->isValid() )
			m_listGraph->addEdge( l_currentNode, l_sq->getNode() );
	}
}

////////////////////////////////////////////////////////////////////////// da FILE
DiscretizedArea::DiscretizedArea(std::string const& _filename) 
	: m_graph(nullptr)
	, m_listGraph(nullptr)
	, m_numberOfValidSquare(-1)
	, m_sinkCoordinate(-1,-1)
{
	std::ifstream iFile(_filename);	// input.txt has integers, one per line
#ifdef _PRINT
	cout << "Filename " << _filename << endl;
#endif
	if(!iFile.is_open())
	{
		cout << "Unable to open Grid File! " << _filename << endl;
		throw std::exception("Unable to open Grid File!");
	}

	Point2D l_bottomLeft;
	Point2D	l_bottomRight;
	Point2D	l_topLeft;
	Point2D	l_topRight;

	std::vector<std::vector<bool>> l_grid;
	int l_numcol(1), l_numrow(1);
	if(iFile.is_open())
	{
		int numOfXXX;
		iFile >> numOfXXX; //vertices
		for(int i = 0; i < numOfXXX; ++i)
		{
			double x, y;
			iFile >> x;
			iFile >> y;

			switch(i)
			{
			case 0:
				l_topLeft = makePoint(IDSReal2D(x,y), EucMetric);
				break;
			case 1:
				l_topRight = makePoint(IDSReal2D(x,y), EucMetric);
				break;
			case 2:
				l_bottomRight = makePoint(IDSReal2D(x,y), EucMetric);
				break;
			case 3:
				l_bottomLeft = makePoint(IDSReal2D(x,y), EucMetric);
				break;
			}
		}

		l_numcol=0; //step x
		l_numrow=0; //step y

		std::string l_sep(",");
		std::string l_line;
		while (std::getline(iFile, l_line))
		{
			std::vector<std::string> l_token;
			tokenize_(l_line, l_sep, l_token);
			if(l_token.empty())
				continue;

			l_numrow++;
			std::vector<bool> l_row;
			for(size_t i = 0; i < l_token.size(); ++i)
			{
				l_row.push_back( atoi(l_token[i].c_str()) ? false : true);
			}
			l_numcol = l_token.size();
			l_grid.push_back(l_row);
		}
	}

	double l_xdist = l_bottomLeft.distance(l_bottomRight);
	double l_ydist = l_bottomLeft.distance(l_topLeft);

	m_xStep = l_xdist / double(l_numcol);
	m_yStep = l_ydist / double(l_numrow);

#ifdef _PRINT
	cout << "Col " << l_numcol << endl;
	cout << "Row " << l_numrow << endl;
#endif

	double l_xpos = 0.;
	double l_ypos = 0.;
	m_numRow = 0;
	m_numCol = 0;
	bool l_firstrow = true;
	int irow = 0;

	while(l_ypos < l_ydist + IDSMath::TOLERANCE)
	{
		double l_yorigin = l_ypos + l_bottomLeft.coord(1);
		int l_count = 0;
		l_xpos=0.;
		int icol = 0;
		while( (l_firstrow && l_xpos < l_xdist + IDSMath::TOLERANCE) || (!l_firstrow && l_count < m_numCol))
		{
			double l_xorigin = l_xpos + l_bottomLeft.coord(0);
			l_xpos+=m_xStep;
			if(l_firstrow)
			{
				m_numCol++;
			}
			l_count++;
			icol++;
		}
		l_ypos+=m_yStep;
		m_numRow++;
		l_firstrow = false;
		irow++;
	}

	l_xpos = 0.;
	l_ypos = 0.;
	l_firstrow = true;
	irow = 0;

#ifdef _PRINT
	cout << "Col " << m_numCol << endl;
	cout << "Row " << m_numRow << endl;
#endif

	m_listGraph = std::make_shared<lemon::ListGraph>();
	m_listGraph->reserveNode(m_numCol*m_numRow);
	m_listGraph->reserveEdge( (m_numCol-1)*(m_numRow-1)*4  );

	while(l_ypos < l_ydist + IDSMath::TOLERANCE)
	{
		double l_yorigin = l_ypos + l_bottomLeft.coord(1);
		int l_count = 0;
		l_xpos=0.;
		int icol = 0;
		while( (l_firstrow && l_xpos < l_xdist + IDSMath::TOLERANCE) || (!l_firstrow && l_count < m_numCol))
		{
			double l_xorigin = l_xpos + l_bottomLeft.coord(0);

			Point2D l_mincord = makePoint( IDSReal2D(l_xorigin, l_yorigin), l_bottomLeft.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);
			Point2D l_maxcord = makePoint( IDSReal2D(l_xorigin+m_xStep, l_yorigin+m_yStep), l_bottomLeft.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);

			Box2D l_boxSquare = makeBoundingBox(l_mincord, l_maxcord);

			SquarePtr l_square = std::make_shared<Square>(m_listGraph);
			l_square->setBoundingBox(l_boxSquare);

			int l_col = double(icol) / double(m_numCol) * double(l_grid.back().size());
			int l_row = double(irow) / double(m_numRow) * double(l_grid.size());

			if( l_col < l_grid.back().size() && l_row < l_grid.size() && l_grid[l_row][l_col] )
				l_square->setValid(true);
			else
				l_square->setValid(false);

			m_lattice.push_back(l_square);

			l_xpos+=m_xStep;
			l_count++;
			icol++;
		}
		l_ypos+=m_yStep;
		l_firstrow = false;
		irow++;
	}

	addEdges();
	m_graph = std::make_shared< lemon::Bfs<lemon::ListGraph> >(*m_listGraph.get());
}

//////////////////////////////////////////////////////////////////////////
DiscretizedArea::DiscretizedArea(std::shared_ptr<StructuredArea> _area) 
	: m_graph(nullptr)
	, m_listGraph(nullptr)
	, m_numberOfValidSquare(-1)
	, m_sinkCoordinate(-1,-1)
{
	Box2D l_box = _area->getBoundingBox();

	Point2D l_bottomLeft = l_box.corner(0);
	Point2D	l_bottomRight = l_box.corner(1);
	Point2D	l_topLeft = l_box.corner(2);
	Point2D	l_topRight = l_box.corner(3);

	double l_xdist = l_bottomLeft.distance(l_bottomRight);
	double l_ydist = l_bottomLeft.distance(l_topLeft);

	m_xStep = l_xdist / double(DISCRETIZATION_COL);
	m_yStep = l_ydist / double(DISCRETIZATION_ROW);

	m_listGraph = std::make_shared<lemon::ListGraph>();
	m_listGraph->reserveNode(DISCRETIZATION_COL*DISCRETIZATION_ROW);
	m_listGraph->reserveEdge( (DISCRETIZATION_COL-1)*(DISCRETIZATION_ROW-1)*4 );

	double l_xpos = 0.;
	double l_ypos = 0.;
	m_numRow = 0;
	m_numCol = 0;
	bool l_firstrow = true;
	while(l_ypos < l_ydist + IDSMath::TOLERANCE)
	{
		double l_yorigin = l_ypos + l_bottomLeft.coord(1);
		int l_count = 0;
		l_xpos=0.;
		while( (l_firstrow && l_xpos < l_xdist + IDSMath::TOLERANCE) || (!l_firstrow && l_count < m_numCol))
		{
			double l_xorigin = l_xpos + l_bottomLeft.coord(0);

			Point2D l_mincord = makePoint( IDSReal2D(l_xorigin, l_yorigin), l_box.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);
			Point2D l_maxcord = makePoint( IDSReal2D(l_xorigin+m_xStep, l_yorigin+m_yStep), l_box.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);

			Box2D l_boxSquare = makeBoundingBox(l_mincord, l_maxcord);

			SquarePtr l_square = std::make_shared<Square>(m_listGraph);
			l_square->setBoundingBox(l_boxSquare);

			if( _area->isInside(l_boxSquare) )
				l_square->setValid(true);
			else
				l_square->setValid(false);

			m_lattice.push_back(l_square);

			l_xpos+=m_xStep;
			if(l_firstrow)
				m_numCol++;
			l_count++;
		}
		l_ypos+=m_yStep;
		m_numRow++;
		l_firstrow = false;
	}

	addEdges();
	m_graph = std::make_shared< lemon::Bfs<lemon::ListGraph> >(*m_listGraph.get());
}

//////////////////////////////////////////////////////////////////////////
IDS::BaseGeometry::Point2D DiscretizedArea::getOrigin() const
{
	return m_lattice.at(0)->getBoundingBox().minCoord();
}

//////////////////////////////////////////////////////////////////////////
DiscretizedArea::DiscretizedArea(std::shared_ptr<UnStructuredArea> _area) 
	: m_graph(nullptr)
	, m_listGraph(nullptr)
	, m_numberOfValidSquare(-1)
	, m_sinkCoordinate(-1, -1)
{}

//////////////////////////////////////////////////////////////////////////
DiscretizedArea::DiscretizedArea(IDS::BaseGeometry::Shape2D const& _external, std::set< IDS::BaseGeometry::Shape2D > const& _obstacles) 
	: m_graph(nullptr)
	, m_listGraph(nullptr)
	, m_numberOfValidSquare(-1)
	, m_sinkCoordinate(-1,-1)
{
	Box2D l_box = _external.getBoundingBox();

	Point2D l_bottomLeft = l_box.corner(0);
	Point2D	l_bottomRight = l_box.corner(1);
	Point2D	l_topLeft = l_box.corner(2);
	Point2D	l_topRight = l_box.corner(3);

	double l_xdist = l_bottomLeft.distance(l_bottomRight);
	double l_ydist = l_bottomLeft.distance(l_topLeft);

	m_xStep = l_xdist / double(DISCRETIZATION_COL); //aggiunto int
	m_yStep = l_ydist / double(DISCRETIZATION_ROW);

	m_listGraph = std::make_shared<lemon::ListGraph>();
	m_listGraph->reserveNode(DISCRETIZATION_COL*DISCRETIZATION_ROW);
	m_listGraph->reserveEdge( (DISCRETIZATION_COL-1)*(DISCRETIZATION_ROW-1)*4);

	double l_xpos = 0.;
	double l_ypos = 0.;
	m_numRow = 0;
	m_numCol = 0;
	bool l_firstrow = true;
	while(l_ypos < l_ydist + IDSMath::TOLERANCE)
	{
		double l_yorigin = l_ypos + l_bottomLeft.coord(1);
		int l_count = 0;
		l_xpos=0.;
		while( (l_firstrow && l_xpos < l_xdist + IDSMath::TOLERANCE) || (!l_firstrow && l_count < m_numCol))
		{
			double l_xorigin = l_xpos + l_bottomLeft.coord(0);

			Point2D l_mincord = makePoint( IDSReal2D(l_xorigin, l_yorigin), l_box.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);
			Point2D l_maxcord = makePoint( IDSReal2D(l_xorigin+m_xStep, l_yorigin+m_yStep), l_box.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);

			Box2D l_boxSquare = makeBoundingBox(l_mincord, l_maxcord);

			SquarePtr l_square = std::make_shared<Square>(m_listGraph);
			l_square->setBoundingBox(l_boxSquare);

			if( _external.contains(l_boxSquare.center()) )
				l_square->setValid(true);
			else
				l_square->setValid(false);

			m_lattice.push_back(l_square);

			l_xpos+=m_xStep;
			if(l_firstrow)
				m_numCol++;
			l_count++;
		}
		l_ypos+=m_yStep;
		m_numRow++;
		l_firstrow = false;
	}

	addEdges();
	m_graph = std::make_shared< lemon::Bfs<lemon::ListGraph> >(*m_listGraph.get());
}

//aggiunta
double DiscretizedArea::updateHeading(std::shared_ptr<DiscretizedArea> _space, AgentPosition _LastAgentPos, AgentPosition _CurrentAgentPos) {

	double orientation;
	Point2D p_last = _LastAgentPos.getPoint2D();
	Point2D p_current = _CurrentAgentPos.getPoint2D();

	AreaCoordinate last = _space->getCoordinate(p_last);
	AreaCoordinate current = _space->getCoordinate(p_current);

	if (current.row == last.row)
	{
		if (current.col == last.col + 1) orientation = IDSMath::Pi/2; //90
		if (current.col == last.col - 1) orientation = IDSMath::Pi*1.5; //270
	}
	if (current.col == last.col)
	{
		if (current.row == last.row + 1) orientation = 0.0; //0
		if (current.row == last.row - 1) orientation = IDSMath::Pi; //180
	}
	if (current.row == last.row + 1)
	{
		if (current.col == last.col + 1) orientation = IDSMath::Pi/4; //45
		if (current.col == last.col - 1) orientation = 7*(IDSMath::Pi/4); //315
	}
	if (current.row == last.row - 1)
	{
		if (current.col == last.col + 1) orientation = 3*(IDSMath::Pi/4); //135
		if (current.col == last.col - 1) orientation = 5*(IDSMath::Pi/4); //225
	}
	if (current.row == last.row && current.col == last.col) {
		orientation = _LastAgentPos.getCamera().getOrientation(); // the same orientation
	}
	// set heading
	_CurrentAgentPos.getCamera().setOrientation(orientation);
	return orientation;


	/*
	//AgentPosition LastAgentPos = m_memory.m_elems.back().getLastPosition();
	Line2D line = _LastAgentPos.getPoint2D().lineTo(_CurrentAgentPos.getPoint2D()); //m_point.lineTo(m_currentPosition.m_point);
	double orientation = line.azimuth();
	_CurrentAgentPos.getCamera().setOrientation(orientation);

	return orientation;
	// La mia attuale orientazione che va a definire il mio get Coverage	
	//m_currentPosition.m_camera.setOrientation(orientation); // orientation is setted as the value of azimuth of the line that links two last points
	*/
}

//////////////////////////////////////////////////////////////////////////
AreaCoordinate DiscretizedArea::getCoordinate(Point2D const& point) const
{
	Point2D l_bottomLeft = m_lattice[0]->getBoundingBox().minCoord();
	Point2D l_bottomRight = m_lattice[0]->getBoundingBox().corner(1);
	Point2D l_topLeft = m_lattice[0]->getBoundingBox().corner(2);

	/*Get the angle between this line and a target line.
		*
		*	The angle is positive if the source line(this) must rotate
		*	in clockwise direction to reach the target line,
		*negative otherwise.*/

	Line2D l_xLine = l_bottomLeft.lineTo(l_bottomRight); // restituisce le coordinate dei punti
	Line2D l_yLine = l_bottomLeft.lineTo(l_topLeft);

	Point2D l_prjVertical = l_yLine.projectPoint(point); // restituisce punto proiettato sull'asse y 
	Point2D l_prjOrizontal = l_xLine.projectPoint(point);// restituisce punto proiettato sull'asse x

	double l_ydist = l_yLine.parameterAt(l_prjVertical);
	double l_xdist = l_xLine.parameterAt(l_prjOrizontal);

	AreaCoordinate l_coord(int( floor(l_xdist / m_xStep) ), int( floor(l_ydist / m_yStep) ));// rispettive coord in AreaCoordinate (row, col)
	return l_coord;
}


BaseGeometry::Point2D DiscretizedArea::getCoordinatePoint2D(AreaCoordinate const& point) const
{
	double l_xdist = point.col * m_xStep;
	double l_ydist = point.row * m_yStep;

	Point2D point2D = makePoint(IDSReal2D(l_xdist, l_ydist), BaseGeometry::EllMetric);

	return point2D;
}

//////////////////////////////////////////////////////////////////////////
SquarePtr DiscretizedArea::getSquare(Point2D const& V) const
{
	AreaCoordinate l_coord = getCoordinate(V);
	return this->getSquare(l_coord);
}

Point2D DiscretizedArea::getPosition(AreaCoordinate const& _coord) const
{
	SquarePtr l_square = this->getSquare(_coord);
	if(!l_square)
		return Point2D();
	return l_square->getBoundingBox().center();
}

bool g_reverse = false;
//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::setRandomSquareValue()
{
	g_reverse = !g_reverse;
	/// Compute Random Position:
	srand ( (unsigned int) time(NULL) );

	int flag = m_lattice.size() / 100;

	for(size_t i = 0; i < m_lattice.size(); ++i)
	{
#if 1
		int l_valueSecret = rand() % 100;
		m_lattice[i]->setThiefValue(l_valueSecret);
#else
		m_lattice[i]->setThiefValue(g_reverse ? 100- i / flag:  i / flag);
#endif
	}
}

/*
std::pair<AreaCoordinate, double> DiscretizedArea::getStandardApproachableValidSquares(AreaCoordinate const& _current) const
{
	// in this function all adiacent square are selected and pushed in result
	std::pair<AreaCoordinate, double> result;

	if (_current.row != DISCRETIZATION_ROW)
	{
		AreaCoordinate pos(_current.col, _current.row + 1); //A: head 0
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.first = pos;
			result.second = 0.0;
	}
	if (_current.row != DISCRETIZATION_ROW && _current.col != DISCRETIZATION_COL)
	{
		AreaCoordinate pos(_current.col + 1, _current.row + 1); //B: head 45
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.first = pos;
			result.second = (3.14/180)*45.0;
	}
	if (_current.col != DISCRETIZATION_COL) // check for upper limit for col becouse it is gonna be changed
	{
		AreaCoordinate pos(_current.col + 1, _current.row); //C: head 90
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.first = pos;
			result.second = (3.14 / 180)*90.0;
	}
	if (_current.row != 0 && _current.col != DISCRETIZATION_COL)
	{
		AreaCoordinate pos(_current.col + 1, _current.row - 1); //D: head 135
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.first = pos;
			result.second = (3.14 / 180)*135.0;
	}
	if (_current.row != 0)
	{
		AreaCoordinate pos(_current.col, _current.row - 1); //E: head 180
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.first = pos;
			result.second = (3.14 / 180)*180.0;
	}
	if (_current.row != 0 && _current.col != 0)
	{
		AreaCoordinate pos(_current.col - 1, _current.row  1); //F:head 225
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.first = pos;
			result.second = (3.14 / 180)*225.0;
	}
	if (_current.col != 0) // check for lower limit of col becouse col is gonna be incremented
	{
		AreaCoordinate pos(_current.col - 1, _current.row); //G: head 270
		if (this->getSquare(pos) && this->getSquare(pos)->isValid()) // se esiste ed � valida
			result.first = pos;
			result.second = (3.14 / 180)*270.0;
	}
	if (_current.row != DISCRETIZATION_ROW && _current.col != 0)
	{
		AreaCoordinate pos(_current.col - 1, _current.row + 1); //H: head 315
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.first = pos;
			result.second = (3.14 / 180)*315.0;
	}
	return result;
}
*/

std::vector<AreaCoordinate> DiscretizedArea::getStandardApproachableValidSquares(AreaCoordinate const& _current) const
{
	// in this function all adiacent square are selected and pushed in result
	std::vector<AreaCoordinate> result;

	if (_current.row != DISCRETIZATION_ROW)
	{
		AreaCoordinate pos(_current.col, _current.row + 1, 0.0); //A: head 0
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.row != DISCRETIZATION_ROW && _current.col != DISCRETIZATION_COL)
	{
		AreaCoordinate pos(_current.col + 1, _current.row + 1, IDSMath::PiDiv4); //B: head 45
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.col != DISCRETIZATION_COL) // check for upper limit for col becouse it is gonna be changed
	{
		AreaCoordinate pos(_current.col + 1, _current.row, IDSMath::PiDiv2); //C: head 90
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.row != 0 && _current.col != DISCRETIZATION_COL)
	{
		AreaCoordinate pos(_current.col + 1, _current.row - 1, 3*IDSMath::PiDiv4); //D: head 135
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.row != 0)
	{
		AreaCoordinate pos(_current.col, _current.row - 1, IDSMath::Pi); //E: head 180
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.row != 0 && _current.col != 0)
	{
		AreaCoordinate pos(_current.col - 1, _current.row - 1, 5*IDSMath::PiDiv4); //F:head 225
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.col != 0) // check for lower limit of col becouse col is gonna be incremented
	{
		AreaCoordinate pos(_current.col - 1, _current.row, 3 * IDSMath::PiDiv2); //G: head 270
		if (this->getSquare(pos) && this->getSquare(pos)->isValid()) // se esiste ed � valida
			result.push_back(pos);
	}
	if (_current.row != DISCRETIZATION_ROW && _current.col != 0)
	{
		AreaCoordinate pos(_current.col - 1, _current.row + 1, 7 * IDSMath::PiDiv4); //H: head 315
		if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	return result;
}


//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::addSpecialApproachableValidSquares(AreaCoordinate const& _current, std::vector<AreaCoordinate> & _loci) const
{
	if( _current.col != 0 )
	{
		if(_current.row != 0)
		{
			AreaCoordinate pos(_current.col-1, _current.row-1);
			if(this->getSquare(pos) && this->getSquare(pos)->isValid())
				_loci.push_back(pos);
		}

		if(_current.row != DISCRETIZATION_ROW)
		{
			AreaCoordinate pos(_current.col-1, _current.row+1);
			if(this->getSquare(pos) && this->getSquare(pos)->isValid())
				_loci.push_back(pos);
		}
	}

	if(_current.col != DISCRETIZATION_COL)
	{
		if(_current.row != 0)
		{
			AreaCoordinate pos(_current.col+1, _current.row-1);
			if(this->getSquare(pos) && this->getSquare(pos)->isValid())
				_loci.push_back(pos);
		}

		if(_current.row != DISCRETIZATION_ROW)
		{
			AreaCoordinate pos(_current.col+1, _current.row+1);
			if(this->getSquare(pos) && this->getSquare(pos)->isValid())
				_loci.push_back(pos);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
std::set< std::shared_ptr<Square> > DiscretizedArea::getVisibleSquares(AgentPosition const& _pos)
{
	AreaCoordinate l_currentPos = this->getCoordinate(_pos.getPoint2D());

	std::set< std::shared_ptr<Square> > result;
	for( int row = 0; row < m_numRow; ++row )
	{
		for( int col = 0; col < m_numCol; ++col )
		{
			if( _pos.visible(m_lattice[row * m_numCol + col]) )
				result.insert(m_lattice[row * m_numCol + col]);
		}
	}
	return result;
}

//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::resetValue()
{
	for(size_t i = 0; i < m_lattice.size(); ++i)
	{
		m_lattice[i]->resetThiefValue();
		m_lattice[i]->resetEnergyValue();
	}
}

////////////////////////////////////////////////////////////////////////// //Monitoring the thief position
void DiscretizedArea::setThiefPosition(AgentPosition const& _pos)
{
	SquarePtr l_square = this->getSquare( _pos.getPoint2D() );
	if(l_square)
		l_square->setThiefValue(g_maxValue/g_maxValuePossible);
	else
		assert(1 == 0);

	AreaCoordinate l_coord = this->getCoordinate( _pos.getPoint2D() );

	for(int i = -4; i < 5; ++i)
	{
		int row = l_coord.row + i;
		if(row < 0 || row >= m_numRow)
			continue;
		for(int j = -4; j < 5; ++j)
		{
			int col = l_coord.col + j;
			if(col < 0 || col >= m_numCol)
				continue;

			if(i == 0 && j == 0)
				continue;

			double l_value = g_maxValue/ double( abs(i)+abs(j) );
			double l_valueEx = m_lattice[row * m_numCol + col]->getThiefValue();
			m_lattice[row * m_numCol + col]->setThiefValue( (l_value + l_valueEx) / g_maxValuePossible);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
double DiscretizedArea::getThiefMaxValue(AgentPosition const& _pos)
{
	double tot = g_maxValue/g_maxValuePossible;
	AreaCoordinate l_coord = this->getCoordinate( _pos.getPoint2D() );

	for(int i = -4; i < 5; ++i)
	{
		int row = l_coord.row + i;
		if(row < 0 || row >= m_numRow)
			continue;
		for(int j = -4; j < 5; ++j)
		{
			int col = l_coord.col + j;
			if(col < 0 || col >= m_numCol)
				continue;

			if(i == 0 && j == 0)
				continue;

			double l_value = g_maxValue/ double( abs(i)+abs(j) );
			if( m_lattice[row * m_numCol + col]->isValid() ) // CONTROLLARE: assume 1 solo ladro
				tot += l_value/g_maxValuePossible;
		}
	}
	return tot;
}

////////////////////////////////////////////////////////////////////////// //Monitoring the thief position
void DiscretizedArea::setSinkPosition(AgentPosition const& _pos)
{
	SquarePtr l_square = this->getSquare( _pos.getPoint2D() );
	if(l_square)
		l_square->setEnergyValue(g_maxValue/g_maxValuePossible);
	else
		assert(1 == 0);

	m_sinkCoordinate = this->getCoordinate( _pos.getPoint2D() );

	for(int i = -4; i < 5; ++i)
	{
		int row = m_sinkCoordinate.row + i;
		if(row < 0 || row >= m_numRow)
			continue;
		for(int j = -4; j < 5; ++j)
		{
			int col = m_sinkCoordinate.col + j;
			if(col < 0 || col >= m_numCol)
				continue;

			if(i == 0 && j == 0)
				continue;

			double l_value = g_maxValue/ double( abs(i)+abs(j) );
			double l_valueEx = m_lattice[row * m_numCol + col]->getEnergyValue();
			m_lattice[row * m_numCol + col]->setEnergyValue( (l_value + l_valueEx) /*/ g_maxValue*/);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
int DiscretizedArea::numberOfSquaresCoveredByGuards() const //calcola il numero di quadrati occupati
{
	int l_total = 0;
	for(size_t i = 0; i < m_lattice.size(); ++i)
		if( m_lattice[i]->getTheNumberOfAgent() > 0)
			++l_total;

	return l_total;
}

int DiscretizedArea::getDistance(
	SquarePtr source, 
	SquarePtr target)
{
	m_graph->run(source->m_node, target->m_node);
	return m_graph->dist(target->m_node);
}

//////////////////////////////////////////////////////////////////////////
int DiscretizedArea::getDistance(
	AreaCoordinate _source, 
	AreaCoordinate _target)
{
	SquarePtr l_sourcePtr = this->getSquare(_source);
	SquarePtr l_target = this->getSquare(_target);

	if (l_target && l_sourcePtr)
	{
		m_graph->run(l_sourcePtr->m_node, l_target->m_node);
		return m_graph->dist(l_target->m_node);
	}
	else 
		return 0.;
}

//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::computeNumberOfValidSquare()
{
	for(size_t i = 0; i < m_lattice.size(); ++i)
	{
		if(m_lattice[i]->isValid())
			m_numberOfValidSquare++;
	}
}

//////////////////////////////////////////////////////////////////////////
int DiscretizedArea::getNumberOfValidSquare()
{
	if(m_numberOfValidSquare <0)
		computeNumberOfValidSquare();
	return m_numberOfValidSquare;
}

//////////////////////////////////////////////////////////////////////////
void DiscretizedArea::printOnFile(std::ofstream & _stream)
{
	_stream << "col "<< m_numCol << endl; 
	_stream << "row "<< m_numRow << endl; 

	for(size_t i = 0; i < m_lattice.size(); ++i)
	{
		if ( m_lattice[i]->isValid() )
			_stream << 1;
		else
			_stream << 0;

		if (i ==  m_lattice.size()-1)
			_stream << endl;
		else
			_stream << "\t";
	}

	return;
}

//////////////////////////////////////////////////////////////////////////
int DiscretizedArea::getDistanceFromNearestSink(Point2D const& _agentPosition)
{
	return getDistance(this->getCoordinate(_agentPosition), m_sinkCoordinate);
}

//////////////////////////////////////////////////////////////////////////
std::vector<int> DiscretizedArea::distanceFromNearestSink(std::vector<AgentPosition> const & _positions)
{
	std::vector<int> result (_positions.size(), 0);
	for(size_t i = 0; i < _positions.size(); ++i)
		result[i] = this->getDistanceFromNearestSink(_positions[i].getPoint2D());

	return result;
}

//////////////////////////////////////////////////////////////////////////
std::vector<int> DiscretizedArea::distanceFromNearestSink(std::vector< std::pair<AgentPosition, int> > const & _positions)
{
	std::vector<int> result (_positions.size(), 0);
	for(size_t i = 0; i < _positions.size(); ++i)
		result[i] = this->getDistanceFromNearestSink(_positions[i].first.getPoint2D());

	return result;
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
///////////////////////		SQUARE		//////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
Square::Square(std::shared_ptr<lemon::ListGraph> _graph) 
	: m_valid(true)
	, m_counter(0)
	, m_values(1,0.)
	, m_old_values(1,0.)
	, m_node(_graph->addNode())
{}

//////////////////////////////////////////////////////////////////////////
IDS::BaseGeometry::Point2D Square::vertex(int i) const
{
	int index = i;
	if(i == 2)
		index = 3;
	else if(i == 3)
		index = 2;
	return m_box.corner( index );
}

//////////////////////////////////////////////////////////////////////////
IDS::BaseGeometry::Point2D Square::agentVertex(int i) const
{
	int index = i;
	if(i == 2)
		index = 3;
	else if(i == 3)
		index = 2;
	Point2D l_center = m_box.center();
	Line2D l_line = m_box.corner( index ).lineTo(l_center);
	double l_dist = m_box.corner( index ).distance(l_center);
	return l_line.pointFromOrigin(l_dist/2.);
}

//////////////////////////////////////////////////////////////////////////
void Square::setBoundingBox(IDS::BaseGeometry::Box2D const& _box)
{
	m_box = _box;
}

//////////////////////////////////////////////////////////////////////////
bool Square::isChanged() const
{
	return fabs(m_values.at(1) - m_old_values.at(1)) > IDSMath::TOLERANCE;
}

//////////////////////////////////////////////////////////////////////////
void Square::resetThiefValue()
{
	if(m_values.size() > 0)
		m_values[0] = 0.;
}

//////////////////////////////////////////////////////////////////////////
void Square::resetEnergyValue()
{
	if(m_values.size() > 1)
		m_values[1] = 0.;
}

//////////////////////////////////////////////////////////////////////////
void Square::setValid(bool valid) 
{
	m_valid = valid;
}

//////////////////////////////////////////////////////////////////////////
void Square::setThiefValue(double _value) 
{
	if(m_values.size() > 0)
	{
		m_old_values[0] = m_values[0];
		m_values[0] = _value;
	}
	else
	{
		m_old_values.push_back(0.);
		m_values.push_back(_value);
	}
}

//////////////////////////////////////////////////////////////////////////
void Square::setEnergyValue(double _value) 
{
	if(m_values.size() > 1)
	{
		m_old_values[1] = m_values[1];
		m_values[1] = _value;
	}
	else if(m_values.size() > 0)
	{
		m_old_values.push_back(0.);
		m_values.push_back(_value);
	}
	else
	{
		m_old_values.push_back(0.);
		m_old_values.push_back(0.);
		m_values.push_back(0.);
		m_values.push_back(_value);
	}
}