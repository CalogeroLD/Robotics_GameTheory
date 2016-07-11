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
#include<rapidjson\document.h>
#include<rapidjson\filereadstream.h>

#define _PRINT
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


DiscretizedArea::DiscretizedArea(rapidjson::Value& Area)
		: m_graph(nullptr)
		, m_listGraph(nullptr)
		, m_numberOfValidSquare(-1)
		, m_sinkCoordinate(-1, -1)
{

	std::vector<std::vector<bool>> l_grid;
	int l_numcol(1), l_numrow(1);

	// reading length and height from file.json
	rapidjson::Value& length = Area["length"];
	rapidjson::Value& height = Area["height"];
	double length_value, height_value;

	// takes it as double
	if (!length.IsDouble())
		cout << "Err: length is not a double" << endl;
	else
		length_value = length.GetDouble();

	if (!height.IsDouble())
		cout << "Err height is not a double" << endl;
	else
		height_value = height.GetDouble();

	// dichiarate nell aclasse DiscretizedArea da C.Li Destri
	l_bottomLeft = makePoint(IDSReal2D(0, 0), EucMetric);
	l_topLeft = makePoint(IDSReal2D(0, height_value), EucMetric);
	l_topRight = makePoint(IDSReal2D(length_value, height_value), EucMetric);
	l_bottomRight = makePoint(IDSReal2D(length_value, 0), EucMetric);

	// reading the number of row and col we set in the file.json
	rapidjson::Value& rows = Area["rows"];
	rapidjson::Value& cols = Area["cols"];

	rapidjson::Value& ship = Area["ship"];
	rapidjson::Value& ship_coord = ship["coord"]; // array
	rapidjson::Value& ship_dimcol = ship["dim_col"]; // int
	rapidjson::Value& ship_dimrow = ship["dim_row"]; // int
	
	int shipDimCol, shipDimRow;

	if (!ship_dimcol.IsInt())
		cout << "ERR: Ship_dimcol is not int" << endl;
	else
		shipDimCol = ship_dimcol.GetInt();

	if (!ship_dimrow.IsInt())
		cout << "ERR: Ship_dimcol is not int" << endl;
	else
		shipDimRow = ship_dimrow.GetInt();

	int coord_shipCol, coord_shipRow;
	
	//cout << ship_coord.GetArray()[0].GetInt() << endl;

	//cout << length.GetDouble() << endl;;
	coord_shipCol = ship_coord.GetArray()[0].GetInt();
	coord_shipRow = ship_coord.GetArray()[1].GetInt();

	std::cout << "ship_coord : " << coord_shipCol << "," << coord_shipRow << std::endl;

	// catch the points covered by ship to protect
	std::vector<AreaCoordinate> pointCoveredByShip;
	AreaCoordinate point;

	for (int i = coord_shipCol - shipDimCol; i <=  coord_shipCol + shipDimCol; i++)
	{
		for (int j = coord_shipRow - shipDimRow; j <= coord_shipRow + shipDimRow; j++)
		{
			AreaCoordinate point(j, i);
			pointCoveredByShip.push_back(point);
		}

	}

	// Gets the dimentions of Area

	l_numcol = cols.GetInt(); //step x
	l_numrow = rows.GetInt(); //step y
	//std::cout << l_numcol << endl;
	//std::cout << l_numrow << endl;

	std::vector<bool> l_row;
	// builds the grid with true if is open sea without obstacles
	for (int i = 0; i < l_numrow; i++)
	{
		for (int j = 0; j < l_numcol; j++)
		{
				l_row.push_back(true);
		}
		l_grid.push_back(l_row);
		l_row.clear();
	}

	// fills the subregions covered by ship with false 
	for (int i = 0; i < l_numcol; i++)
	{
		for (int j = 0; j < l_numcol; j++)
		{
			for (int k = 0; k < pointCoveredByShip.size(); k++)
			{
				if (i == pointCoveredByShip.at(k).col && j == pointCoveredByShip.at(k).row)
					l_grid.at(i).at(j) = false;
			}
		}
	}
	
	// only for visualization of grid points
	/*for (int i = 0; i < l_numrow; i++)
	{
		int c;
		std::vector<bool> Riga = l_grid.at(i);

		std::cout << "riga: " << i << " " ;
		for (int j = 0; j < Riga.size(); j++)
		{
			if (Riga.at(j) == true)	c = 0;
			if (Riga.at(j) == false)	c = 1;
			std::cout << " " << c << " ";
		}
		std::cout << " " << endl;

	}
	std::cout << "dim grid: " << l_grid.size() << "  per   " << l_grid.at(0).size() << endl;*/

	double l_xdist = l_bottomLeft.distance(l_bottomRight);
	double l_ydist = l_bottomLeft.distance(l_topLeft);

	m_xStep = l_xdist / double(l_numcol);
	m_yStep = l_ydist / double(l_numrow);
	//std::cout << m_xStep << endl;
	//std::cout << m_yStep << endl;

#ifdef _PRINT
	//cout << "Col " << l_numcol << endl;
	//cout << "Row " << l_numrow << endl;
#endif

	double l_xpos = 0.;
	double l_ypos = 0.;
	m_numRow = 0;
	m_numCol = 0;
	bool l_firstrow = true;
	int irow = 0;

	while (l_ypos < l_ydist + IDSMath::TOLERANCE)
	{
		double l_yorigin = l_ypos + l_bottomLeft.coord(1);
		int l_count = 0;
		l_xpos = 0.;
		int icol = 0;
		while ((l_firstrow && l_xpos < l_xdist + IDSMath::TOLERANCE) || (!l_firstrow && l_count < m_numCol))
		{
			double l_xorigin = l_xpos + l_bottomLeft.coord(0);
			l_xpos += m_xStep;
			if (l_firstrow)
			{
				m_numCol++;
			}
			l_count++;
			icol++;
		}
		l_ypos += m_yStep;
		m_numRow++;
		l_firstrow = false;
		irow++;
	}

	l_xpos = 0.;
	l_ypos = 0.;
	l_firstrow = true;
	irow = 0;

	// messo da me
	//m_numCol = l_numcol;
	//m_numRow = l_numrow;

	m_listGraph = std::make_shared<lemon::ListGraph>();
	m_listGraph->reserveNode(m_numCol*m_numRow);
	m_listGraph->reserveEdge((m_numCol - 1)*(m_numRow - 1) * 4);

	while (l_ypos < l_ydist + IDSMath::TOLERANCE)
	{
		double l_yorigin = l_ypos + l_bottomLeft.coord(1);
		int l_count = 0;
		l_xpos = 0.;
		int icol = 0;
		while ((l_firstrow && l_xpos < l_xdist + IDSMath::TOLERANCE) || (!l_firstrow && l_count < m_numCol))
		{
			double l_xorigin = l_xpos + l_bottomLeft.coord(0);

			Point2D l_mincord = makePoint(IDSReal2D(l_xorigin, l_yorigin), l_bottomLeft.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);
			Point2D l_maxcord = makePoint(IDSReal2D(l_xorigin + m_xStep, l_yorigin + m_yStep), l_bottomLeft.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);

			Box2D l_boxSquare = makeBoundingBox(l_mincord, l_maxcord);

			SquarePtr l_square = std::make_shared<Square>(m_listGraph);
			l_square->setBoundingBox(l_boxSquare);

			int l_col = double(icol) / double(m_numCol) * double(l_grid.back().size());
			int l_row = double(irow) / double(m_numRow) * double(l_grid.size());
			//std::cout << l_col << std::endl;

			if (l_col < l_grid.back().size() && l_row < l_grid.size() && l_grid[l_row][l_col])
				l_square->setValid(true);
			else
				l_square->setValid(false);

			m_lattice.push_back(l_square);

			l_xpos += m_xStep;
			l_count++;
			icol++;
		}
		l_ypos += m_yStep;
		l_firstrow = false;
		irow++;
	}

	addEdges();
	m_graph = std::make_shared< lemon::Bfs<lemon::ListGraph> >(*m_listGraph.get());
}

// c'era
////////////////////////////////////////////////////////////////////////// da FILE
/*DiscretizedArea::DiscretizedArea(std::string const& _filename) 
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
}*/

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


////////////////////////////////L. Destri//////////////////////////////////////////
AreaCoordinate DiscretizedArea::getCoordinate(Point2D const& point)const
{

	/*std::cout << "controllo sui punti bottomleft: x " << l_bottomLeft.coord(0) << " y " << l_bottomLeft.coord(1) << std::endl;
	std::cout << "controllo sui punti topleft: x " << l_topLeft.coord(0) << " y " << l_topLeft.coord(1) << std::endl;
	std::cout << "controllo sui punti topRight: x " << l_topRight.coord(0) << " y " << l_topRight.coord(1) << std::endl;
	std::cout << "controllo sui punti bottomRight: x " << l_bottomRight.coord(0) << " y " << l_bottomRight.coord(1) << std::endl;*/

	Line2D l_xLine = l_bottomLeft.lineTo(l_bottomRight); // restituisce le coordinate dei punti
	Line2D l_yLine = l_bottomLeft.lineTo(l_topLeft);

	Point2D l_prjVertical = l_yLine.projectPoint(point); // restituisce punto proiettato sull'asse y 
	Point2D l_prjOrizontal = l_xLine.projectPoint(point);// restituisce punto proiettato sull'asse x

	//cout << " getCoordinate del punto x =  " << point.coord(0) << "y = " << point.coord(1) << endl;
	
	/*cout <<" punto proiettati orizz x : " << l_prjOrizontal.coord(0) << "y : " << l_prjOrizontal.coord(1) << endl;
	cout << "punto proiettati verti x : " << l_prjVertical.coord(0) << "y : " << l_prjVertical.coord(1) << endl;*/

	/*double l_ydist = l_yLine.parameterAt(l_prjVertical);
	double l_xdist = l_xLine.parameterAt(l_prjOrizontal);*/

	double l_ydist = point.coord(1); // coord y del punto
	double l_xdist = point.coord(0); // coord x del punto

	/*std::cout << l_ydist << l_xdist <<endl;
	std::cout << l_xdist << l_ydist << endl;*/

	//std::cout << " AreaCoordinate row = " << int(floor(l_xdist / m_xStep)) << " col = " << int(floor(l_ydist / m_yStep)) << endl;

	AreaCoordinate l_coord(int(floor(l_xdist / m_xStep)), int(floor(l_ydist / m_yStep)));// rispettive coord in AreaCoordinate (row, col)

	return l_coord;
}

//////////////////////////////////////////////////////////////////////////
/*AreaCoordinate DiscretizedArea::getCoordinate(Point2D const& point) const
{
	Point2D l_bottomLeft = m_lattice[0]->getBoundingBox().minCoord();
	Point2D l_bottomRight = m_lattice[0]->getBoundingBox().corner(1);
	Point2D l_topLeft = m_lattice[0]->getBoundingBox().corner(2);

	Line2D l_xLine = l_bottomLeft.lineTo(l_bottomRight); // restituisce le coordinate dei punti
	Line2D l_yLine = l_bottomLeft.lineTo(l_topLeft);

	Point2D l_prjVertical = l_yLine.projectPoint(point); // restituisce punto proiettato sull'asse y 
	Point2D l_prjOrizontal = l_xLine.projectPoint(point);// restituisce punto proiettato sull'asse x

	double l_ydist = l_yLine.parameterAt(l_prjVertical);
	double l_xdist = l_xLine.parameterAt(l_prjOrizontal);

	AreaCoordinate l_coord(int( floor(l_xdist / m_xStep) ), int( floor(l_ydist / m_yStep) ));// rispettive coord in AreaCoordinate (row, col)
	
	return l_coord;
}*/


BaseGeometry::Point2D DiscretizedArea::getCoordinatePoint2D(AreaCoordinate const& point) const
{
	double l_xdist = point.col * m_xStep;
	double l_ydist = point.row * m_yStep;
	Point2D point2D;
	try
	{
		point2D = makePoint(IDSReal2D(l_xdist, l_ydist), point2D.isEllipsoidic() ? BaseGeometry::EllMetric : BaseGeometry::EucMetric);
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		system("pause");
	}

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


double Mod2Pi(double angle) {
	if (angle < 0)	return angle += IDSMath::TwoPi;
	if (angle > IDSMath::TwoPi)	return angle -= IDSMath::TwoPi;
	if (angle >= 0 && angle <= IDSMath::TwoPi)	return angle;
	else std::cout << "values out of range" <<std::endl;
}

std::vector<AreaCoordinate> DiscretizedArea::getStandardApproachableValidSquares(AreaCoordinate const& _current) const
{
	std::vector<AreaCoordinate> result; // AreaCoordinate possible
	if( _current.row != DISCRETIZATION_ROW && _current.col != DISCRETIZATION_COL && _current.row != 0 && _current.col != 0)
		result = this->goStraight(_current);
	if (_current.row == DISCRETIZATION_ROW && _current.heading == 0.0)
		result = this->turnDown(_current);
	if (_current.row == 0 /*&& _current.heading == 3.14*/)
		result = this->turnUp(_current);
	if (_current.col == 0)
		result = this->turnRight(_current);
	if (_current.col == DISCRETIZATION_COL)
		result = this->turnLeft(_current);
	// in every case robot can rotate
	this->rotate(_current, result);


	//std::cout << result.size() << std::endl;
	return result;
}

std::vector<AreaCoordinate> DiscretizedArea::goStraight(AreaCoordinate const& _current) const
{
	std::vector<AreaCoordinate> result;
	if (_current.heading == 0.0){
	AreaCoordinate pos(_current.col, _current.row + 1, _current.heading);
	result.push_back(pos);
	}
	if (_current.heading == 1.57){
	AreaCoordinate pos(_current.col + 1, _current.row, _current.heading);
	result.push_back(pos);
	}
	if (_current.heading == 3.14) {
		AreaCoordinate pos(_current.col, _current.row - 1, _current.heading);
		result.push_back(pos);
	}
	if (_current.heading == 4.71) {
		AreaCoordinate pos(_current.col - 1, _current.row, _current.heading);
		result.push_back(pos);
	}
	// diagonale
	
	if (_current.heading == 0.785) {
		AreaCoordinate pos(_current.col + 1, _current.row + 1, _current.heading);
		cout << "NE" << endl;
		result.push_back(pos);
	}
	if (_current.heading == 2.35) {
		AreaCoordinate pos(_current.col + 1, _current.row - 1, _current.heading);
		cout << "SE" << endl;
		result.push_back(pos);
	}
	if (_current.heading == 3.92) {
		AreaCoordinate pos(_current.col - 1, _current.row - 1, _current.heading);
		cout << "SO" << endl;
		result.push_back(pos);
	}
	if (_current.heading == 5.49) {
		AreaCoordinate pos(_current.col - 1, _current.row + 1, _current.heading);
		result.push_back(pos);
		cout << "NO" << endl;
	}
	this->changeDirection(_current, result);
	return result;
}

void DiscretizedArea::rotate(AreaCoordinate const& _current, std::vector<AreaCoordinate>& result) const
{
	AreaCoordinate pos(_current.col, _current.row, Mod2Pi(_current.heading - 0.785));
	result.push_back(pos);
	AreaCoordinate pos1(_current.col, _current.row, Mod2Pi(_current.heading + 0.785));
	result.push_back(pos1);
}

void DiscretizedArea::changeDirection(AreaCoordinate const& _current, std::vector<AreaCoordinate>& result) const
{
	if (_current.heading == 0.0)
	{
		AreaCoordinate pos(_current.col - 1, _current.row + 1, Mod2Pi(_current.heading - 0.785));
			result.push_back(pos);

		AreaCoordinate pos1(_current.col + 1, _current.row + 1, Mod2Pi(_current.heading + 0.785));
			result.push_back(pos1);
	}
	if (_current.heading == 3.14)
	{
		AreaCoordinate pos(_current.col - 1, _current.row -1, Mod2Pi(_current.heading + 0.785));
			result.push_back(pos);

		AreaCoordinate pos1(_current.col + 1, _current.row - 1, Mod2Pi(_current.heading - 0.785));
			result.push_back(pos1);
	}
}

std::vector<AreaCoordinate> DiscretizedArea::turnDown(AreaCoordinate const& _current) const
{
	std::vector<AreaCoordinate> result;
	std::cout << "turndown" << endl;
	AreaCoordinate pos(_current.col, _current.row - 1, 3.14);
	if (this->getSquare(pos) && this->getSquare(pos)->isValid())
		result.push_back(pos);

	return result;
}

std::vector<AreaCoordinate> DiscretizedArea::turnUp(AreaCoordinate const& _current) const
{
	std::vector<AreaCoordinate> result;
	std::cout << "turnUp" << endl;
	AreaCoordinate pos(_current.col, _current.row + 1, 0.0);
	if (this->getSquare(pos) && this->getSquare(pos)->isValid())
		result.push_back(pos);

	return result;
}

std::vector<AreaCoordinate> DiscretizedArea::turnRight(AreaCoordinate const& _current) const
{
	std::vector<AreaCoordinate> result;
	std::cout << "turnRight" << endl;
	AreaCoordinate pos(_current.col + 1, _current.row, 1.57);
	if (this->getSquare(pos) && this->getSquare(pos)->isValid())
		result.push_back(pos);

	return result;
}

std::vector<AreaCoordinate> DiscretizedArea::turnLeft(AreaCoordinate const& _current) const
{
	std::vector<AreaCoordinate> result;
	std::cout << "turnLeft" << endl;
	AreaCoordinate pos(_current.col - 1, _current.row, 4.71);
	if (this->getSquare(pos) && this->getSquare(pos)->isValid())
		result.push_back(pos);

	return result;
}


///////////////////////////////////////////THIEF MOTION///////////////////
std::vector<AreaCoordinate> DiscretizedArea::getStandardApproachableValidSquaresThief(AreaCoordinate const& _current) const
{
	std::vector<AreaCoordinate> result;
	if (_current.col != 0)
	{
		AreaCoordinate pos(_current.col - 1, _current.row);
		//if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.col != DISCRETIZATION_COL)
	{
		AreaCoordinate pos(_current.col + 1, _current.row);
		//if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.row != 0)
	{
		AreaCoordinate pos(_current.col, _current.row - 1);
		///if (this->getSquare(pos) && this->getSquare(pos)->isValid())
			result.push_back(pos);
	}
	if (_current.row != DISCRETIZATION_ROW)
	{
		AreaCoordinate pos(_current.col, _current.row + 1);
		//if (this->getSquare(pos) && this->getSquare(pos)->isValid())
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
	SquarePtr l_square = this->getSquare( _pos.getPoint2D() ); // valore associato a quella regione
	
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

			if(i == 0 && j == 0) // per evitarli al den
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