// corridor.cpp : Defines the entry point for the application.
//
#include "stdafx.h"
#include "Resource.h"
#include "CoverageTest.h"

#include "BaseGeometry/PointEuclidean2D.h"
#include "BaseGeometry/PointEllipsoidic2D.h"
#include "BaseGeometry/Point2D.h"

#include "Coverage/Agent.h"
#include "Coverage/Thief.h"
#include "Coverage/Guard.h"
#include "Coverage/Sink.h"

#include "Coverage/StructuredArea.h"

#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <map>

#include<rapidjson\document.h>
#include<rapidjson\filereadstream.h>



using namespace std;

double const PI = 3.1415926536;
int const kk = 10000;

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	Curve(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	SetNumberOfGuards(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	SetNumberOfSteps(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	SetMonitorStep(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	SetAgentsPeriod(HWND, UINT, WPARAM, LPARAM);

//////////////////////////////////////////////////////////////////////////
CoverageTest::CoverageTest(int nPoints, const Point2D *bound, bool counterclockwise)
{

}

//////////////////////////////////////////////////////////////////////////
CoverageTest::CoverageTest(const vector<Point2D>& bound, bool counterclockwise)
{
	std::shared_ptr<Area> l_space = std::make_shared<StructuredArea>(bound);

	std::string conf_file = "Scenario_5G_1T_multiAgent.json";
	FILE * cf = fopen(conf_file.c_str(), "r");
	char readBuffer[65536];
	rapidjson::FileReadStream is(cf, readBuffer, sizeof(readBuffer));
	rapidjson::Document document;
	document.ParseStream(is);	
	
	if (!document.IsObject()) {
		std::cout << "ERR: error during the parsing of configuration file" << std::endl;
		exit(1);
	}

	// Agents
	rapidjson::Value& Agents = document["Agents"];

	if (!(Agents.HasMember("coord")))
		cout << "ERR: Nautral Agents has not coord" << endl;
	// check
	if (!(Agents.HasMember("sensors")))
		cout << "ERR: sensors field of Agents hasn't been decleared" << endl;

	rapidjson::Value& Agents_coord = Agents["coord"];
	rapidjson::Value& Agents_sensors = Agents["sensors"];

	std::cout << "agenti " << Agents_coord.Size() << std::endl;
 
	std::set< std::shared_ptr<Agent> >l_agents; 
	int l_id = -1;
	
	/*for (int i = 0; i < Agents_coord.Size(); i++)
	{
		++l_id;
		double x = Agents_coord[i].GetArray()[0].GetDouble();
		double y = Agents_coord[i].GetArray()[1].GetDouble();
		double heading = Agents_coord[i].GetArray()[2].GetDouble();
		double farRadius = Agents_sensors[i].GetArray()[0].GetDouble();
		double nearRadius = Agents_sensors[i].GetArray()[1].GetDouble();
		double orientation = Agents_sensors[i].GetArray()[2].GetDouble();
		double fov = Agents_sensors[i].GetArray()[3].GetDouble();
		AgentPosition l_pos(makePoint(IDSReal2D(x, y), EucMetric), heading, CameraPosition(farRadius, nearRadius, orientation, fov));

		std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, l_id, l_pos, g_agentsPeriod, g_pareto ? 1 : 2);
		l_agents.insert(l_agents);
		Sleep(1000);
	}*/


	for(int i = 0; i < g_numberOfAgents; ++i)
	{
	
		double x = Agents_coord[i].GetArray()[0].GetDouble();
		double y = Agents_coord[i].GetArray()[1].GetDouble();

		double farRadius = Agents_sensors[i].GetArray()[0].GetDouble();
		double nearRadius = Agents_sensors[i].GetArray()[1].GetDouble();
		double orientation = Agents_sensors[i].GetArray()[2].GetDouble();
		double fov = Agents_sensors[i].GetArray()[3].GetDouble();

		++l_id;
		AgentPosition l_pos(l_space->randomPosition(), 3.14/2, CameraPosition(farRadius, nearRadius, orientation, fov ) );
		std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, l_id, l_pos, g_agentsPeriod, g_pareto?1:2);
		l_agents.insert(l_agent);
		Sleep(1000);
	}

	m_algorithm = std::make_shared<CoverageAlgorithm>(l_agents, l_space, g_correlated? 3 : g_pareto? 2: g_DISL? 0 : 1);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::getGuardsPosition(std::vector<AgentPosition> & _pos)
{
	return m_algorithm->getGuardsPosition(_pos);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::getGuardsSquare(std::vector< std::pair<SquarePtr, AgentActionIndex> > & _pos)
{
	return m_algorithm->getGuardsSquare(_pos);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::getGuardsCoverage(std::vector<LineString2D>& _areas)
{
	return m_algorithm->getGuardsCoverage(_areas);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::getSinksCoverage(std::vector<LineString2D>& _areas)
{
	return m_algorithm->getSinksCoverage(_areas); // CONTROLLARE
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::getSinksSquare(std::vector< std::pair<SquarePtr,int> > & _pos)
{
	return m_algorithm->getSinksSquare(_pos); // CONTROLLARE
}

//////////////////////////////////////////////////////////////////////////
int CoverageTest::numberOfSquaresCoveredByGuards()
{
	return m_algorithm->numberOfSquaresCoveredByGuards();
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::printPotential(std::string const& name, bool _printOnFile)
{
	return m_algorithm->printPotential(name, _printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::printBenefit(std::string const& name, bool _printOnFile)
{
	return m_algorithm->printBenefit(name, _printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::printPotentialIndex(std::string const& name, bool _print)
{
	return m_algorithm->printPotentialIndex(name, _print);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::printNewPerformanceIndex(std::string const& name, bool _print)
{
	return m_algorithm->printNewPerformanceIndex(name, _print);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::printNewPerformanceIndexVersusExplorationRate(std::string const& name, bool printOnFile)
{
	return m_algorithm->printNewPerformanceIndexVersusExplorationRate(name, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::printExplorationRate(std::string const& name, bool printOnFile)
{
	return m_algorithm->printExplorationRate(name, printOnFile);
}

//////////////////////////////////////////////////////////////////////////
std::string CoverageTest::getExplorationRateStr()
{
	return m_algorithm->getExplorationRateStr();
}

//////////////////////////////////////////////////////////////////////////
std::string CoverageTest::getBatteryValueStr()
{
	return m_algorithm->getBatteryValueStr();
}

//////////////////////////////////////////////////////////////////////////
double CoverageTest::getExplorationRate()
{
	return m_algorithm->getExplorationRate();
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
std::wstring s2ws(const std::string& s)
{
	int len;
	int slength = (int)s.length() + 1;
	len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0); 
	wchar_t* buf = new wchar_t[len];
	MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
	std::wstring r(buf);
	delete[] buf;
	return r;
}

//////////////////////////////////////////////////////////////////////////
void DrawString(HDC hdc, int width, int height, double pt_x, double pt_y,
	int R, int G, int B, std::string const& text)
{
	std::wstring stemp = s2ws(text);
	LPCWSTR result = stemp.c_str();

	int x = (int) (pt_x/kk*(double)width);
	int y = (int) (pt_y/kk*(double)height);

	TextOut(hdc, x,y, result, wcslen(result));

	g_explorationFile << text << std::endl;
	g_explorationFile.flush();

}

//////////////////////////////////////////////////////////////////////////
void MarkPoint(LONG x, LONG y, HDC hdc, int R, int G, int B, int thickness)
{
	HPEN markerPen=CreatePen(PS_SOLID, thickness, RGB(R,G,B));
	HBRUSH myBrush=CreateSolidBrush(RGB(R,G,B));
	HGDIOBJ oldPen;
	HGDIOBJ oldBrush=SelectObject(hdc,myBrush);

	oldPen=SelectObject(hdc,markerPen);

	for( int i = -thickness; i!= thickness; i++ )
	{
		MoveToEx(hdc, (int) x - thickness, (int) y - i, (LPPOINT) NULL); 
		LineTo(hdc, (int) x + thickness, (int) y - i); 
		MoveToEx(hdc, (int) x - i, (int) y - thickness, (LPPOINT) NULL); 
		LineTo(hdc, (int) x - i, (int) y + thickness);
	}
	SelectObject(hdc,oldPen);
	SelectObject(hdc,oldBrush);
	DeleteObject(markerPen);
	DeleteObject(myBrush);
}

//////////////////////////////////////////////////////////////////////////
void DrawPath(LineString2D *path, HDC hdc, int width, int height,
	int R, int G, int B, int thickness)
{
	HPEN myPen=CreatePen(PS_SOLID, thickness, RGB(R,G,B));
	HGDIOBJ oldPen=SelectObject(hdc,myPen);

	if(path->size()>0){
		LineString2DIterator vert;
		vert=path->begin();			
		MoveToEx(hdc, (int) ((*vert).coord().v[0]/kk*(double)width), (int) ((*vert).coord().v[1]/kk*(double)height), (LPPOINT) NULL); 

		for(vert=path->begin();vert!=path->end();){
			vert++;
			if(vert==path->end())
				break;
			LineTo(hdc, (int) ((*vert).coord().v[0]/kk*(double)width), (int) ((*vert).coord().v[1]/kk*(double)height)); 
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void DrawTriangle(SquarePtr rect, HDC hdc, int width, int height,
	int R, int G, int B, int thickness)
{
	HPEN myPen=CreatePen(PS_SOLID, thickness, RGB(0,0,0));
	HBRUSH myBrush=CreateSolidBrush(RGB(R,G,B));
	HGDIOBJ oldPen=SelectObject(hdc,myPen);
	HGDIOBJ oldBrush=SelectObject(hdc,myBrush);

	POINT l_points[4];
	for(int i=0; i!=4; i++)
	{
		l_points[i].x=rect->agentVertex(i).coord().v[0]/kk*(double)width;
		l_points[i].y=rect->agentVertex(i).coord().v[1]/kk*(double)height;
	}

	POINT points[3];
	points[0].x=l_points[0].x;
	points[0].y=l_points[0].y;
	points[1].x=l_points[1].x;
	points[1].y=l_points[1].y;
	points[2].x=(l_points[2].x + l_points[3].x) / 2.;
	points[2].y=(l_points[2].y + l_points[3].y) / 2.;

	Polygon(hdc,points,3);

	SelectObject(hdc,oldPen);
	SelectObject(hdc,oldBrush);
	DeleteObject(myPen);
	DeleteObject(myBrush);
}

//////////////////////////////////////////////////////////////////////////
void DrawSquare(
	SquarePtr rect, HDC hdc, int width, int height,
	int R, int G, int B, int thickness)
{
	HPEN myPen=CreatePen(PS_SOLID, thickness, RGB(0,0,0));
	HBRUSH myBrush=CreateSolidBrush(RGB(R,G,B));
	HGDIOBJ oldPen=SelectObject(hdc,myPen);
	HGDIOBJ oldBrush=SelectObject(hdc,myBrush);

	POINT points[4];
	for(int i=0; i!=4; i++)
	{
		points[i].x=rect->vertex(i).coord().v[0]/kk*(double)width;
		points[i].y=rect->vertex(i).coord().v[1]/kk*(double)height;
	}

	Polygon(hdc,points,4);

	SelectObject(hdc,oldPen);
	SelectObject(hdc,oldBrush);
	DeleteObject(myPen);
	DeleteObject(myBrush);
}

//////////////////////////////////////////////////////////////////////////
void DrawGuard(
	SquarePtr rect, HDC hdc, int width, int height,
	int R, int G, int B, int thickness)
{
	HPEN myPen=CreatePen(PS_SOLID, thickness, RGB(0,0,0));
	HBRUSH myBrush=CreateSolidBrush(RGB(R,G,B));
	HGDIOBJ oldPen=SelectObject(hdc,myPen);
	HGDIOBJ oldBrush=SelectObject(hdc,myBrush);

	POINT points[4];
	for(int i=0; i!=4; i++)
	{
		points[i].x=rect->agentVertex(i).coord().v[0]/kk*(double)width;
		points[i].y=rect->agentVertex(i).coord().v[1]/kk*(double)height;
	}

	Polygon(hdc,points,4);

	SelectObject(hdc,oldPen);
	SelectObject(hdc,oldBrush);
	DeleteObject(myPen);
	DeleteObject(myBrush);
}

//////////////////////////////////////////////////////////////////////////
void DrawSink(
	HDC hdc, int width, int height,
	int R, int G, int B, int thickness )
{
	std::vector<LineString2D> l_coverageArea;
	if(g_coverageTest)
		g_coverageTest->getSinksCoverage(l_coverageArea);
	for(size_t i = 0; i < l_coverageArea.size(); ++i)
		DrawPath(&l_coverageArea[i], hdc, width, height, 255,0,255, 1);

	std::vector< std::pair<SquarePtr,int> > l_pos;
	if(g_coverageTest)
		g_coverageTest->getSinksSquare(l_pos);
	for(size_t i = 0; i < l_pos.size(); ++i)
	{
		DrawTriangle(l_pos[i].first, hdc, width, height, 255,0,255, thickness);
	}
}

//////////////////////////////////////////////////////////////////////////
template <typename T>
string NumberToString ( T Number )
{
	ostringstream ss;
	ss << Number;
	return ss.str();
}

//////////////////////////////////////////////////////////////////////////
void DrawGuards(
	HDC hdc, int width, int height,
	int R, int G, int B, int thickness )
{
	std::vector<LineString2D> l_coverageArea;
	if(g_coverageTest)
		g_coverageTest->getGuardsCoverage(l_coverageArea);
	for(size_t i = 0; i < l_coverageArea.size(); ++i)
		DrawPath(&l_coverageArea[i], hdc, width, height, 255,0,0, 1);

	std::vector< std::pair<SquarePtr, AgentActionIndex> > l_pos;
	if(g_coverageTest)
		g_coverageTest->getGuardsSquare(l_pos);
	for(size_t i = 0; i < l_pos.size(); ++i)
	{
		DrawGuard(l_pos[i].first, 
			hdc, 
			width, height, 
			255,255,255, 
			thickness);
		string name = NumberToString(i+1);
		string name_complete = name;
		if (l_pos[i].second.m_total > 1 || 1)
		{
			string actionElem = NumberToString(l_pos[i].second.m_elem);
			string actionTotal = NumberToString(l_pos[i].second.m_total);
			name_complete +=  ": " + actionElem + "/" + actionTotal;
		}
		DrawString(hdc, width, height, 
			l_pos[i].first->agentVertex(3).coord().v[0],
			l_pos[i].first->agentVertex(3).coord().v[1],
			0, 0, 0, name_complete);
	}
}
class ColorMap;

/////////////////////////////////////////////////////////////////////////
class MyColour
{
public:
	int R;
	int G;
	int B;

	void setRandom()
	{
		R = rand()%255;
		G = rand()%255;
		B = rand()%255;
	}

	void setMonitor()
	{
		MyColour local;
		R = rand()%255;
		G = rand()%255;
		B = rand()%255;
	}
};

//////////////////////////////////////////////////////////////////////////
class ColorMap
{
public:
	std::map<double, MyColour> m_legend;

	inline ColorMap() : m_legend() 
	{
		MyColour local;
		//blu
		local.R = 0;
		local.G = 0;
		local.B = 255;
		m_legend.insert(std::make_pair(0., local));

		//violetto
		local.R = 128;
		local.G = 128;
		local.B = 255;
		m_legend.insert(std::make_pair(10., local));

		//azzurro
		local.R = 128;
		local.G = 255;
		local.B = 255;
		m_legend.insert(std::make_pair(20., local));

		//verde chiaro
		local.R = 0;
		local.G = 255;
		local.B = 64;
		m_legend.insert(std::make_pair(30., local));

		// verde scuro
		local.R = 0;
		local.G = 128;
		local.B = 0;
		m_legend.insert(std::make_pair(40., local));

		//	arancione
		local.R = 255;
		local.G = 128;
		local.B = 64;
		m_legend.insert(std::make_pair(50., local));

		// giallo
		local.R = 255;
		local.G = 255;
		local.B = 0;
		m_legend.insert(std::make_pair(60., local));

		// rosso
		local.R = 255;
		local.G = 0;
		local.B = 0;
		m_legend.insert(std::make_pair(70., local));

		//	marrone
		local.R = 128;
		local.G = 64;
		local.B = 0;
		m_legend.insert(std::make_pair(80., local));

		//	verde oliva
		local.R = 128;
		local.G = 128;
		local.B = 0;
		m_legend.insert(std::make_pair(90., local));

		//	grigio
		local.R = 164;
		local.G = 164;
		local.B = 164;
		m_legend.insert(std::make_pair(100., local));
	}

	inline MyColour getColor(double _val) 
	{
		int l_val = _val / 10.;
		return m_legend[l_val*10];
	}
};

//////////////////////////////////////////////////////////////////////////
void DrawAllPolygons(HDC hdc, int width, int height, int thickness)
{
	if( g_coverageTest == NULL )
		return;

	srand ( time(NULL) );
	if( g_drawone )
	{
		MyColour Col;
		Col.setRandom();

		switch(g_TTRR)
		{
		case TRIANGLE:
			//	DrawTriangle( &TT, hdc, width, height, Col.R, Col.G, Col.B, thickness);
			return;
		case RECTANGLE:
		case SQUARE:
			DrawSquare(RR, hdc, width, height, Col.R, Col.G, Col.B, thickness);
		}

		return;
	}

	if(g_coverageTest == NULL)
		return;

	int l_nSquare;
	g_coverageTest->getNumOfSquare(l_nSquare);
	std::vector<SquarePtr> l_squares = g_coverageTest->getSquares();

	ColorMap l_colorMap; 

	for(size_t i = 0; i < l_squares.size(); ++i)
	{
		double l_energyVal = l_squares[i]->getEnergyValue();
		double l_thiefVal = l_squares[i]->getThiefValue();

		if (l_thiefVal > 0)
		{
			MyColour Col = l_colorMap.getColor( l_thiefVal * Robotics::GameTheory::g_maxValuePossible );
			if( l_squares[i]->isValid() && (g_drawSquare || l_squares[i]->isChanged()) )
				DrawSquare(l_squares[i], hdc, width, height, Col.R, Col.G, Col.B, thickness);
		} 
		else if (l_energyVal > 0)
		{
			MyColour Col = l_colorMap.getColor( l_energyVal * Robotics::GameTheory::g_maxValue );
			if( l_squares[i]->isValid() && (g_drawSquare || l_squares[i]->isChanged()) )
				DrawSquare(l_squares[i], hdc, width, height, Col.R, Col.G, Col.B, thickness);
		}
		else 
		{
			MyColour Col = l_colorMap.getColor( 0 );
			if( l_squares[i]->isValid() && (g_drawSquare || l_squares[i]->isChanged()) )
				DrawSquare(l_squares[i], hdc, width, height, Col.R, Col.G, Col.B, thickness);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void ReverseBoundary()
{
	int n;
	if( g_dim % 2 == 0 )
		n = g_dim/2;
	else
		n = (g_dim+1)/2;
	for ( int count = 0; count != n; count++ )
	{
		double x = g_boundary2D[count].coord()[0];
		double y = g_boundary2D[count].coord()[1];
		g_boundary2D[count]=makePoint( g_boundary2D[g_dim-count-1].coord(), g_metric );
		g_boundary2D[g_dim-count-1]= makePoint ( IDSReal2D(x,y), g_metric );
	}
}

//////////////////////////////////////////////////////////////////////////
void MakeCurve()
{
	g_dim = 2*g_window_camp+2;
	g_boundary2D.resize(g_dim);

	double l_ang;
	l_ang = g_window_ang2 - g_window_ang1;
	l_ang *= 2*PI/(360*g_window_camp);
	double l_angg1 = g_window_ang1;
	l_angg1 *= 2*IDSMath::Pi/(360);
	double l_angg2 = g_window_ang2;
	l_angg2 *= 2*IDSMath::Pi/(360);
	for( int i = 0; i != g_window_camp+1; i++ )
	{
		g_boundary2D[i]=
			makePoint( IDSReal2D( g_window_Ox + g_window_rad1*cos((double)(l_angg1+i*l_ang)),
			g_window_Oy + g_window_rad1*sin((double)-(l_angg1+i*l_ang)) ), g_metric);
		g_boundary2D[g_window_camp+1+i]=
			makePoint( IDSReal2D( g_window_Ox + g_window_rad2*cos((double)(l_angg2-i*l_ang)),
			g_window_Oy + g_window_rad2*sin((double)(l_angg2-i*l_ang))), g_metric);
	}
	if( l_ang > 0 )
		ReverseBoundary();
}

//////////////////////////////////////////////////////////////////////////
void SaveBoundary()
{
	int count;
	FILE*f = fopen("..\\Boundary.txt","w");
	fprintf(f,"%d\n\n", g_dim);
	for ( count = 0; count != g_dim; count++ )
		fprintf(f,"%d %lf %lf\n", count, g_boundary2D[count].coord()[0], g_boundary2D[count].coord()[1]);
	fclose(f);
}

//////////////////////////////////////////////////////////////////////////
bool LoadBoundary(bool inverti = false, int index = -1)
{
	int count = 0;
	FILE*f;
	if( index < -1 || index > 99 )
		return false;
	if( index == -1 )
	{
		f = fopen("..\\Boundary.txt","r");
		if (!f)
			return false;
	}
	else
	{
		if( g_drawing_mode == -3 )
		{
			const int nn = 25;
			char name[nn] = "..\\TestM\\Boundary   .txt";
			int t = index % 10;
			name[nn-6] = t+'0';
			t = (index-t)/10;
			name[nn-7] = t+'0';
			f = fopen(name,"r");
		}
		else
		{
			const int nn = 24;
			char name[nn] = "..\\Test\\Boundary   .txt";
			int t = index % 10;
			name[nn-6] = t+'0';
			t = (index-t)/10;
			name[nn-7] = t+'0';
			f = fopen(name,"r");
		}
		if (!f)
			return false;
	}
	fscanf(f,"%d",&g_dim);
	g_boundary2D.resize(g_dim);

	double minX = IDSMath::LargeReal;
	double maxX = -IDSMath::LargeReal;
	double minY = IDSMath::LargeReal;
	double maxY = -IDSMath::LargeReal;
	for ( count = 0; count != g_dim; count++ )
	{
		int qwe;
		fscanf(f,"%d",&qwe);
		if( count == 0 && qwe != 0 )
		{
			g_numberOfAgents = qwe;
			fscanf(f,"%d",&qwe);
		}
		double x,y;
		fscanf(f,"%lf",&x);
		fscanf(f,"%lf",&y);
		g_boundary2D[count]= makePoint( IDSReal2D(x,y), g_metric);

		if( x < minX )
			minX = x;
		if( x > maxX )
			maxX = x;
		if( y < minY )
			minY = y;
		if( y > maxY )
			maxY = y;
	}

	if( minX < 25 || minX > 9000 || maxX < 1000 || maxX > 9850 ||
		minY < -9185 || minY > -1000 || maxY < -8500 || maxY > -15 )
	{
		double scaleX = 9825/(maxX-minX);
		double scaleY = 9160/(maxY-minY);

		for ( count = 0; count != g_dim; count++ )
			g_boundary2D[count]=makePoint( 
			IDSReal2D( 
			(g_boundary2D[count].coord()[0]-minX)*scaleX, 
			-9185 + (g_boundary2D[count].coord()[1]-minY)*scaleY ), 
			g_metric );
	}

	g_counterclockwise = true;
	if( inverti )
	{
		g_counterclockwise = false;
		for ( count = 0; count != g_dim; count++ )
			g_boundary2D[count]=makePoint( IDSReal2D(kk - g_boundary2D[count].coord()[0], g_boundary2D[count].coord()[1]), g_metric);
	}
	fclose(f);
	return true;
}

//////////////////////////////////////////////////////////////////////////
void SetStandardBoundary()
{
	int count = 0;
	g_dim = g_boundary->size();
	g_boundary2D.resize(g_dim);
	for( LineString2DIterator it = g_boundary->begin() ; it != g_boundary->end() ; it++ )
	{
		g_boundary2D[count]= *it;
		count++;
	}
}

//////////////////////////////////////////////////////////////////////////
// mode:
// 0  :	 standard
// 1  :	 read from file
// 2  :	 read from file and invert
// 3  :	 written here
bool SetBoundary2D(int mode = 0)
{
	g_boundary2D.clear();
	g_counterclockwise = true;
	if( mode == 0 )
	{
		SetStandardBoundary();
		if( g_dim > 1 )
			SaveBoundary();
		return true;
	}
	else if( mode == 1 )
	{
		if(!LoadBoundary())
			return false;
	}

	g_boundary->clear();
	if(g_coverageTest)
	{
		delete g_coverageTest;
		g_coverageTest = NULL;
	}

	// Copy boundary
	g_boundary->insert(g_boundary->begin(), g_boundary2D.begin(), g_boundary2D.end());

	return true;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////WINDOW CONTROL//////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
int APIENTRY _tWinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPTSTR    lpCmdLine,
	int       nCmdShow)
{
	g_explorationFile.open("exploration.txt");
	g_explorationFile << "Exploration Rate:\n";

	srand( (unsigned int) time(NULL) );

	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// TODO: Place code here.
	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_CORRIDOR, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CORRIDOR));

	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	g_explorationFile.close();

	return (int) msg.wParam;
}

//////////////////////////////////////////////////////////////////////////
//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
//  COMMENTS:
//
//    This function and its usage are only necessary if you want this code
//    to be compatible with Win32 systems prior to the 'RegisterClassEx'
//    function that was added to Windows 95. It is important to call this function
//    so that the application will get 'well formed' small icons associated
//    with it.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_CORRIDOR));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_CORRIDOR);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

//////////////////////////////////////////////////////////////////////////
//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	HWND hWnd;

	hInst = hInstance; // Store instance handle in our global variable

	hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

	if (!hWnd)
	{
		return FALSE;
	}

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}

//////////////////////////////////////////////////////////////////////////
//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent, width, height, j=0;
	PAINTSTRUCT ps;
	HDC hdc;
	wstringstream tmp_str;
	RECT winrect;
	Point2D V1, //= makePoint( IDSReal2D(0.,0.), g_metric),
		V2, //= makePoint( IDSReal2D(0.,0.), g_metric),
		VB; //= makePoint( IDSReal2D(0.,0.), g_metric);
	LineString2DIterator vert,vert2;

	switch (message)
	{
	case WM_CREATE:

		g_boundary = new LineString2D;

		g_drawing_externalBoundary=true;
		g_drawing_thiefPosition=false; 
		g_drawing_sink=false;

	case WM_COMMAND:
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case ID_HELP_GUIDE:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_GUIDE), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		case ID_MODE_DRAWPATH:
			g_drawing_mode = 0;
			g_drawing_loadCounter = 0;

			g_boundary->clear();

			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}
			g_drawing_externalBoundary=true;
			g_drawing_thiefPosition=false;
			g_drawing_sink=false;

			break;
		case ID_MODE_LOADPATH:
			g_drawing_mode = 1;
			g_drawing_loadCounter = 0;
			g_boundary->clear();
			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}
			g_drawing_externalBoundary = false;
			g_drawing_thiefPosition = true;
			g_drawing_sink=false;

			break;
		case ID_MODE_DRAWCURVE:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_CURVE), hWnd, Curve);
			g_drawing_loadCounter = 0;
			g_boundary->clear();
			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}
			g_drawing_externalBoundary = false;
			g_drawing_thiefPosition = true;
			g_drawing_sink=false;
			break;
		case ID_MODE_TESTALLPAT:
			g_drawing_mode = -1;
			g_drawing_loadCounter = 0;
			g_boundary->clear();
			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}
			g_drawing_externalBoundary = false;
			g_drawing_thiefPosition = true;
			g_drawing_sink=false;
			break;
		case ID_MODE_TESTMULTISTARTPATHS:
			g_drawing_mode = -3;
			g_drawing_loadCounter = 0;
			g_boundary->clear();
			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}
			g_drawing_externalBoundary = false;
			g_drawing_thiefPosition = true;
			g_drawing_sink=false;
			break;
		case ID_FILE_NUMBER_OF_GUARDS:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_GUARDS), hWnd, SetNumberOfGuards);
			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}
			break;
		case ID_FILE_NUMBER_OF_STEPS:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_STEPS), hWnd, SetNumberOfSteps);
			break;
		case ID_FILE_MONITOR_STEP:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_STEPS), hWnd, SetMonitorStep);
			break;
		case ID_FILE_AGENTS_PERIOD:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_PERIOD), hWnd, SetAgentsPeriod);
			break;
		case ID_FILE_DRAWSQUARES:
			g_drawSquare = !g_drawSquare;
			{
				HMENU hmenu = GetMenu(hWnd);
				if(g_drawSquare)
					CheckMenuItem(hmenu,ID_FILE_DRAWSQUARES,MF_CHECKED);
				else
					CheckMenuItem(hmenu,ID_FILE_DRAWSQUARES,MF_UNCHECKED);
				EndMenu();
			}
			InvalidateRect(hWnd, NULL, TRUE);
			UpdateWindow(hWnd);
			break;
		case ID_FILE_HIGHLIGHT:
			g_drawone = !g_drawone;
			{HMENU hmenu = GetMenu(hWnd);
			if(g_drawone)
				CheckMenuItem(hmenu,ID_FILE_HIGHLIGHT,MF_CHECKED);
			else
				CheckMenuItem(hmenu,ID_FILE_HIGHLIGHT,MF_UNCHECKED);
			EndMenu();}
			InvalidateRect(hWnd, NULL, TRUE);
			UpdateWindow(hWnd);
			break;
		case ID_FILE_POTENTIAL:
			g_PrintPotential = !g_PrintPotential;
			{HMENU hmenu = GetMenu(hWnd);
			if(g_PrintPotential)
				CheckMenuItem(hmenu,ID_FILE_POTENTIAL,MF_CHECKED);
			else
				CheckMenuItem(hmenu,ID_FILE_POTENTIAL,MF_UNCHECKED);
			EndMenu();}
			if( g_drawing_mode < 0 )
				g_drawing_loadCounter--;

			break;
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
		case ID_FILE_PARETO:
			g_pareto = !g_pareto;
			{HMENU hmenu = GetMenu(hWnd);
			if(g_pareto)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_CHECKED);
				g_pareto = true;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_correlated)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_CHECKED);
				g_correlated = true;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_PIPIP)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_CHECKED);
				g_PIPIP=true;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_DISL)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			else 
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			EndMenu();}

			break;
			//////////////////////////////////////////////////////////////////////////
		case ID_FILE_CORRELATED:
			g_correlated = !g_correlated;
			{HMENU hmenu = GetMenu(hWnd);
			if(g_correlated)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_CHECKED);
				g_correlated = true;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_pareto)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_CHECKED);
				g_pareto = true;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_PIPIP)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_CHECKED);
				g_PIPIP=true;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_DISL)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			else 
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			EndMenu();}
			break;
			//////////////////////////////////////////////////////////////////////////
		case ID_FILE_DISL:
			g_DISL = !g_DISL;
			{HMENU hmenu = GetMenu(hWnd);
			if(g_DISL)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			else if(g_correlated)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_CHECKED);
				g_correlated = true;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_pareto)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_CHECKED);
				g_pareto = true;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_PIPIP)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_CHECKED);
				g_PIPIP=true;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else 
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			EndMenu();}
			break;
			//////////////////////////////////////////////////////////////////////////
		case ID_FILE_PIPIP:
			g_PIPIP = !g_PIPIP;
			{HMENU hmenu = GetMenu(hWnd);
			if(g_PIPIP)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_CHECKED);
				g_PIPIP=true;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_DISL)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			else if(g_correlated)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_CHECKED);
				g_correlated = true;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else if(g_pareto)
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_CHECKED);
				g_pareto = true;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_UNCHECKED);
				g_DISL=false;
			}
			else 
			{
				CheckMenuItem(hmenu,ID_FILE_CORRELATED,MF_UNCHECKED);
				g_correlated = false;
				CheckMenuItem(hmenu,ID_FILE_PARETO,MF_UNCHECKED);
				g_pareto = false;
				CheckMenuItem(hmenu,ID_FILE_DISL,MF_UNCHECKED);
				g_PIPIP=false;
				CheckMenuItem(hmenu,ID_FILE_PIPIP,MF_CHECKED);
				g_DISL=true;
			}
			EndMenu();}
			break;
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////

		case ID_FILE_DRAWREALPARTITION:
			g_drawRealArea = !g_drawRealArea;
			{HMENU hmenu = GetMenu(hWnd);
			if(g_drawRealArea)
				CheckMenuItem(hmenu,ID_FILE_DRAWREALPARTITION,MF_CHECKED);
			else
				CheckMenuItem(hmenu,ID_FILE_DRAWREALPARTITION,MF_UNCHECKED);
			EndMenu();}
			break;

		case ID_FILE_UNDO:
			if( g_drawing_externalBoundary )
			{
				g_boundary->pop_back();
				InvalidateRect(hWnd, NULL, TRUE);
				UpdateWindow(hWnd);
			}
			break;

		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);

		GetWindowRect(hWnd,&winrect);
		width=winrect.right-winrect.left;
		height=winrect.top-winrect.bottom;

		if(g_drawing_externalBoundary | g_drawing_thiefPosition | g_drawing_sink)
			DrawPath(g_boundary,hdc,width,height,255,0,0,1);
		else
		{
			MarkPoint((int)(g_thiefStartingPt.coord().v[0]*width),(int)(g_thiefStartingPt.coord().v[1]*height),hdc,128,0,128,10);
			MarkPoint((int)(g_sinkPt.coord().v[0]*width),(int)(g_sinkPt.coord().v[1]*height),hdc,128,0,128,10);
		}

		DrawAllPolygons(hdc,width,height,1);

		DrawSink( hdc, width, height, 0,0,0,1 );

		DrawGuards( hdc, width, height, 0,0,0,1 );

		if(g_coverageTest)
		{
			DrawString(hdc, width, height, g_boundary->at(0).coord().v[0], g_boundary->at(0).coord().v[1], 0,255,0, g_coverageTest->getExplorationRateStr() );
			DrawString(hdc, width, height, g_boundary->at(1).coord().v[0], g_boundary->at(1).coord().v[1], 0,255,0, g_coverageTest->getBatteryValueStr() );
		}

		EndPaint(hWnd, &ps);
		break;
	case WM_DESTROY:
		//deallocate list
		delete g_boundary;
		PostQuitMessage(0);
		break;
		///////User Interface

	case WM_LBUTTONUP:

		GetWindowRect(hWnd,&winrect);
		if(g_drawing_externalBoundary)
		{
			VB = makePoint( 
				IDSReal2D( ((double)LOWORD(lParam)/(winrect.right-winrect.left)*kk), 
				((double)HIWORD(lParam)/(winrect.top-winrect.bottom)*kk)), 
				g_metric );

			g_boundary->insert(g_boundary->end(),VB);
		}
		else if(g_drawing_thiefPosition)
		{
			g_thiefStartingPt = makePoint( 
				IDSReal2D( ((double)LOWORD(lParam)/(winrect.right-winrect.left)*kk), 
				((double)HIWORD(lParam)/(winrect.top-winrect.bottom)*kk)), 
				g_metric );

			if(!g_coverageTest)
			{
				try
				{
					vector<Point2D> v;
					for(int i = 0; i != g_dim; i++)
					{
						v.push_back(g_boundary2D[i]);
					}
					g_coverageTest= new CoverageTest(v,g_counterclockwise);
				}
				catch(...) 
				{
					MessageBox(hWnd,L"Error: bad Corridor!",L"Error",MB_OK | MB_ICONERROR);
				}

				Point2D l_thief = g_thiefStartingPt;

				if(g_coverageTest)
				{
					if( g_coverageTest->areaContains(g_thiefStartingPt) )
					{
						g_coverageTest->removeAllThieves();

						ThiefPtr l_agent = std::make_shared<Thief>(g_coverageTest->getAlgorithm()->getNumberOfAgent(), g_thiefStartingPt);
						g_coverageTest->setThief(g_thiefStartingPt, l_agent);

						g_coverageTest->exportOnFile("./scenario.dat");

						g_coverageTest->updateMonitor();
					}
				}
				if(g_coverageTest)
					g_coverageTest->FindSquare(l_thief, RR);
			}

			g_drawing_thiefPosition = false;
			g_drawing_sink = true;
		}
		else if(g_drawing_sink)
		{
			g_sinkPt = makePoint( 
				IDSReal2D( ((double)LOWORD(lParam)/(winrect.right-winrect.left)*kk), 
				((double)HIWORD(lParam)/(winrect.top-winrect.bottom)*kk)), 
				g_metric );

			if(g_coverageTest)
			{
				g_coverageTest->removeAllSinks();

				SinkPtr l_agent = std::make_shared<Sink>(g_coverageTest->getAlgorithm()->getNumberOfAgent(), g_sinkPt);
				g_coverageTest->setSink(g_sinkPt, l_agent);
			}
		}
		InvalidateRect(hWnd, NULL, TRUE);
		UpdateWindow(hWnd);

		break;

	case WM_RBUTTONUP:
		if(g_drawing_externalBoundary)
		{
			g_drawing_externalBoundary = false;
			g_drawing_thiefPosition = true;
			SetStandardBoundary();
		}
		else if(g_drawing_thiefPosition)
		{
			g_boundary->clear();

			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}

			g_drawing_externalBoundary = true;
			g_drawing_thiefPosition = false;
			g_drawing_sink = false;
		}
		else if(g_drawing_sink)
		{
			g_boundary->clear();

			if(g_coverageTest)
			{
				delete g_coverageTest;
				g_coverageTest = NULL;
			}

			g_drawing_externalBoundary = true;
			g_drawing_thiefPosition = false;
			g_drawing_sink = false;
		}
		InvalidateRect(hWnd, NULL, TRUE);
		UpdateWindow(hWnd);

		break;

	case WM_MOUSEWHEEL:
		//	Go Forward next step:

		if((int)wParam > 0)
			//scroll forward
		{
			if(g_coverageTest)
			{
				g_coverageTest->goForward(g_numberOfSteps, g_monitor);
			}
		}
		else if((int)wParam < 0)
			//scroll backward
		{
			if(0)
			{
				if(g_PrintPotential)
					g_coverageTest->printPotential("../Prova.txt", false);
				else
					g_coverageTest->printBenefit("../Prova.txt", false);
			}


			//g_coverageTest->printPotentialIndex("../performance_index.txt", false);
			//g_coverageTest->printNewPerformanceIndex("../performance_index.txt", false);
			if(g_coverageTest)
				g_coverageTest->printBenefit("../Prova.txt", false);
			//g_coverageTest->printExplorationRate("../performance_index.txt", false);

			//g_coverageTest->moveThief();
		}

		InvalidateRect(hWnd, NULL, TRUE);
		UpdateWindow(hWnd);

		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////
void setvalue(HWND hDlg,int nIDDlgItem,int& n)
{
	WORD len;
	char c[16];
	n = 0;
	// Get the characters. 
	len = SendDlgItemMessage(hDlg,nIDDlgItem,EM_LINELENGTH,(WPARAM) 0,(LPARAM) 0);
	SendDlgItemMessage(hDlg,nIDDlgItem,EM_GETLINE,(WPARAM) 0,(LPARAM) c);
	for( int i = 0; i != len; i++ )
	{
		int exp = len-i-1;
		n += (c[2*i]-48) * pow(10.0,exp);
	}
}

//////////////////////////////////////////////////////////////////////////
// Message handler for curve box.
INT_PTR CALLBACK Curve(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if ( LOWORD(wParam) == IDOK )
		{
			g_drawing_mode = 3;

			setvalue(hDlg, IDC_EDIT4, g_window_rad2);
			if( g_window_rad2 == 0 ){
				g_window_Ox=5000;
				g_window_Oy=-5000;
				g_window_rad1=2000;
				g_window_rad2=3500;
				g_window_ang1=90;
				g_window_ang2=0;
				g_window_camp=30;}
			else{
				setvalue(hDlg, IDC_EDIT1, g_window_Ox);
				setvalue(hDlg, IDC_EDIT2, g_window_Oy); g_window_Oy = -g_window_Oy;
				setvalue(hDlg, IDC_EDIT3, g_window_rad1);
				setvalue(hDlg, IDC_EDIT5, g_window_ang1);
				setvalue(hDlg, IDC_EDIT6, g_window_ang2);
				setvalue(hDlg, IDC_EDIT7, g_window_camp);}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		else if ( LOWORD(wParam) == IDCANCEL )
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////
// Message handler for start box.
INT_PTR CALLBACK SetNumberOfGuards(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if ( LOWORD(wParam) == IDOK )
		{
			setvalue(hDlg, IDC_EDITT, g_numberOfAgents);
			if( g_numberOfAgents <= 2 )
			{
				g_numberOfAgents = 3;
			}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		else if ( LOWORD(wParam) == IDCANCEL )
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////
// Message handler for start box.
INT_PTR CALLBACK SetNumberOfSteps(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if ( LOWORD(wParam) == IDOK )
		{
			setvalue(hDlg, IDC_EDITT, g_numberOfSteps);
			if( g_numberOfSteps <= 0 )
			{
				g_numberOfSteps = 1;
			}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		else if ( LOWORD(wParam) == IDCANCEL )
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////
// Message handler for start box.
INT_PTR CALLBACK SetMonitorStep(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if ( LOWORD(wParam) == IDOK )
		{
			setvalue(hDlg, IDC_EDITT, g_monitor);
			if( g_monitor <= 0 )
			{
				g_monitor = -1;
			}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		else if ( LOWORD(wParam) == IDCANCEL )
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////
// Message handler for start box.
INT_PTR CALLBACK SetAgentsPeriod(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if ( LOWORD(wParam) == IDOK )
		{
			setvalue(hDlg, IDC_EDITT, g_agentsPeriod);
			if( g_agentsPeriod <= 0 )
			{
				g_agentsPeriod = 1;
			}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		else if ( LOWORD(wParam) == IDCANCEL )
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::exportOnFile(std::string const& filename)
{
	int count;
	ofstream myfile;
	myfile.open (filename);

	myfile.precision(16.);

	myfile << g_dim << std::endl;

	for ( int count = 0; count != g_dim; count++ )
	{
		myfile << g_boundary2D[count].coord()[0] << std::endl;
		myfile << g_boundary2D[count].coord()[1] << std::endl;
	}

	std::vector<AgentPosition> l_positions; 
	m_algorithm->getGuardsPosition(l_positions);
	myfile << l_positions.size() << std::endl;
	for ( size_t count = 0; count < l_positions.size() ; count++ )
	{
		Point2D l_pos = l_positions[count].getPoint2D();
		myfile << l_pos[0] << std::endl;
		myfile << l_pos[1] << std::endl;
	}

	m_algorithm->getThievesPosition(l_positions);
	myfile << l_positions.size() << std::endl;
	for ( size_t count = 0; count < l_positions.size() ; count++ )
	{
		Point2D l_pos = l_positions[count].getPoint2D();
		myfile << l_pos[0] << std::endl;
		myfile << l_pos[1] << std::endl;
	}

	myfile << 0;
	myfile.close();

}

//////////////////////////////////////////////////////////////////////////
void CoverageTest::updateMonitor()
{
	return m_algorithm->updateMonitor();
}