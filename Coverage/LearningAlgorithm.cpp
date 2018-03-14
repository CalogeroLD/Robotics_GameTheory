#include "LearningAlgorithm.h"

#include "Guard.h"
#include "CoverageUtility.h"

#include "DiscretizedArea.h"
#include "Thief.h"
#include "Sink.h"
#include "Probability.h"

#include <fstream>
#include <sstream>
#include <string>
#include <set>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace IDS::BaseGeometry;

//////////////////////////////////////////////////////////////////////////
// calcolo della epsilon nell'articolo
double LearningAlgorithm::computeExplorationRate(std::shared_ptr<Guard> _agent)
{
	if( fabs(m_experimentalRate) > IDSMath::TOLERANCE )
		return m_experimentalRate;

	if(!_agent)
		_agent = *m_guards.begin();
	
	int Discretization_Col = m_space->getNumCol();
	int Discretization_Row = m_space->getNumRow();

	double rate = max(double(Discretization_Col), double(Discretization_Row)) + 1.;
	//double rate = max(double(Robotics::GameTheory::DISCRETIZATION_COL), double(Robotics::GameTheory::DISCRETIZATION_ROW)) + 1.;
	return pow(double(m_time)/double(_agent->getTrajectoryLength()) , -double(m_guards.size())/rate);
}

//////////////////////////////////////////////////////////////////////////
int LearningAlgorithm::getNumberOfSteps(double _stopRate)
{
	if( _stopRate < 0 || fabs(m_experimentalRate) > IDSMath::TOLERANCE )
		return N_MAX;
	
	return 5000; // CONTROLLARE
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::initialize()
{
	this->updateCounterOfVisibleSquare();

	for(std::set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		//	ogni agente guardia comunica con i vicini:
		this->communicate(*it);

		//	ogni agente guardia calcola la propria prima utilità:
		this->compute(*it);

		//	ogni agente guardia compie un'azione random:
		AgentPosition pos = (*it)->selectRandomFeasibleAction(m_space); // qui bisognerà tenere conto dell'heading
	}

	return;
}

//////////////////////////////////////////////////////////////////////////
std::string LearningAlgorithm::getExplorationRateStr()
{
	double l_exploration = this->computeExplorationRate();
	std::ostringstream strs;
	strs << l_exploration;
	return strs.str();
}

//////////////////////////////////////////////////////////////////////////
double LearningAlgorithm::getExplorationRate()
{
	return this->computeExplorationRate();
}

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::communicate(std::shared_ptr<Guard> _agent)
{
	// ogni agente guardia comunica ai vicini quali riquadri può esaminare:

	//	1) recupero il gruppo di agenti con cui posso comunicare
	set<GuardPtr> l_communicableAgents = _agent->getCommunicableAgents(m_guards);

	//	2) recupero i riquadri che riesco a vedere
	set<SquarePtr> l_visibleSquares = _agent->getVisibleSquares(m_space); //ok

	//	3) Spedisco il messaggio
	for(set<GuardPtr>::iterator it = l_communicableAgents.begin(); it != l_communicableAgents.end(); ++it)
	{
		GuardPtr l_agent = *it;
		l_agent->receiveMessage(l_visibleSquares);
	}

	return;
}


//double ProbabilityOfDetection(AreaCoordinate _center, int _row, int _col);
//This function takes into account the influence of sonar performances change with the distance from CameraPosition point
double ProbabilityOfDetection(AreaCoordinate _center, int _row, int _col) {
	double x_c = _center.row;
	double y_c = _center.col;
	double distance = sqrt(pow((x_c - _row), 2) + pow((y_c - _col), 2));
	double probability;

	probability = exp(-(pow(distance, 2)) / 100);
	return probability;
}

/*
double ProbabilityOfDetection(AreaCoordinate _center, int _row, int _col) {
	double p1, p2, p3, p4, p5, p6, p7;
	double fun;
	double x_c = _center.row;
	double y_c = _center.col;
	double distance = sqrt(pow((x_c - _row), 2) + pow((y_c - _col), 2));
	if (distance <= 83)
		fun = 0;
	if (distance >= 2*83 && distance <= 3*83)
		fun = 0.9995;
	if (distance >= 3*83 && distance <= 4*83)
		fun = 0.6500;
	if (distance >= 4*83 && distance <= 5*83)
		fun =0.8000;
	if (distance >= 5*83)
		fun = 0.28999;

	cout << "dist " << distance << endl;
	return fun;
}
*/

std::vector<double> LearningAlgorithm::ProbOfNeighbour(AreaCoordinate l_coord, int _id) {
	std::vector<double> prob;

	for (auto it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		if (_id != (*it)->getID()) {
			int contributorID = (*it)->getID();
			AreaCoordinate center_other1 = m_space->getCoordinate((*it)->getCurrentPosition().getPoint2D());
			double p_i = ProbabilityOfDetection(center_other1, l_coord.row, l_coord.col);
			if (p_i >= 0.1)
				prob.push_back(p_i);
		}
	}
	return prob;
}

double LearningAlgorithm::InclusioEsclusionPrinciple(std::vector<double> p)
{
	double sum = 0;
	double n = p.size();
	int n_int = floor(n);
	double	temp = 1;
	for (int i = 0; i < n; i++) {
		temp = temp * p.at(i);
		sum = sum + p.at(i);
		for (int j = i + 1; j < n; j++) {
			sum = sum - p.at(i)*p.at(j);
			for (int k = j + 1; k < n; k++) {
				sum = sum + p.at(i)*p.at(j)*p.at(k);
				for (int a = k + 1; a < n; a++) {
					sum = sum - p.at(i)*p.at(j)*p.at(k)*p.at(a);
					for (int b = a + 1; b < n; b++) {
						sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b);
						for (int c = b + 1; c < n; c++) {
							sum = sum - p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c);
							for (int d = c + 1; d < n; d++) {
								sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d);
								for (int e = d + 1; e < n; e++) {
									sum = sum - p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d);
									for (int f = e + 1; f < n; f++) {
										sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f);
										for (int g = f + 1; g < n; g++) {
											sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g);
											for (int h = g + 1; h < n; h++) {
												sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g)*p.at(h);
												for (int l = h + 1; l < n; l++) {
													sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g)*p.at(h)*p.at(l);
													for (int m = l + 1; m < n; m++) {
														sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g)*p.at(h)*p.at(l)*p.at(m);
														for (int o = m + 1; o < n; o++) {
															sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g)*p.at(h)*p.at(l)*p.at(m)*p.at(o);
															for (int q = o + 1; q < n - 1; q++) {
																sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g)*p.at(h)*p.at(l)*p.at(m)*p.at(o)*p.at(q);
																for (int r = q + 1; r < n - 1; r++) {
																	sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g)*p.at(h)*p.at(l)*p.at(m)*p.at(o)*p.at(q)*p.at(r);
																	for (int s = r + 1; s < n - 1; s++) {
																		sum = sum + p.at(i)*p.at(j)*p.at(k)*p.at(a)*p.at(b)*p.at(c)*p.at(d)*p.at(e)*p.at(f)*p.at(g)*p.at(h)*p.at(l)*p.at(m)*p.at(o)*p.at(q)*p.at(r)*p.at(s);
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	if (n_int % 2 == 0) {
		sum = sum + pow((-1.0), (n - 1)) *temp;
	}
	return sum;
}

void LearningAlgorithm::compute(std::shared_ptr<Guard> _agent)
{
	double l_benefit = 0;
	// all information i need about current _agent
	AreaCoordinate p_center = m_space->getCoordinate(_agent->getCurrentPosition().getPoint2D());
	int identificator = _agent->getID();
	std::vector<AreaCoordinate> l_coord = _agent->getCurrentPosition().getCoverage(m_space);

	// run all sub-regions of _agent footprint
	for (int i = 0; i < l_coord.size(); ++i)
	{
		std::shared_ptr<Square> temp_square = m_space->getSquare(l_coord[i]);
		double W_q = temp_square->getThiefValue();

		if (!temp_square->isValid())
			continue;

		int l_nq = temp_square->getTheNumberOfAgent();
		double Pod_agent = ProbabilityOfDetection(p_center, l_coord[i].row, l_coord[i].col);
		std::vector<double> prob = this->ProbOfNeighbour(l_coord.at(i), identificator);
		double ProbOtherDetect = this->InclusioEsclusionPrinciple(prob);

		if (ProbOtherDetect > 1) {
			ProbOtherDetect = 1;
			//cout << "fail " << endl;
		}
		if (ProbOtherDetect < 0) {
			ProbOtherDetect = 0;
			//cout << "fail 0" << ProbOtherDetect << endl;
		}
		double l_value = W_q * (Pod_agent + (1 - Pod_agent) * ProbOtherDetect);
		//cout << "ciao " << Pod_agent + (1 - Pod_agent) * ProbOtherDetect << endl;
		//cout << "Prob single " << Pod_agent << endl;
		l_benefit += l_value / double(l_nq);
	}
	l_benefit -= _agent->computeCurrentCosts();
	//l_benefit -= _agent->computeBatteryCosts(m_space);
	_agent->setCurrentPayoff(l_benefit);

	// Battery
	std::shared_ptr<Square> l_square = m_space->getSquare(_agent->getCurrentPosition().getPoint2D());

	if (m_space->isThereASink())
		_agent->updateBattery(MAXIMUM_BATTERY*LOSTBATTERY_PERSTEP*l_square->getEnergyValue());
	else
		_agent->updateBattery(MAXIMUM_BATTERY);

	//	ogni agente guardia identifica le nuove azioni feasible:
	return;
}


//////////////////////////////////////////////////////////////////////////
/*
void LearningAlgorithm::compute(std::shared_ptr<Guard> _agent)
{
	//ogni agente guardia identifica la propria utilità:
	double l_benefit = 0;
	AgentPosition p = _agent->getCurrentPosition();
	AreaCoordinate p_center = m_space->getCoordinate(p.getPoint2D()); // centro del sensore
	
	std::vector <AreaCoordinate> l_coord = p.getCoverage(m_space); // tutti i quadrati dell'area del sensore
	std::set<std::shared_ptr<Square> > l_squares = _agent->getVisibleSquares(m_space);
	std::shared_ptr<Square> temp_square;
	
	for (int i = 0; i < l_coord.size(); ++i) { // ispeziona tutti i quadrati visti dai sensori per contare quanti robot lo vedono 
		temp_square = m_space->getSquare(l_coord[i]); // prende un quadrato delle m_lattice (griglia)
		if (!temp_square->isValid())
			continue;
		int l_nq = temp_square->getTheNumberOfAgent(); // numero di agenti che vedono il quadrato
		//std::cout << "num ag che vedono square " << l_nq << std::endl;
		double l_value = temp_square->getThiefValue(); // valore di probabilità di vedere il thief

		/* Probability of detection */
		/*l_value = l_value * ProbabilityOfDetection(p_center, l_coord[i].row, l_coord[i].col); // valore prob modificato
		l_benefit += l_value/ double(l_nq);

	}

	l_benefit -= _agent->computeCurrentCosts();
	//l_benefit -= _agent->computeBatteryCosts(m_space);
	_agent->setCurrentPayoff(l_benefit); // aggiorna il valore di benefit
	// Battery
	std::shared_ptr<Square> l_square = m_space->getSquare(_agent->getCurrentPosition().getPoint2D());
	if( m_space->isThereASink() )
		_agent->updateBattery(MAXIMUM_BATTERY*LOSTBATTERY_PERSTEP*l_square->getEnergyValue());
	else
		_agent->updateBattery(MAXIMUM_BATTERY);
	//	ogni agente guardia identifica le nuove azioni feasible:
	//this->computeFeasibleActions(_agent);
	return;
}*/

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::updateCounterOfVisibleSquare( std::shared_ptr<Guard> _agent )
{
	std::set<SquarePtr> l_visible = _agent->getVisibleSquares(m_space); // dipende anch'essa da getCoverage opportunamente modificata
	for(std::set<SquarePtr>::iterator it = l_visible.begin(); it != l_visible.end(); ++it)
	{
		(*it)->increaseCounter();
	}
}

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::updateCounterOfVisibleSquare()
{
	for(auto it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		updateCounterOfVisibleSquare(*it);
	}
}

///
bool LearningAlgorithm::forwardOneStep()
{
	double l_rate = this->computeExplorationRate(); // calcolo epsilon
	if(l_rate < 1.e-5)
		return false;

	//	UPDATE
	for(set<GuardPtr>::iterator it = m_guards.begin(); it!= m_guards.end(); ++it)
	{
		//	ogni agente guardia aggiorna la prossima azione da compiere:
		this->update(*it);
		/*double x, y, theta;
		std::vector<v_pos> v_p = this->getGuardsPosition1();
		ofstream SaveFile("PositionsOfGuards.txt");

		for (int i = 0; i < v_p.size(); i++)
		{
			x = v_p.at(i).x;
			y = v_p.at(i).y;
			theta = v_p.at(i).theta;
			//std::cout << "agente " << i << "-esimo " << " x : " << x  << " y : " << y << " theta : " << theta << endl;
			SaveFile << "Robot " << i << endl;
			SaveFile << x << endl;
			SaveFile << y << endl;
			SaveFile << theta << endl;
		}

		SaveFile.close();*/
	}

	updateCounterOfVisibleSquare();

	//	COMMUNICATE & COMPUTE:
	for(set<GuardPtr>::iterator it = m_guards.begin(); it!= m_guards.end(); ++it)
	{
		//	ogni agente guardia calcola la prima utilità:
		this->compute(*it);
		double payoff = (*it)->getCurrentPayoff();
	}

	return true;
}


//////////////////////////////////////////////////////////////////////////
bool LearningAlgorithm::forwardOneStep(std::shared_ptr<Guard> _agent)
{
	double l_rate = this->computeExplorationRate();
	if(l_rate < 1.e-5)
		return false;
	//	ogni agente guardia aggiorna la prossima azione da compiere:
	this->update(_agent);

	//	ogni agente guardia comunica con i vicini:
	this->communicate(_agent);

	//	ogni agente guardia calcola la prima utilità:
	this->compute(_agent);

	return true;
}

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::computeNextPosition()
{
	for(std::set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
		this->forwardOneStep(*it);

}

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::resetCounter()
{
	m_space->resetCounter();
}

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::resetValue()
{
	m_space->resetValue();
}

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::monitoringThieves(std::set< ThiefPtr > const& _agents)
{
	for(std::set< ThiefPtr >::const_iterator it = _agents.begin(); it != _agents.end(); ++it)
	{
		AgentPosition l_thiefPos = (*it)->getCurrentPosition();
		m_space->setThiefPosition(l_thiefPos);
	}
}

//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::monitoringSinks(std::set< SinkPtr > const& _agents)
{
	for(std::set< SinkPtr >::const_iterator it = _agents.begin(); it != _agents.end(); ++it)
	{
		AgentPosition l_sinkPos = (*it)->getCurrentPosition();
		m_space->setSinkPosition(l_sinkPos);
	}
}

/////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::getGuardsPosition(std::vector<AgentPosition> & _pos)
{
	_pos.clear();
	_pos.reserve(m_guards.size());
	for(set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		GuardPtr l_agent = *it;
		if(l_agent->isGuard())
		{
			_pos.push_back( l_agent->getCurrentPosition() );
		}
	}
	
}
///////////////// aggiunta da C. Li Destri /////////////////
std::vector<v_pos> LearningAlgorithm::getGuardsPosition1()
{
	v_pos vettore;
	std::vector<v_pos> _pos;
	_pos.clear();
	_pos.reserve(m_guards.size());
	for (set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		GuardPtr l_agent = *it;
		if (l_agent->isGuard())
		{
			vettore.x = l_agent->getCurrentPosition().getPoint2D().coord(0);
			vettore.y = l_agent->getCurrentPosition().getPoint2D().coord(1);
			vettore.theta = l_agent->getCurrentPosition().getHeading();
			_pos.push_back(vettore);
		}
	}
	return _pos;
}



//////////////////////////////////////////////////////////////////////////
void LearningAlgorithm::getGuardsSquare(std::vector<std::pair<SquarePtr, AgentActionIndex>> & _pos)
{
	_pos.clear();
	_pos.reserve(m_guards.size());
	for(set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		GuardPtr l_agent = *it;
		if(l_agent->isGuard())
		{
			_pos.push_back( std::make_pair(m_space->getSquare( l_agent->getCurrentPosition().getPoint2D() ),
				AgentActionIndex( l_agent->actualActionIndex(), l_agent->totalActions() ) ) );
		}
	}
}

////////////////////////////////////////////////////////////////////////// la usa solo per il disegno
void LearningAlgorithm::getGuardsCoverage( std::vector< std::vector<IDS::BaseGeometry::Point2D> > & _areas)
{
	for(set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it) // scorro su tutte le guardie
	{
		std::vector<IDS::BaseGeometry::Point2D> l_agentArea;
		GuardPtr l_agent = *it;
		Shape2D l_area = l_agent->getVisibleArea(); // return the shape with m_farRadius
		std::vector<Curve2D> l_bound = l_area.getBoundary(); // get the boundary of sensor area		

		for(size_t i = 0 ; i < l_bound.size(); ++i)
		{
			cout << l_bound.size() << std::endl;
			std::vector<IDS::BaseGeometry::Point2D> l_partial = l_bound[i].approxByPoints(1); // approssima la Curva con punti
			l_agentArea.insert(l_agentArea.end(), l_partial.begin(), l_partial.end()); // vettore con tutti i punti che approssimano area sensore
		}
		_areas.push_back(l_agentArea);
	}
}

//////////////////////////////////////////////////////////////////////////
int LearningAlgorithm::getGlobalTrajectoryCoverage()
{
	std::set< std::shared_ptr<Square> > l_globalSquare;
	for(set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		GuardPtr l_guard = *it;
		std::set< std::shared_ptr<Square> > l_squares = l_guard->getTrajectoryCoverage();
		l_globalSquare.insert(l_squares.begin(), l_squares.end());
	}
	return l_globalSquare.size();
}

//////////////////////////////////////////////////////////////////////////
double LearningAlgorithm::getBenefitValue()
{
	double l_total = 0.;
	for(std::set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		GuardPtr l_agent = *it;
		l_total += l_agent->getCurrentPayoff();
	}
	return l_total;
}

//////////////////////////////////////////////////////////////////////////
double LearningAlgorithm::getPotentialValue()
{
	double l_total = 0.;
	for(size_t i = 0;  i < m_space->m_lattice.size(); ++i)
	{
		double l_partialTotal = 0.;
		double l_Wq = m_space->m_lattice[i]->getThiefValue(); // W(q)
		for(int j = 1; j <= m_space->m_lattice[i]->getTheNumberOfAgent(); ++j)
		{
			l_partialTotal += l_Wq / j;
		}
		l_total += l_partialTotal;
	}
	return l_total;
}

//////////////////////////////////////////////////////////////////////////
double LearningAlgorithm::getBatteryValue()
{
	double l_total = 0.;
	for(std::set<GuardPtr>::iterator it = m_guards.begin(); it != m_guards.end(); ++it)
	{
		GuardPtr l_agent = *it;
		l_total += l_agent->getBatteryValue();
	}
	return l_total / (double(m_guards.size()) * MAXIMUM_BATTERY);
}

//////////////////////////////////////////////////////////////////////////
std::string LearningAlgorithm::getBatteryValueStr()
{
	double l_value = this->getBatteryValue();
	std::ostringstream strs;
	strs << l_value;
	return strs.str();
}

