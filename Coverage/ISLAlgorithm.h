///////////////////////////////////////////////////////////
//  ISLAlgorithm.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef ISL_ALGORITHM_H
#define ISL_ALGORITHM_H
#pragma once

//	Coverage
#include "CoverageExport.h"
#include "LearningAlgorithm.h"

#include <memory>
#include <set>

namespace Robotics 
{
	namespace GameTheory 
	{
		class Agent;
		class DiscretizedArea;

		/**
		*	\brief	Inhomogeneous Synchronous Learning Algorithm
		*	see M.Zhu and S.Martinez, 
		*	Distributed coverage game for mobile visual sensors (I): Reaching the set of Nash equilibria.
		*	in Proceedings of the 48th IEEE Conference on Decision and Control.
		*/
		class COVERAGE_API ISLAlgorithm : public LearningAlgorithm
		{
			int m_numMemory;

			std::shared_ptr<DiscretizedArea> m_space;

		protected:
			void update(std::shared_ptr<Agent> _agent);

			void communicate(std::shared_ptr<Agent> _agent);

			void compute(std::shared_ptr<Agent> _agent);

			void computeCurrentUtilities(std::shared_ptr<Agent> _agent);

			void sendInformationToNeighbors(std::shared_ptr<Agent> _agent);

			virtual void selectRandomFeasibleAction(std::shared_ptr<Agent> _agent);

			void selectBestMemoryAction(std::shared_ptr<Agent> _agent);

			void forwardOneStep(std::shared_ptr<Agent> _agent);
			void forwardOneStep();

			void updateCounterOfVisibleSquare( );
			void updateCounterOfVisibleSquare( std::shared_ptr<Agent> _agent );

		public:
			ISLAlgorithm(std::shared_ptr<DiscretizedArea> _space, int _numMemory = 2);

			virtual void computeNextPosition();

			virtual void initialize();

			virtual void resetCounter();
			
			virtual void updateTime(); 
		};
	}
}
#endif