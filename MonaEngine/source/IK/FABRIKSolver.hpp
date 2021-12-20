#pragma once
#ifndef FABRIKSOLVER_HPP
#define FABRIKSOLVER_HPP
#include "SimpleIKChain.hpp"
#include "IKSolver.hpp"

namespace Mona {

	class FABRIKSolver : IKSolver {
	public:
		FABRIKSolver() = default;
		FABRIKSolver(SimpleIKChain bindPoseChain, int numOfSteps, float threshold);

		void solve(aiVector3D target);

		SimpleIKChain getBindPoseChain() {
			return m_bindPoseChain;
		}
		SimpleIKChain getSolvedChain() {
			return m_solvedChain;
		}
		aiVector3D getCurrentTarget() {
			return m_currentTarget;
		}
		int getNumberOfSteps() {
			return m_numOfSteps;
		}
		float getThreshold() {
			return m_threshold;
		}
		void setNumberOfSteps(int numOfSteps) {
			m_numOfSteps = numOfSteps;
		}
		void setThreshold(float threshold) {
			m_threshold = threshold;
		}

	private:
		void IterateBackward(aiVector3D target);
		void IterateForward(aiVector3D base);

};




}



#endif // !FABRIKSOLVER_HPP
