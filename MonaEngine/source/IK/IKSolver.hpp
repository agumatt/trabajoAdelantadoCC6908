#pragma once
#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP
#include "SimpleIKChain.hpp"
#include "IKUtils.hpp"

namespace Mona {

	class IKSolver {
	
	protected:
		SimpleIKChain m_bindPoseChain;
		SimpleIKChain m_solvedChain;
		int m_numOfSteps;
		float m_threshold;
		IKSolver() = default;
		IKSolver(SimpleIKChain bindPoseChain, int numOfSteps, float threshold) {
			m_bindPoseChain = bindPoseChain;
			m_solvedChain = SimpleIKChain(bindPoseChain.getNumNodes(), bindPoseChain.getLinkLength());
			m_numOfSteps = numOfSteps;
			m_threshold = threshold;
		}
		virtual void solve(aiVector3D target) {

		}

	public:

		SimpleIKChain getBindPoseChain() {
			return m_bindPoseChain;
		}
		SimpleIKChain getSolvedChain() {
			return m_solvedChain;
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


	};




}

#endif
