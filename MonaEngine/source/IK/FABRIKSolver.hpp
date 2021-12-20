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

	private:
		void IterateBackward(aiVector3D target);
		void IterateForward(aiVector3D base);

};




}



#endif // !FABRIKSOLVER_HPP
