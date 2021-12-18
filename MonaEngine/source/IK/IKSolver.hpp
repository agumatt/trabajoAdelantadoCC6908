#pragma once
#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP
#include "SimpleIKChain.hpp"

namespace Mona {

	class IKSolver {

		IKSolver(SimpleIKChain& chain) {
			m_chain = chain;
		}

		virtual void solve(aiVector3D target) = 0;

	private:
		SimpleIKChain m_chain;


	};




}

#endif
