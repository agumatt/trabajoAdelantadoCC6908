#include "FABRIKSolver.hpp"
#include <iostream>

namespace Mona {



	FABRIKSolver::FABRIKSolver(SimpleIKChain bindPoseChain, int numOfSteps, float threshold): IKSolver(bindPoseChain, numOfSteps, threshold){}

	void FABRIKSolver::IterateBackward(aiVector3D target) {
		int nodeIndex = m_bindPoseChain.getNumNodes()-1;
		SimpleIKChainNode* currNode;
		aiVector3D newTarget = target;
		auto testCurrNode = m_solvedChain.getChainNode(nodeIndex);
		while (nodeIndex >= 0) {
			// current node
			currNode = m_solvedChain.getChainNode(nodeIndex);
			aiVector3D currLocalTarget = currNode->posWorldToLocal(newTarget);
			aiMatrix4x4 moveCurrNodeToTarget = IKUtils::createTranslationMatrix(currLocalTarget);
			aiMatrix4x4 newCurrNodeTransform = moveCurrNodeToTarget * currNode->getLocalTransform();
			currNode->setLocalTransform(newCurrNodeTransform);
			// set next target
			if (nodeIndex > 0) {
				// parent node
				SimpleIKChainNode* parNode = m_solvedChain.getChainNode(nodeIndex - 1);
				// calculamos direccion del nodo actualizado al nodo padre
				aiVector3D currToParDirection = (parNode->getGlobalTranslation() - currNode->getGlobalTranslation()).Normalize();
				aiVector3D currToParVec = currToParDirection * m_bindPoseChain.getLinkLength();
				newTarget = currNode->getGlobalTranslation() + currToParVec;
			}
			nodeIndex -= 1;
		}

	}

	void FABRIKSolver::IterateForward(aiVector3D base) {
		int nodeIndex;
		SimpleIKChainNode* currNode;
		aiVector3D newTarget = base;
		for (nodeIndex = 0; nodeIndex<= m_bindPoseChain.getNumNodes()-1; nodeIndex++) {
			// current node
			currNode = m_solvedChain.getChainNode(nodeIndex);
			aiVector3D currLocalTarget = currNode->posWorldToLocal(newTarget);
			aiMatrix4x4 moveCurrNodeToTarget = IKUtils::createTranslationMatrix(currLocalTarget);
			aiMatrix4x4 newCurrNodeTransform = moveCurrNodeToTarget * currNode->getLocalTransform();
			currNode->setLocalTransform(newCurrNodeTransform);
			// set next target
			if (nodeIndex < m_bindPoseChain.getNumNodes()-1) {
				// child node
				SimpleIKChainNode* chNode = m_solvedChain.getChainNode(nodeIndex + 1);
				// calculamos direccion del nodo actualizado al nodo hijo
				aiVector3D currToChDirection = (chNode->getGlobalTranslation() - currNode->getGlobalTranslation()).Normalize();
				aiVector3D currToChVec = currToChDirection * m_bindPoseChain.getLinkLength();
				newTarget = currNode->getGlobalTranslation() + currToChVec;
			}
		}

	}


	void FABRIKSolver::solve(aiVector3D target) {
		m_solvedChain.set(m_bindPoseChain);
		int chainLength = m_bindPoseChain.getNumNodes();
		aiVector3D baseTranslation = m_bindPoseChain.getChainNode(0)->getGlobalTranslation();
		for (int i = 0; i < m_numOfSteps; i++) {

			IterateBackward(target);
			std::string j = "j";
			IterateForward(baseTranslation);
			aiVector3D currEndEfectorTranslation = m_solvedChain.getChainNode(chainLength - 1)->getGlobalTranslation();
			float currError = (target - currEndEfectorTranslation).Length();
			if (currError < m_threshold) {
				break;
			}
		}
	}

}