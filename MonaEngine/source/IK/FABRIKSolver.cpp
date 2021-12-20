#include "FABRIKSolver.hpp"

namespace Mona {



	FABRIKSolver::FABRIKSolver(SimpleIKChain bindPoseChain, int numOfSteps, float threshold): IKSolver(bindPoseChain, numOfSteps, threshold){}

	void FABRIKSolver::IterateBackward(aiVector3D target) {
		int nodeIndex = m_bindPoseChain.getNumNodes()-1;
		aiVector3D newTarget = target;
		while (nodeIndex > 0) {
			// current node
			auto currNode = m_solvedChain.getChainNode(nodeIndex);
			auto currNodeTransform = currNode->getLocalTransform();
			aiVector3D currNodeTrans = IKUtils::getMatrixTranslation(currNodeTransform);
			aiVector3D diff = newTarget - currNodeTrans;
			auto newCurrNodeTransform = IKUtils::createTranslationMatrix(diff) * currNodeTransform;
			currNode->setLocalTransform(newCurrNodeTransform);
			// parent node
			auto parNode = m_solvedChain.getChainNode(nodeIndex-1);
			auto parNodeTransform = parNode->getLocalTransform();
			aiVector3D parNodeTrans = IKUtils::getMatrixTranslation(parNodeTransform);
			// calculamos direccion del nodo actualizado al nodo padre
			aiVector3D currToParDirection = parNodeTrans - newTarget;
			currToParDirection = currToParDirection.Normalize();
			aiVector3D currToParTrans = currToParDirection * m_bindPoseChain.getLinkLength();
			aiVector3D currNodeNewTrans = IKUtils::getMatrixTranslation(newCurrNodeTransform);
			// posicionamos el nodo padre a distancia largo de segmento del nodo, en la direccion recien calculada
			auto newParTransform = IKUtils::createTranslationMatrix(currToParTrans) * IKUtils::createTranslationMatrix(currNodeNewTrans);
			parNode->setLocalTransform(newParTransform);
			// actualizamos el target, ahora corresponde a la posicion del curr node
			newTarget = currNodeNewTrans;
			nodeIndex -= 1;
		}

	}

	void FABRIKSolver::IterateForward(aiVector3D base) {
		int nodeIndex;
		aiVector3D newTarget = base;
		for (nodeIndex = 0; nodeIndex< m_bindPoseChain.getNumNodes()-1; nodeIndex++) {
			// current node
			auto currNode = m_solvedChain.getChainNode(nodeIndex);
			auto currNodeTransform = currNode->getLocalTransform();
			aiVector3D currNodeTrans = IKUtils::getMatrixTranslation(currNodeTransform);
			aiVector3D diff = newTarget - currNodeTrans;
			auto newCurrNodeTransform = IKUtils::createTranslationMatrix(diff) * currNodeTransform;
			currNode->setLocalTransform(newCurrNodeTransform);
			// child node
			auto chNode = m_solvedChain.getChainNode(nodeIndex + 1);
			auto chNodeTransform = chNode->getLocalTransform();
			aiVector3D chNodeTrans = IKUtils::getMatrixTranslation(chNodeTransform);
			// calculamos direccion del nodo actualizado al nodo hijo
			aiVector3D currToChDirection = chNodeTrans - newTarget;
			currToChDirection = currToChDirection.Normalize();
			aiVector3D currToChTrans = currToChDirection * m_bindPoseChain.getLinkLength();
			aiVector3D currNodeNewTrans = IKUtils::getMatrixTranslation(newCurrNodeTransform);
			// posicionamos el nodo padre a distancia largo de segmento del nodo, en la direccion recien calculada
			auto newChTransform = IKUtils::createTranslationMatrix(currToChTrans) * IKUtils::createTranslationMatrix(currNodeNewTrans);
			chNode->setLocalTransform(newChTransform);
			// actualizamos el target, ahora corresponde a la posicion del curr node
			newTarget = currNodeNewTrans;
			nodeIndex -= 1;
		}

	}


	void FABRIKSolver::solve(aiVector3D target) {
		m_solvedChain = m_bindPoseChain;
		int chainLength = m_bindPoseChain.getNumNodes();
		aiMatrix4x4 baseTransform = m_bindPoseChain.getChainNode(0)->getLocalTransform();
		aiVector3D baseTranslation = IKUtils::getMatrixTranslation(baseTransform);
		for (int i = 0; i < m_numOfSteps; i++) {
			IterateBackward(target);
			IterateForward(baseTranslation);
			aiVector3D currEndEfectorTranslation = m_solvedChain.getChainNode(chainLength - 1)->getGlobalTranslation();
			float currError = (target - currEndEfectorTranslation).Length();
			if (currError < m_threshold) {
				break;
			}
		}
	}

}