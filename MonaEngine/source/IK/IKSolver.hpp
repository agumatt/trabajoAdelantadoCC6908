#pragma once
#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP
#include "SimpleIKChain.hpp"

namespace Mona {

	class IKSolver {
	
	protected:
		SimpleIKChain m_bindPoseChain;
		SimpleIKChain m_solvedChain;
		aiVector3D m_currentTarget;
		int m_numOfSteps;
		float m_threshold;
		IKSolver(SimpleIKChain bindPoseChain, int numOfSteps, float threshold) {
			m_bindPoseChain = bindPoseChain;
			m_solvedChain = bindPoseChain;
			SimpleIKChainNode* endEffector = bindPoseChain.getChainNode(bindPoseChain.getNumNodes() - 1);
			aiMatrix4x4 endEffectorTransform = endEffector->getGlobalTransform();
			aiVector3D pos;
			aiQuaternion rot;
			aiVector3D scl;
			endEffectorTransform.Decompose(scl, rot, pos);
			m_currentTarget = pos;
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

		static aiVector3D getMatrixTranslation(aiMatrix4x4 mat) {
			aiVector3D pos;
			aiQuaternion rot;
			aiVector3D scl;
			mat.Decompose(scl, rot, pos);
			return pos;
		}

		static aiVector3D getMatrixScaling(aiMatrix4x4 mat) {
			aiVector3D pos;
			aiQuaternion rot;
			aiVector3D scl;
			mat.Decompose(scl, rot, pos);
			return scl;
		}

		static aiQuaternion getMatrixRotation(aiMatrix4x4 mat) {
			aiVector3D pos;
			aiQuaternion rot;
			aiVector3D scl;
			mat.Decompose(scl, rot, pos);
			return rot;
		}

		static aiMatrix4x4 createRotationMatrix(aiQuaternion rot) {
			aiVector3D pos = aiVector3D(0.0f, 0.0f, 0.0f);
			aiVector3D scl = aiVector3D(1.0f, 1.0f, 1.0f);
			return aiMatrix4x4(scl, rot, pos);
		}

		static aiMatrix4x4 createTranslationMatrix(aiVector3D trans) {
			aiQuaternion rot = aiQuaternion(0.0f, 0.0f, 0.0f);
			aiVector3D scl = aiVector3D(1.0f, 1.0f, 1.0f);
			return aiMatrix4x4(scl, rot, trans);
		}

		static aiMatrix4x4 createScalingMatrix(aiVector3D scl) {
			aiQuaternion rot = aiQuaternion(0.0f, 0.0f, 0.0f);
			aiVector3D pos = aiVector3D(0.0f, 0.0f, 0.0f);
			return aiMatrix4x4(scl, rot, pos);
		}

		static aiMatrix4x4 identityMatrix() {
			aiVector3D trans = aiVector3D(0.0f, 0.0f, 0.0f);
			aiQuaternion rot = aiQuaternion(0.0f, 0.0f, 0.0f);
			aiVector3D scal = aiVector3D(1.0f, 1.0f, 1.0f);
			return aiMatrix4x4(scal, rot, trans);
		}

	};




}

#endif
