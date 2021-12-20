#pragma once
#ifndef SIMPLEIKCHAIN_HPP
#define SIMPLEIKCHAIN_HPP
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <assimp/scene.h>

namespace Mona {


	class SimpleIKChainNode {

	private:
		SimpleIKChainNode* m_parent;
		SimpleIKChainNode* m_child;
		aiMatrix4x4 m_localTransform;
	public:
		SimpleIKChainNode();
		aiMatrix4x4 getGlobalTransform();
		aiVector3D getGlobalTranslation();
		aiMatrix4x4 getLocalTransform();
		void setLocalTransform(aiMatrix4x4 t);
		SimpleIKChainNode* getParent();
		void setParent(SimpleIKChainNode* parentNode);
		SimpleIKChainNode* getChild();
		void setChild(SimpleIKChainNode* childNode);
		aiVector3D posWorldToLocal(aiVector3D worldVec);
		aiVector3D posLocalToWorld(aiVector3D localVec);
		

	};

	class SimpleIKChain {

	public:
		SimpleIKChain() = default;
		SimpleIKChain(int numNodes, float linkLength);
		int getNumNodes();
		SimpleIKChainNode* getChainNode(int index);
		float getLinkLength();
		void set(SimpleIKChain& copyChain);
	private:
		int m_numNodes;
		float m_linkLength;
		std::vector<SimpleIKChainNode*> m_IKNodes;
	};






}

#endif