#include "SimpleIKChain.hpp"
#include "../Core/Log.hpp"


namespace Mona {

	SimpleIKChain::SimpleIKChain(int numNodes, float linkLength) {
		m_numNodes = numNodes;
		m_linkLength = linkLength;
		if (numNodes < 2 ) {
			MONA_LOG_ERROR("Chain must have at least 2 nodes");
		}
		for (int i = 0; i < numNodes; i++) {
			SimpleIKChainNode* node = new SimpleIKChainNode();
			m_IKNodes.push_back(node);
		}
		SimpleIKChainNode* rootNode = m_IKNodes[0];
		rootNode->setParent(nullptr);
		rootNode->setChild(m_IKNodes[1]);
		for (int i = 1; i < numNodes- 1; i++) {
			SimpleIKChainNode* node = m_IKNodes[i];
			node->setParent(m_IKNodes[i - 1]);
			node->setChild(m_IKNodes[i + 1]);
		}
		SimpleIKChainNode* endNode = m_IKNodes[numNodes-1];
		rootNode->setParent(m_IKNodes[numNodes - 2]);
		rootNode->setChild(nullptr);
	}
	int SimpleIKChain::getNumNodes() {
		return m_numNodes;
	}
	SimpleIKChainNode* SimpleIKChain::getChainNode(int index) {
		if (index < 0 || index >= m_numNodes) {
			MONA_LOG_ERROR("SimpleIKChain Error: Index {0} out of bounds", index);
		}
		return m_IKNodes[index];
	}





	SimpleIKChainNode::SimpleIKChainNode() {
		m_parent = nullptr;
		m_child = nullptr;
		m_localTransform = identityMatrix();
	}
	aiMatrix4x4 SimpleIKChainNode::getGlobalTransform() {
		SimpleIKChainNode* currNode = this;
		aiMatrix4x4 accTransform = currNode->m_localTransform;
		
		while (currNode->m_parent != nullptr) {
			currNode = currNode->m_parent;
			accTransform = accTransform * currNode->m_localTransform;
		}
		return accTransform;
	}

	aiMatrix4x4 SimpleIKChainNode::getLocalTransform() {
		return m_localTransform;
	}
	void SimpleIKChainNode::setLocalTransform(aiMatrix4x4 t) {
		m_localTransform = t;
	}

	SimpleIKChainNode* SimpleIKChainNode::getChild() {
		return m_child;
	}
	void SimpleIKChainNode::setChild(SimpleIKChainNode* childNode) {
		m_child = childNode;
	}

	SimpleIKChainNode* SimpleIKChainNode::getParent() {
		return m_parent;
	}
	void SimpleIKChainNode::setParent(SimpleIKChainNode* parentNode) {
		m_parent = parentNode;
	}

	aiMatrix4x4 SimpleIKChainNode::identityMatrix() {
		aiVector3D trans = aiVector3D(0.0f, 0.0f, 0.0f);
		aiQuaternion rot = aiQuaternion(0.0f, 0.0f, 0.0f);
		aiVector3D scal = aiVector3D(1.0f, 1.0f, 1.0f);
		return aiMatrix4x4(scal, rot, trans);
	}


}