#include <glm/glm.hpp>
#include <vector>

namespace Mona {


	class SimpleIKChain {


		SimpleIKChain(int numNodes, float lenLinks);
		std::vector<SimpleIKChainNode*> m_IKNodes;




	};


	class SimpleIKChainNode {

	private:
		SimpleIKChainNode* m_parent;
		glm::fquat m_localRotation;
		glm::vec3 m_localTranslation;
	public:
		glm::fquat getGlobalRotation();
		glm::vec3 getGlobalTranslation();
		glm::fquat getLocalRotation();
		glm::vec3 getLocalTranslation();
		void setLocalRotation();
		void setLocalTranslation();
		SimpleIKChainNode* getParent();
		void setParent(SimpleIKChainNode& parentNode);
		

	};






}