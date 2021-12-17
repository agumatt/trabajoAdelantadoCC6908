#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include "Animation/SkinnedMesh.hpp"
#include "iostream"


aiScene* animatedChainScene(int numOfSegments, float animDuration) {
	std::cout << "hola2" << std::endl;
	// creamos la escena que contiene la cadena
	aiScene* scene = new aiScene();
	std::vector<aiMesh*>* meshes = new std::vector<aiMesh*>; // meshes para agregar a la escena
	// create vector with nodes. nombramos nodos y huesos correspondientes con indice de asignacion
	std::vector<aiNode*>* nodes = new std::vector<aiNode*>;
	// hay una malla por cada nodo (esfera(joint) o cubo(link))
	// el nodo 0_joint será el nodo raiz de la escena.
	bool configEndEffector = false;
	std::cout << "hola3" << std::endl;
	unsigned int i = 0;
	for (i = 0; i <= numOfSegments; i++) {
		if (i == numOfSegments) {
			configEndEffector = true;
		}
		std::string jointName = std::to_string(i) + "_joint";
		if (configEndEffector) {
			std::string jointName = "endEffector";
		}
		std::cout << "hola4" << std::endl;
		// create joint node
		aiNode* jointNode = new aiNode(jointName);
		(*nodes).push_back(jointNode);
		unsigned int meshIndex = 2 * i;
		(*nodes).back()->mMeshes = &meshIndex;
		(*nodes).back()->mNumMeshes = 1;
		std::cout << "hola5" << std::endl;
		// create joint mesh
		aiMesh* sphereMesh = Mona::Mesh::sphereMeshData();
		std::cout << "hola5.1" << std::endl;
		(*meshes).push_back(sphereMesh);
		std::cout << "hola6" << std::endl;
		// add bone
		aiBone* jointBone = new aiBone();
		jointBone->mName = jointName;
		std::vector<unsigned int> sphereIndices = {};
		unsigned int j = 0;
		for (j = 0; j < sphereMesh->mNumFaces; j++) {
			aiFace f = sphereMesh->mFaces[j];
			sphereIndices.push_back(f.mIndices[0]);
			sphereIndices.push_back(f.mIndices[1]);
			sphereIndices.push_back(f.mIndices[2]);
		}
		std::cout << "hola7" << std::endl;
		// remove duplicated indices
		std::cout << "numWeights" << sphereIndices.size() << std::endl;
		std::sort(sphereIndices.begin(), sphereIndices.end());
		sphereIndices.erase(std::unique(sphereIndices.begin(), sphereIndices.end()), sphereIndices.end());
		jointBone->mNumWeights = sphereIndices.size();
		std::vector<aiVertexWeight> *jointWeights = new std::vector < aiVertexWeight>;
		std::cout << "hola7.1" << std::endl;
		std::cout << "numWeights"<< jointBone->mNumWeights << std::endl;
		/*for (j = 0; jointBone->mNumWeights; j++) {
			std::cout << j << std::endl;
			aiVertexWeight* w = new aiVertexWeight();
			w->mVertexId = sphereIndices[j];
			w->mWeight = 1.0f;
			(*jointWeights).push_back(*w);
		}*/
		aiVertexWeight* w = new aiVertexWeight();
		w->mVertexId = sphereIndices[0];
		w->mWeight = 1.0f;
		(*jointWeights).push_back(*w);
		std::cout << "hola7.2" << std::endl;
		jointBone->mWeights = &(*jointWeights)[0];
		(*meshes).back()->mBones = &jointBone;

		std::cout << "hola8" << std::endl;
		if (!configEndEffector) { // si llegamos al endEffector, no hace falta agregar un link luego.
			// create link node
			std::string linkName = std::to_string(i) + "_link";
			aiNode* linkNode = new aiNode(linkName);
			(*nodes).push_back(linkNode);
			meshIndex = 2 * i + 1;
			(*nodes).back()->mMeshes = &meshIndex;
			(*nodes).back()->mNumMeshes = 1;
			// create link mesh
			aiMesh* cubeMesh = Mona::Mesh::cubeMeshData();
			(*meshes).push_back(cubeMesh);
			// add bone
			aiBone* linkBone = new aiBone();
			linkBone->mName = linkName;
			std::vector<unsigned int> cubeIndices = {};
			for (j = 0; j < cubeMesh->mNumFaces; j++) {
				aiFace f = cubeMesh->mFaces[j];
				cubeIndices.push_back(f.mIndices[0]);
				cubeIndices.push_back(f.mIndices[1]);
				cubeIndices.push_back(f.mIndices[2]);
			}
			// remove duplicated indices
			std::sort(cubeIndices.begin(), cubeIndices.end());
			cubeIndices.erase(std::unique(cubeIndices.begin(), cubeIndices.end()), cubeIndices.end());
			linkBone->mNumWeights = cubeIndices.size();
			std::vector<aiVertexWeight>* linkWeights = new std::vector<aiVertexWeight>;
			for (j = 0; linkBone->mNumWeights; j++) {
				aiVertexWeight* w = new aiVertexWeight();
				w->mVertexId = cubeIndices[j];
				w->mWeight = 1.0f;
				(*linkWeights).push_back(*w);
			}
			linkBone->mWeights = &(*linkWeights)[0];
			linkBone->mOffsetMatrix;
			(*meshes).back()->mBones = &linkBone;
		}
	}
	std::cout << "hola9" << std::endl;
	// agregamos valores de mallas a escena
	scene->mNumMeshes = (*meshes).size();
	scene->mMeshes = &(*meshes)[0];
	// asignamos relaciones padre-hijo
	scene->mRootNode = (*nodes)[0];
	(*nodes)[0]->mChildren = &(*nodes)[1];
	(*nodes)[0]->mNumChildren = 1;
	(*nodes)[0]->mParent = nullptr;
	for (i = 1; i < 2 * numOfSegments; i++) {
		(*nodes)[i]->mChildren = &(*nodes)[i + 1];
		(*nodes)[i]->mNumChildren = 1;
		(*nodes)[i]->mParent = (*nodes)[i - 1];
	}
	// para el end-effector
	(*nodes)[2 * numOfSegments]->mChildren = nullptr;
	(*nodes)[2 * numOfSegments]->mNumChildren = 0;
	(*nodes)[2 * numOfSegments]->mParent = (*nodes)[2 * numOfSegments - 1];

	std::cout << "hola10" << std::endl;
	// por ultimo seteamos las transformaciones
	// los dos primeros nodos son especiales
	float bScale = 1.0f; // base scale
	float flLen = 6.0f; // first link relative length
	float flThin = 0.8f; // make first link thinner
	float scaleDown = 0.8f; // reduce size of joints and links further in the chain
	float linkCurrentLength = bScale * flLen;
	aiVector3D scaling = aiVector3D(bScale, bScale, bScale); // escalamiento base
	aiQuaternion rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
	aiVector3D translation = aiVector3D(0.0f, 0.0f, 0.0f);
	aiMatrix4x4 jointTransform = aiMatrix4x4(scaling, rotation, translation);
	aiMatrix4x4 finalJointTransform = jointTransform;
	(*nodes)[0]->mTransformation = finalJointTransform;
	// link
	scaling = aiVector3D(flThin, flLen, flThin); // escalamiento base
	rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
	translation = aiVector3D(0.0f, (bScale * flLen) / 2, 0.0f); // dejamos la base del primer link en (0,0,0)
	aiMatrix4x4 linkTransform = aiMatrix4x4(scaling, rotation, translation);
	aiMatrix4x4 finalLinkTransform = linkTransform;
	(*nodes)[1]->mTransformation = finalLinkTransform;

	// acc transform
	aiMatrix4x4 accTransform = finalLinkTransform * finalJointTransform;
	// acc joint transform
	aiMatrix4x4 accJointTransform = jointTransform;
	// acc link transform
	aiMatrix4x4 accLinkTransform = linkTransform;
	// joint 

	std::cout << "hola11" << std::endl;
	configEndEffector = false;
	for (i = 1; i <= numOfSegments; i++) {
		if (i == numOfSegments) {
			configEndEffector = true;
			scaleDown = 1.0f;
		}
		// joint
		scaling = aiVector3D(scaleDown, scaleDown, scaleDown);
		rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
		translation = aiVector3D(0.0f, linkCurrentLength, 0.0f);
		jointTransform = aiMatrix4x4(scaling, rotation, translation);
		finalJointTransform = jointTransform * accJointTransform * accTransform.Inverse();
		(*nodes)[2 * i]->mTransformation = finalJointTransform;
		// update acc transform
		accTransform = finalJointTransform * accTransform;
		// set inverseBindMat
		(*meshes)[2 * i]->mBones[0]->mOffsetMatrix = accTransform.Inverse();
		// update acc joint transform
		accJointTransform = jointTransform * accJointTransform;
		
		if (!configEndEffector) {
			// link
			scaling = aiVector3D(scaleDown, scaleDown, scaleDown);
			rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
			translation = aiVector3D(0.0f, linkCurrentLength, 0.0f);
			linkTransform = aiMatrix4x4(scaling, rotation, translation);
			finalLinkTransform = linkTransform * accLinkTransform * accTransform.Inverse();
			(*nodes)[2 * i + 1]->mTransformation = finalLinkTransform;
			// update acc transform
			accTransform = finalLinkTransform * accTransform;
			// set inverseBindMat
			(*meshes)[2 * i + 1]->mBones[0]->mOffsetMatrix = accTransform.Inverse();
			// update acc link transform
			accLinkTransform = linkTransform * accLinkTransform;

			// update link length
			linkCurrentLength = linkCurrentLength * scaleDown;
		}

	}
	std::cout << "hola12" << std::endl;
	// animacion base
	aiAnimation* animation = new aiAnimation();
	scene->mAnimations = &animation;
	scene->mNumAnimations = 1;
	animation->mTicksPerSecond = 10.0f;
	animation->mDuration = animDuration* animation->mTicksPerSecond;
	animation->mNumChannels = (*nodes).size();
	std::vector<aiNodeAnim*>* channels = new std::vector<aiNodeAnim*>;

	// pasamos por cada nodo
	for (i = 0; i < animation->mNumChannels; i++) {
		std::string nodeName = ((*nodes)[i]->mName).C_Str();
		aiNodeAnim* anim = new aiNodeAnim();
		std::vector<aiVectorKey> pos = {aiVectorKey(0.0f, aiVector3D(0.0f,0.0f,0.0f)), aiVectorKey(animation->mDuration, aiVector3D(0.0f, 0.0f, 0.0f))};
		std::vector<aiQuatKey> rot = {aiQuatKey(0.0f, aiQuaternion(0.0f, 0.0f, 0.0f)), aiQuatKey(animation->mDuration, aiQuaternion(0.0f, 0.0f, 0.0f))};
		std::vector<aiVectorKey> scl = {aiVectorKey(0.0f, aiVector3D(1.0f, 1.0f, 1.0f)), aiVectorKey(animation->mDuration, aiVector3D(1.0f, 1.0f, 1.0f))};
		anim->mNodeName = nodeName;
		anim->mNumPositionKeys = 2;
		anim->mNumRotationKeys = 2;
		anim->mNumScalingKeys = 2;
		anim->mPositionKeys = &pos[0];
		anim->mRotationKeys = &rot[0];
		anim->mScalingKeys = &scl[0];
		(*channels).push_back(anim);
	}
	animation->mChannels = &(*channels)[0];
	
	return scene;
}


class AnimatedChain :public Mona::GameObject {

private:
	aiScene* m_chainScene;
	Mona::SkeletalMeshHandle m_skeletalMesh;

private:
	void SetTargetPosition(const glm::vec3 position) {}

	void UpdateTargetPosition(Mona::World& world) {	}


	void UpdateSteeringBehaviour() {}

	void UpdateAnimationState() {}

public:
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		UpdateTargetPosition(world);
		UpdateSteeringBehaviour();
		UpdateAnimationState();
	};
	virtual void UserStartUp(Mona::World& world) noexcept {
		int numOfSegments = 10;
		float animDuration = 1.5f;
		std::cout << "hola1" << std::endl;
		m_chainScene = animatedChainScene(numOfSegments, animDuration);
		Mona::Skeleton skeleton = Mona::Skeleton(m_chainScene);
		std::shared_ptr<Mona::Skeleton> skeletonPtr = std::shared_ptr<Mona::Skeleton>(&skeleton);
		Mona::SkinnedMesh skinnedMesh = Mona::SkinnedMesh(skeletonPtr, m_chainScene);
		std::shared_ptr<Mona::SkinnedMesh> skinnedMeshPtr = std::shared_ptr<Mona::SkinnedMesh>(&skinnedMesh);
		Mona::AnimationClip animationClip = Mona::AnimationClip(m_chainScene, skeletonPtr);
		std::shared_ptr<Mona::AnimationClip> animationClipPtr = std::shared_ptr<Mona::AnimationClip>(&animationClip);
		//Mona::Material material = world.CreateMaterial(Mona::MaterialType::DiffuseFlat);
		auto materialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		//materialPtr->SetDiffuseColor(glm::vec3(0.3f, 0.3f, 0.75f));
		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMeshPtr, animationClipPtr, materialPtr);
	}

};

Mona::CameraHandle CreateCamera(Mona::World& world) {
	auto camera = world.CreateGameObject<Mona::GameObject>();
	auto cameraTransform = world.AddComponent<Mona::TransformComponent>(camera);
	cameraTransform->SetTranslation(glm::vec3(0.0f, -12.0f, 7.0f));
	cameraTransform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.7f);
	auto cameraComp = world.AddComponent<Mona::CameraComponent>(camera);
	world.SetMainCamera(cameraComp);
	return cameraComp;
}

void CreatePlane(Mona::World& world) {
	auto plane = world.CreateGameObject<Mona::GameObject>();
	auto& meshManager = Mona::MeshManager::GetInstance();
	auto materialPtr = std::static_pointer_cast<Mona::UnlitFlatMaterial>(world.CreateMaterial(Mona::MaterialType::UnlitFlat));
	materialPtr->SetColor(glm::vec3(0.3, 0.5f, 0.7f));
	float planeScale = 10.0f;
	auto transform = world.AddComponent<Mona::TransformComponent>(plane);
	transform->SetScale(glm::vec3(planeScale));
	world.AddComponent<Mona::StaticMeshComponent>(plane, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Plane), materialPtr);
	Mona::BoxShapeInformation boxInfo(glm::vec3(planeScale, planeScale, planeScale));
	Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(plane, boxInfo, Mona::RigidBodyType::StaticBody, 1.0f, false, glm::vec3(0.0f, 0.0f, -planeScale));
}

void AddDirectionalLight(Mona::World& world, const glm::vec3& axis, float lightIntensity, float angle)
{
	auto light = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(light);
	transform->Rotate(axis, angle);
	world.AddComponent<Mona::DirectionalLightComponent>(light, lightIntensity * glm::vec3(1.0f));

}


class ExampleIKChain : public Mona::Application
{
public:
	ExampleIKChain() = default;
	~ExampleIKChain() = default;
	virtual void UserStartUp(Mona::World& world) noexcept override {
		world.SetAmbientLight(glm::vec3(0.1f));
		CreateCamera(world);
		CreatePlane(world);
		world.CreateGameObject<AnimatedChain>();
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-45.0f));
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-135.0f));
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {

	}

};

int main()
{
	ExampleIKChain exampleIKChain;
	Mona::Engine engine(exampleIKChain);
	engine.StartMainLoop();
}