#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include "Animation/SkinnedMesh.hpp"
#include "iostream"


aiScene* animatedChainScene(int numOfSegments, float animDuration) {
	// creamos la escena que contiene la cadena
	aiScene* scene = new aiScene();
	int numOfMeshes = numOfSegments * 2 + 1;
	int numOfNodes = numOfSegments * 2 + 1;
	aiMesh** meshes = new aiMesh*[numOfMeshes]; // meshes para agregar a la escena
	// create vector with nodes. nombramos nodos y huesos correspondientes con indice de asignacion
	aiNode** nodes = new aiNode*[numOfNodes];
	// hay una malla por cada nodo (esfera(joint) o cubo(link))
	// el nodo 0_joint será el nodo raiz de la escena.
	bool configEndEffector = false;
	unsigned int i = 0;
	for (i = 0; i <= numOfSegments; i++) {
		std::string jointName = std::to_string(i) + "_joint";
		if (i == numOfSegments) {
			configEndEffector = true;
			jointName = "endEffector";
		}
		// create joint node
		aiNode* jointNode = new aiNode(jointName);
		nodes[2*i] = jointNode;
		unsigned int* jointMeshIndex = new unsigned int();
		*jointMeshIndex = 2 * i;
		nodes[2 * i]->mMeshes = jointMeshIndex;
		nodes[2 * i]->mNumMeshes = 1;
		// create joint mesh
		aiMesh* sphereMesh = Mona::Mesh::sphereMeshData();
		meshes[2 * i] = sphereMesh;
		// add bone
		aiBone** jointBones = new aiBone* [1];
		jointBones[0] = new aiBone();
		jointBones[0]->mName = jointName;
		std::vector<unsigned int> sphereIndices = {};
		unsigned int j = 0;
		for (j = 0; j < sphereMesh->mNumFaces; j++) {
			aiFace f = sphereMesh->mFaces[j];
			sphereIndices.push_back(f.mIndices[0]);
			sphereIndices.push_back(f.mIndices[1]);
			sphereIndices.push_back(f.mIndices[2]);
		}
		// remove duplicated indices
		std::sort(sphereIndices.begin(), sphereIndices.end());
		sphereIndices.erase(std::unique(sphereIndices.begin(), sphereIndices.end()), sphereIndices.end());
		jointBones[0]->mNumWeights = sphereIndices.size();
		aiVertexWeight* jointWeights = new aiVertexWeight[jointBones[0]->mNumWeights];
		for (j = 0; j < jointBones[0]->mNumWeights; j++) {
			aiVertexWeight* w = new aiVertexWeight();
			w->mVertexId = sphereIndices[j];
			w->mWeight = 1.0f;
			jointWeights[j] = *w;
		}
		jointBones[0]->mWeights = jointWeights;
		meshes[2 * i]->mBones = jointBones;
		meshes[2 * i]->mNumBones = 1;

		if (!configEndEffector) { // si llegamos al endEffector, no hace falta agregar un link luego.
			// create link node
			std::string linkName = std::to_string(i) + "_link";
			aiNode* linkNode = new aiNode(linkName);
			nodes[2 * i + 1] = linkNode;
			unsigned int* linkMeshIndex = new unsigned int();
			*linkMeshIndex = 2 * i + 1;
			nodes[2 * i + 1]->mMeshes = linkMeshIndex;
			nodes[2 * i + 1]->mNumMeshes = 1;
			// create link mesh
			aiMesh* cubeMesh = Mona::Mesh::cubeMeshData();
			meshes[2 * i + 1] = cubeMesh;
			// add bone
			aiBone** linkBones = new aiBone* [1];
			linkBones[0] = new aiBone();
			linkBones[0]->mName = linkName;
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
			linkBones[0]->mNumWeights = cubeIndices.size();
			aiVertexWeight* linkWeights = new aiVertexWeight[linkBones[0]->mNumWeights];
			for (j = 0; j < linkBones[0]->mNumWeights; j++) {
				aiVertexWeight* w = new aiVertexWeight();
				w->mVertexId = cubeIndices[j];
				w->mWeight = 1.0f;
				linkWeights[j] = *w;
			}
			linkBones[0]->mWeights = linkWeights;
			meshes[2 * i + 1]->mBones = linkBones;
			meshes[2 * i + 1]->mNumBones = 1;
		}
	}
	// agregamos valores de mallas a escena
	scene->mNumMeshes = numOfMeshes;
	scene->mMeshes = meshes;
	// asignamos relaciones padre-hijo
	scene->mRootNode = nodes[0];
	aiNode** rootChildren = new aiNode* [1];
	rootChildren[0] = nodes[1];
	nodes[0]->mChildren = rootChildren;
	nodes[0]->mNumChildren = 1;
	nodes[0]->mParent = nullptr;
	for (i = 1; i < 2 * numOfSegments; i++) {
		aiNode** children = new aiNode * [1];
		children[0] = nodes[i+1];
		nodes[i]->mChildren = children;
		nodes[i]->mNumChildren = 1;
		nodes[i]->mParent = nodes[i - 1];
	}
	// para el end-effector
	nodes[2 * numOfSegments]->mChildren = nullptr;
	nodes[2 * numOfSegments]->mNumChildren = 0;
	nodes[2 * numOfSegments]->mParent = nodes[2 * numOfSegments - 1];
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
	nodes[0]->mTransformation = finalJointTransform;
	// set inverseBindMat
	meshes[0]->mBones[0]->mOffsetMatrix = finalJointTransform.Inverse();
	// link
	scaling = aiVector3D(flThin, flLen, flThin); // escalamiento base
	rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
	translation = aiVector3D(0.0f, (bScale * flLen) / 2, 0.0f); // dejamos la base del primer link en (0,0,0)
	aiMatrix4x4 linkTransform = aiMatrix4x4(scaling, rotation, translation);
	aiMatrix4x4 finalLinkTransform = linkTransform;
	nodes[1]->mTransformation = finalLinkTransform;
	// set inverseBindMat
	meshes[1]->mBones[0]->mOffsetMatrix = (finalLinkTransform*finalJointTransform).Inverse();

	// acc transform
	aiMatrix4x4 accTransform = finalLinkTransform * finalJointTransform;
	// acc joint transform
	aiMatrix4x4 accJointTransform = jointTransform;
	// acc link transform
	aiMatrix4x4 accLinkTransform = linkTransform;
	// joint 
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
		nodes[2 * i]->mTransformation = finalJointTransform;
		// update acc transform
		accTransform = finalJointTransform * accTransform;
		// set inverseBindMat
		meshes[2 * i]->mBones[0]->mOffsetMatrix = accTransform.Inverse();
		// update acc joint transform
		accJointTransform = jointTransform * accJointTransform;
		
		if (!configEndEffector) {
			// link
			scaling = aiVector3D(scaleDown, scaleDown, scaleDown);
			rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
			translation = aiVector3D(0.0f, linkCurrentLength, 0.0f);
			linkTransform = aiMatrix4x4(scaling, rotation, translation);
			finalLinkTransform = linkTransform * accLinkTransform * accTransform.Inverse();
			nodes[2 * i + 1]->mTransformation = finalLinkTransform;
			// update acc transform
			accTransform = finalLinkTransform * accTransform;
			// set inverseBindMat
			meshes[2 * i + 1]->mBones[0]->mOffsetMatrix = accTransform.Inverse();
			// update acc link transform
			accLinkTransform = linkTransform * accLinkTransform;

			// update link length
			linkCurrentLength = linkCurrentLength * scaleDown;
		}

	}
	// animacion base
	aiAnimation** animations = new aiAnimation* [1];
	animations[0] = new aiAnimation();
	animations[0]->mTicksPerSecond = 10.0f;
	animations[0]->mDuration = animDuration* animations[0]->mTicksPerSecond;
	animations[0]->mNumChannels = numOfNodes;
	aiNodeAnim** channels = new aiNodeAnim*[animations[0]->mNumChannels];

	// pasamos por cada nodo
	for (i = 0; i < animations[0]->mNumChannels; i++) {
		std::string nodeName = (nodes[i]->mName).C_Str();
		aiNodeAnim* anim = new aiNodeAnim();
		aiVectorKey* pos = new aiVectorKey[2];
		pos[0] = aiVectorKey(0.0f, aiVector3D(0.0f, 0.0f, 0.0f));
		pos[1] = aiVectorKey(animations[0]->mDuration, aiVector3D(0.0f, 0.0f, 0.0f));
		aiQuatKey* rot = new aiQuatKey[2];
		rot[0] = aiQuatKey(0.0f, aiQuaternion(0.0f, 0.0f, 0.0f));
		rot[1] = aiQuatKey(animations[0]->mDuration, aiQuaternion(0.0f, 0.0f, 0.0f));
		aiVectorKey* scl = new aiVectorKey[2];
		scl[0] = aiVectorKey(0.0f, aiVector3D(1.0f, 1.0f, 1.0f));
		scl[1] = aiVectorKey(animations[0]->mDuration, aiVector3D(1.0f, 1.0f, 1.0f));
		anim->mNodeName = nodeName;
		anim->mNumPositionKeys = 2;
		anim->mNumRotationKeys = 2;
		anim->mNumScalingKeys = 2;
		anim->mPositionKeys = pos;
		anim->mRotationKeys = rot;
		anim->mScalingKeys = scl;
		channels[i] = anim;
	}
	animations[0]->mChannels = channels;
	scene->mAnimations = animations;
	scene->mNumAnimations = 1;
	
	return scene;
}


class AnimatedChain :public Mona::GameObject {

private:
	aiScene* m_chainScene;
	Mona::SkeletalMeshHandle m_skeletalMesh;
	Mona::TransformHandle m_transform;
	std::shared_ptr<Mona::AnimationClip> m_chainBaseAnimation;

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
		m_chainScene = animatedChainScene(numOfSegments, animDuration);
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);

		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& skeletonManager = Mona::SkeletonManager::GetInstance();
		auto& animationManager = Mona::AnimationClipManager::GetInstance();
		auto skeleton = skeletonManager.LoadSkeleton("chainSkeleton", m_chainScene);
		auto skinnedMesh = meshManager.LoadSkinnedMesh(skeleton, m_chainScene, "chainSkinnedMesh");
		m_chainBaseAnimation = animationManager.LoadAnimationClip("chainBaseAnimation", skeleton, m_chainScene);
		auto materialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat, true));
		materialPtr->SetDiffuseColor(glm::vec3(0.3f, 0.3f, 0.75f));
		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_chainBaseAnimation, materialPtr);
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