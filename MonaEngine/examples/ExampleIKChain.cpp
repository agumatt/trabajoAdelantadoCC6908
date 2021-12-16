#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include "Animation/SkinnedMesh.hpp"


aiScene* animatedChainScene(int numOfSegments, float animDuration) {
	// creamos la escena que contiene la cadena
	aiScene* scene = new aiScene();
	std::vector<aiMesh*> meshes = {}; // meshes para agregar a la escena
	unsigned int i = 0;
	// create vector with nodes. nombramos nodos y huesos correspondientes con indice de asignacion
	std::vector<aiNode*> nodes = {};
	// hay una malla por cada nodo (esfera(joint) o cubo(link))
	// el nodo 0_joint será el nodo raiz de la escena.
	bool configEndEffector = false;
	for (i = 0; i <= numOfSegments; i++) {
		if (i == numOfSegments) {
			configEndEffector = true;
		}
		std::string jointName = std::to_string(i) + "_joint";
		if (configEndEffector) {
			std::string jointName = "endEffector";
		}
		// create joint node
		aiNode tmpNode = aiNode(jointName);
		aiNode* jointNode = &tmpNode;
		nodes.push_back(jointNode);
		unsigned int meshIndex = 2 * i;
		nodes[2 * i]->mMeshes = &meshIndex;
		nodes[2 * i]->mNumMeshes = 1;
		// create joint mesh
		aiMesh* sphereMesh = Mona::Mesh::sphereMeshData();
		meshes.push_back(sphereMesh);
		// add bone
		aiBone tmpBone = aiBone();
		aiBone* jointBone = &tmpBone;
		jointBone->mName = jointName;
		std::vector<unsigned int> sphereIndices = {};
		for (int i = 0; i < sphereMesh->mNumFaces; i++) {
			aiFace f = sphereMesh->mFaces[i];
			sphereIndices.push_back(f.mIndices[0]);
			sphereIndices.push_back(f.mIndices[1]);
			sphereIndices.push_back(f.mIndices[2]);
		}
		// remove duplicated indices
		std::sort(sphereIndices.begin(), sphereIndices.end());
		sphereIndices.erase(std::unique(sphereIndices.begin(), sphereIndices.end()), sphereIndices.end());
		jointBone->mNumWeights = sphereIndices.size();
		std::vector<aiVertexWeight> jointWeights = {};
		for (int i = 0; jointBone->mNumWeights; i++) {
			aiVertexWeight w = aiVertexWeight();
			w.mVertexId = sphereIndices[i];
			w.mWeight = 1.0f;
			jointWeights.push_back(w);
		}
		jointBone->mWeights = &jointWeights[0];
		meshes[2 * i]->mBones = &jointBone;

		if (!configEndEffector) { // si llegamos al endEffector, no hace falta agregar un link luego.
			// create link node
			std::string linkName = std::to_string(i) + "_link";
			tmpNode = aiNode(linkName);
			aiNode* linkNode = &tmpNode;
			nodes.push_back(linkNode);
			meshIndex = 2 * i + 1;
			nodes[2 * i + 1]->mMeshes = &meshIndex;
			nodes[2 * i + 1]->mNumMeshes = 1;
			// create link mesh
			aiMesh* cubeMesh = Mona::Mesh::cubeMeshData();
			meshes.push_back(cubeMesh);
			// add bone
			tmpBone = aiBone();
			aiBone* linkBone = &tmpBone;
			linkBone->mName = linkName;
			std::vector<unsigned int> cubeIndices = {};
			for (int i = 0; i < cubeMesh->mNumFaces; i++) {
				aiFace f = cubeMesh->mFaces[i];
				cubeIndices.push_back(f.mIndices[0]);
				cubeIndices.push_back(f.mIndices[1]);
				cubeIndices.push_back(f.mIndices[2]);
			}
			// remove duplicated indices
			std::sort(cubeIndices.begin(), cubeIndices.end());
			cubeIndices.erase(std::unique(cubeIndices.begin(), cubeIndices.end()), cubeIndices.end());
			linkBone->mNumWeights = cubeIndices.size();
			std::vector<aiVertexWeight> linkWeights = {};
			for (int i = 0; linkBone->mNumWeights; i++) {
				aiVertexWeight w = aiVertexWeight();
				w.mVertexId = cubeIndices[i];
				w.mWeight = 1.0f;
				linkWeights.push_back(w);
			}
			linkBone->mWeights = &linkWeights[0];
			linkBone->mOffsetMatrix;
			meshes[2 * i + 1]->mBones = &linkBone;
		}
	}
	// agregamos valores de mallas a escena
	scene->mNumMeshes = meshes.size();
	scene->mMeshes = &meshes[0];
	// asignamos relaciones padre-hijo
	scene->mRootNode = nodes[0];
	nodes[0]->mChildren = &nodes[1];
	nodes[0]->mNumChildren = 1;
	nodes[0]->mParent = nullptr;
	for (i = 1; i < 2 * numOfSegments; i++) {
		nodes[i]->mChildren = &nodes[i + 1];
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
	// link
	scaling = aiVector3D(flThin, flLen, flThin); // escalamiento base
	rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
	translation = aiVector3D(0.0f, (bScale * flLen) / 2, 0.0f); // dejamos la base del primer link en (0,0,0)
	aiMatrix4x4 linkTransform = aiMatrix4x4(scaling, rotation, translation);
	aiMatrix4x4 finalLinkTransform = linkTransform;
	nodes[1]->mTransformation = finalLinkTransform;

	// acc transform
	aiMatrix4x4 accTransform = finalLinkTransform * finalJointTransform;
	// acc joint transform
	aiMatrix4x4 accJointTransform = jointTransform;
	// acc link transform
	aiMatrix4x4 accLinkTransform = linkTransform;
	// joint 

	configEndEffector = false;
	for (int i = 1; i <= numOfSegments; i++) {
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
	aiAnimation tmpAnimation = aiAnimation();
	aiAnimation* animation = &tmpAnimation;
	scene->mAnimations = &animation;
	scene->mNumAnimations = 1;
	animation->mTicksPerSecond = 10.0f;
	animation->mDuration = animDuration* animation->mTicksPerSecond;
	animation->mNumChannels = nodes.size();
	std::vector<aiNodeAnim*> channels = {};

	// pasamos por cada nodo
	for (int i = 0; i < animation->mNumChannels; i++) {
		std::string nodeName = (nodes[i]->mName).C_Str();
		aiNodeAnim tmpNodeAnim = aiNodeAnim();
		aiNodeAnim* anim = &tmpNodeAnim;
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
		channels.push_back(anim);
	}
	animation->mChannels = &channels[0];
	
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