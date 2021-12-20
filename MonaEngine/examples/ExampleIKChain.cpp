#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include "Animation/SkinnedMesh.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include "IK/SimpleIKChain.hpp"
#include "IK/FABRIKSolver.hpp"
#include "IK/CCDSolver.hpp"
#include <imgui.h>
#include <iostream>

aiScene* animatedChainScene(int numOfSegments, float segmentLength, float animDuration, std::vector<aiMatrix4x4>* worldTransforms) {
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
	float flLen = segmentLength; // first link relative length to joint
	float flThin = 0.5f; // make first link thinner relative to joint
	float scaleDown = 1.0f; // reduce size of joints and links further in the chain
	aiMatrix4x4 accTransform = Mona::IKSolver::identityMatrix();
	// joint
	aiVector3D scaling = aiVector3D(bScale, bScale, bScale); // escalamiento base
	aiQuaternion rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
	aiVector3D translation = aiVector3D(0.0f, 0.0f, 0.0f);
	aiMatrix4x4 jointTransform = aiMatrix4x4(scaling, rotation, translation);
	aiMatrix4x4 finalJointTransform = jointTransform;
	nodes[0]->mTransformation = finalJointTransform;
	// acc transform
	accTransform = accTransform* finalJointTransform;
	worldTransforms->push_back(accTransform);
	// set inverseBindMat
	meshes[0]->mBones[0]->mOffsetMatrix = Mona::IKSolver::identityMatrix();
	// link
	scaling = aiVector3D(flThin, flThin, flLen); // escalamiento base
	rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
	translation = aiVector3D(0.0f, 0.0f, bScale*flLen); // dejamos la base del primer link en (0,0,0)
	aiMatrix4x4 linkTransform = aiMatrix4x4(scaling, rotation, translation);
	aiMatrix4x4 finalLinkTransform = linkTransform;
	nodes[1]->mTransformation = finalLinkTransform;
	// acc transform
	accTransform = accTransform * finalLinkTransform;
	worldTransforms->push_back(accTransform);
	// set inverseBindMat
	meshes[1]->mBones[0]->mOffsetMatrix = Mona::IKSolver::identityMatrix();
	// last local link transform
	aiMatrix4x4 lastLinkTransform = linkTransform;
	aiMatrix4x4 lastJointTransform = jointTransform;
	// joint 
	configEndEffector = false;
	float linkCurrentLength = bScale * flLen * 2;
	float linkCurrentBase = 0.0f;
	float moveUpwards;
	for (i = 1; i <= numOfSegments; i++) {
		if (i == numOfSegments) {
			configEndEffector = true;
			scaleDown = 1.5f;
		}
		// joint
		moveUpwards = linkCurrentBase - linkCurrentBase * scaleDown + linkCurrentLength;
		scaling = aiVector3D(scaleDown, scaleDown, scaleDown);
		rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
		translation = aiVector3D(0.0f, 0.0f, moveUpwards);
		jointTransform = aiMatrix4x4(scaling, rotation, translation);
		finalJointTransform = jointTransform * lastLinkTransform.Inverse();
		nodes[2 * i]->mTransformation = finalJointTransform;
		// acc transform
		accTransform = accTransform*finalJointTransform;
		worldTransforms->push_back(accTransform);
		// set inverseBindMat
		meshes[2 * i]->mBones[0]->mOffsetMatrix = Mona::IKSolver::identityMatrix();
		// update saved local joint transform
		lastJointTransform = finalJointTransform;
		
		if (!configEndEffector) {
			// link
			scaling = aiVector3D(scaleDown, scaleDown, scaleDown);
			rotation = aiQuaternion(0.0f, 0.0f, 0.0f);
			translation = aiVector3D(0.0f, 0.0f, moveUpwards);
			linkTransform = aiMatrix4x4(scaling, rotation, translation);
			finalLinkTransform = linkTransform * lastJointTransform.Inverse();
			nodes[2 * i + 1]->mTransformation = finalLinkTransform;
			// acc transform
			accTransform = accTransform*finalLinkTransform;
			worldTransforms->push_back(accTransform);
			// set inverseBindMat
			meshes[2 * i + 1]->mBones[0]->mOffsetMatrix = Mona::IKSolver::identityMatrix();
			// update saved local link transform
			lastLinkTransform = finalLinkTransform;

			// update link length
			linkCurrentBase = linkCurrentBase + linkCurrentLength;
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
		if (i == -6) {
			aiMatrix4x4 worldTrans = worldTransforms->at(i);
			aiVector3D aiPos = aiVector3D(0.0f, 0.0f, 0.0f);
			aiQuaternion aiRot = aiQuaternion(0.0f, 0.0f, 0.0f);
			aiVector3D aiScale = aiVector3D(1.0f, 1.0f, 1.0f);
			worldTrans.Decompose(aiScale, aiRot, aiPos);
			aiVector3D displaceToOriginV = -aiPos;
			aiVector3D displaceBackV = aiPos;
			aiMatrix4x4 displaceToOriginM = aiMatrix4x4(aiScale, aiRot, displaceToOriginV);
			aiRot = aiQuaternion(1.0f, 0.0f, 0.0f);
			aiScale = aiVector3D(1.0f, 1.0f, 1.0f);
			aiMatrix4x4 rotateAndDisplaceBackM = aiMatrix4x4(aiScale, aiRot, displaceBackV);
			aiMatrix4x4 finalMatrix = rotateAndDisplaceBackM * displaceToOriginM;
			finalMatrix.Decompose(aiScale, aiRot, aiPos);
			pos[1] = aiVectorKey(animations[0]->mDuration, aiPos);
			rot[1] = aiQuatKey(animations[0]->mDuration, aiRot);
			//scl[1] = aiVectorKey(animations[0]->mDuration, aiScale);
		}
		if (i == 0) {
			rot[1] = aiQuatKey(animations[0]->mDuration, aiQuaternion(1.0f, 0.0f, 0.0f));
		}
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


aiMatrix4x4 fixRotation(aiMatrix4x4 inputTransform, aiMatrix4x4 jointWorldTransform) {
	aiVector3D aiPos = aiVector3D(0.0f, 0.0f, 0.0f);
	aiQuaternion aiRot = aiQuaternion(0.0f, 0.0f, 0.0f);
	aiVector3D aiScale = aiVector3D(1.0f, 1.0f, 1.0f);
	jointWorldTransform.Decompose(aiScale, aiRot, aiPos);
	aiVector3D displaceToOriginV = -aiPos;
	aiVector3D displaceBackV = aiPos;
	aiMatrix4x4 displaceToOriginM = aiMatrix4x4(aiScale, aiRot, displaceToOriginV);
	aiRot = aiQuaternion(1.0f, 0.0f, 0.0f);
	aiScale = aiVector3D(1.0f, 1.0f, 1.0f);
	aiMatrix4x4 rotateAndDisplaceBackM = aiMatrix4x4(aiScale, aiRot, displaceBackV);
	aiMatrix4x4 outTransform = rotateAndDisplaceBackM * displaceToOriginM;
	return outTransform;
}

class AnimatedChain :public Mona::GameObject {

private:
	aiScene* m_chainScene;
	Mona::SkeletalMeshHandle m_skeletalMesh;
	Mona::TransformHandle m_transform;
	std::shared_ptr<Mona::AnimationClip> m_chainBaseAnimation;
	Mona::FABRIKSolver m_FABRIKSolver;
	//Mona::CCDSolver m_CCDSolver;
public:
	std::vector<aiMatrix4x4> m_worldTransforms = {};

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
		int numOfSegments = 1;
		float segmentLength = 5.0f;
		float animDuration = 3.0f;
		m_chainScene = animatedChainScene(numOfSegments, segmentLength, animDuration, &m_worldTransforms);
		// create IK chains
		Mona::SimpleIKChain bindPoseChain = Mona::SimpleIKChain(numOfSegments + 1, segmentLength);
		aiNode* rootNode = m_chainScene->mRootNode;
		aiNode* currNode = rootNode;
		int chainJointIndex = numOfSegments;
		while (chainJointIndex >= 0) {
			bindPoseChain.getChainNode(chainJointIndex)->setLocalTransform(currNode->mTransformation);
			chainJointIndex -= 1;
			currNode = currNode->mParent->mParent; //nos saltamos los links	
		}
		m_FABRIKSolver = Mona::FABRIKSolver(bindPoseChain, 20, 0.001);

		m_transform = world.AddComponent<Mona::TransformComponent>(*this);

		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& skeletonManager = Mona::SkeletonManager::GetInstance();
		auto& animationManager = Mona::AnimationClipManager::GetInstance();
		auto skeleton = skeletonManager.LoadSkeleton("chainSkeleton", m_chainScene);
		auto skinnedMesh = meshManager.LoadSkinnedMesh(skeleton, m_chainScene, "chainSkinnedMesh");
		m_chainBaseAnimation = animationManager.LoadAnimationClip("chainBaseAnimation", skeleton, m_chainScene);
		auto materialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat, true));
		materialPtr->SetDiffuseColor(glm::vec3(0.8f, 0.3f, 0.75f));
		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_chainBaseAnimation, materialPtr);
	}

};

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
		MONA_LOG_INFO("Starting App: ExampleIKChain");
		world.SetAmbientLight(glm::vec3(0.8f));
		CreatePlane(world);
		world.CreateGameObject<AnimatedChain>();
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 2.0f, glm::radians(-45.0f));
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 2.0f, glm::radians(-135.0f));
		std::vector<Mona::IKSolver> ikSolvers = {};

		// input/camera set up
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(m_windowResizeSubcription, this, &ExampleIKChain::OnWindowResize);
		eventManager.Subscribe(m_debugGUISubcription, this, &ExampleIKChain::OnDebugGUIEvent);
		m_camera = world.CreateGameObject<Mona::BasicPerspectiveCamera>();
		//world.AddComponent<Mona::SpotLightComponent>(m_camera, glm::vec3(100.0f), 15.0f, glm::radians(25.0f), glm::radians(37.0f));
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));
		world.GetInput().SetCursorType(Mona::Input::CursorType::Disabled);

	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
		MONA_LOG_INFO("ShuttingDown User App: ExampleIKChain");
		auto& eventManager = world.GetEventManager();
		eventManager.Unsubscribe(m_debugGUISubcription);
		eventManager.Unsubscribe(m_windowResizeSubcription);
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		auto& input = world.GetInput();
		auto& window = world.GetWindow();
		if (input.IsKeyPressed(MONA_KEY_G))
		{
			window.SetFullScreen(true);
		}
		else if (input.IsKeyPressed(MONA_KEY_H))
		{
			window.SetFullScreen(false);
		}
		else if (input.IsKeyPressed(MONA_KEY_J))
		{
			window.SetWindowDimensions(glm::ivec2(1000, 1000));
		}
		else if (input.IsKeyPressed(MONA_KEY_1)) {
			m_camera->SetActive(false);
			input.SetCursorType(Mona::Input::CursorType::Normal);
		}
		else if (input.IsKeyPressed(MONA_KEY_2)) {
			m_camera->SetActive(true);
			input.SetCursorType(Mona::Input::CursorType::Disabled);
		}
		else if (input.IsKeyPressed(MONA_KEY_ESCAPE)) {
			world.EndApplication();
		}
	}

	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Scene Options:");
		static bool selected[3] = { false, false, false };
		ImGui::End();
	}

	void OnWindowResize(const Mona::WindowResizeEvent& event)
	{
		MONA_LOG_INFO("A WindowResizeEvent has ocurred! {0} {1}", event.width, event.height);
	}

private:
	Mona::SubscriptionHandle m_windowResizeSubcription;
	Mona::SubscriptionHandle m_debugGUISubcription;
	Mona::GameObjectHandle<Mona::BasicPerspectiveCamera> m_camera;
	float somefloat = 0.0f;
	int m_currentMaterialIndex;



};

int main()
{
	ExampleIKChain exampleIKChain;
	Mona::Engine engine(exampleIKChain);
	engine.StartMainLoop();
}