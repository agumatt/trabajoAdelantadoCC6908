#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"

void CreateBasicCameraWithMusicAndLight(Mona::World& world)
{
	auto camera = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(camera, glm::vec3(0.0f, -15.0f, 15.0f));
	transform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.5f);
	world.SetMainCamera(world.AddComponent<Mona::CameraComponent>(camera));
	world.SetAudioListenerTransform(transform);
	auto& audioClipManager = Mona::AudioClipManager::GetInstance();
	auto audioClipPtr = audioClipManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/music.wav"));
	auto audioSource = world.AddComponent<Mona::AudioSourceComponent>(camera, audioClipPtr);
	audioSource->SetIsLooping(true);
	audioSource->SetVolume(0.3f);
	audioSource->Play();

	world.AddComponent<Mona::DirectionalLightComponent>(camera, glm::vec3(1.0f));

}

void CreateWall(Mona::World& world,
	const glm::vec3& position,
	const glm::vec3& scale,
	std::shared_ptr<Mona::Material> wallMaterial) {
	auto& meshManager = Mona::MeshManager::GetInstance();
	auto wall = world.CreateGameObject<Mona::GameObject>();
	world.AddComponent<Mona::TransformComponent>(wall, position, glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), scale);
	world.AddComponent<Mona::StaticMeshComponent>(wall, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), wallMaterial);
	Mona::BoxShapeInformation wallShape(scale);
	Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(wall, wallShape, Mona::RigidBodyType::StaticBody);
	rb->SetRestitution(1.0f);
	rb->SetFriction(0.0f);
}


class animatedChain :public Mona::GameObject {

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
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		// creamos la escena que contiene la cadena
		
		// create skeleton
		
		// create mesh
		// create base animation
		//m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_idleAnimation, materialPtr);
	}

};
void CreateAnimatedChain(Mona::World& world, int numOfSegments) {
	auto chain = world.CreateGameObject<Mona::GameObject>();
	for (int i; i < numOfSegments; i++) {


	}


}

class Character : public Mona::GameObject
{
private:
	void SetTargetPosition(const glm::vec3 position) {
		m_targetPosition = position;
		const glm::vec3& currentPos = m_transform->GetLocalTranslation();
		glm::vec3 toTarget = glm::vec3(m_targetPosition.x - currentPos.x,
			m_targetPosition.y - currentPos.y,
			0.0f);
		m_targetFrontVector = glm::normalize(toTarget);
	}

	void UpdateTargetPosition(Mona::World& world) {
		auto& input = world.GetInput();
		if (input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1) && !m_prevIsPress) {
			auto mousePos = input.GetMousePosition();
			MONA_LOG_INFO("Mouse Button press At: ({0},{1}).", mousePos.x, mousePos.y);
			auto camera = world.GetMainCameraComponent();
			auto cameraTransform = world.GetSiblingComponentHandle<Mona::TransformComponent>(camera);
			glm::vec3 rayFrom = cameraTransform->GetLocalTranslation();
			glm::vec3 rayTo = world.MainCameraScreenPositionToWorld(mousePos);
			glm::vec3 direction = glm::normalize(rayTo - rayFrom);
			auto raycastResult = world.ClosestHitRayTest(rayFrom, rayFrom + camera->GetZFarPlane() * direction);
			if (raycastResult.HasHit())
			{
				SetTargetPosition(raycastResult.m_hitPosition);
			}
			m_prevIsPress = true;
		}
		else if (m_prevIsPress && !input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1)) {
			m_prevIsPress = false;
		}

	}


	void UpdateSteeringBehaviour() {
		const glm::vec3& currentPos = m_transform->GetLocalTranslation();
		glm::vec3 toTarget = glm::vec3(m_targetPosition.x - currentPos.x,
			m_targetPosition.y - currentPos.y,
			0.0f);
		float distance = glm::length(toTarget);



		float speed = distance / m_deacelerationFactor;
		speed = std::min(speed, m_maxSpeed);
		glm::vec3 desiredVelocity = distance >= m_distanceThreshold ? speed * toTarget / distance : glm::vec3(0.0f);
		glm::vec3 rbVelocity = m_rigidBody->GetLinearVelocity();
		glm::vec3 force = glm::vec3(desiredVelocity.x - rbVelocity.x,
			desiredVelocity.y - rbVelocity.y, 0.0f);
		m_rigidBody->SetLinearVelocity(desiredVelocity);


		glm::vec3 currentFrontVector = m_transform->GetUpVector();
		float cosAngle = glm::dot(currentFrontVector, m_targetFrontVector);
		float angle = glm::acos(cosAngle);
		float sign = glm::cross(currentFrontVector, m_targetFrontVector).z > 0.0f ? 1.0f : -1.0f;
		if (angle != 0.0f) {
			m_rigidBody->SetAngularVelocity(glm::vec3(0.0f, 0.0f, m_angularVelocityFactor * sign * angle));
		}



	}

	void UpdateAnimationState() {
		auto& animController = m_skeletalMesh->GetAnimationController();
		glm::vec3 currentVelocity = m_rigidBody->GetLinearVelocity();
		currentVelocity.z = 0.0f;
		float speed = glm::length(currentVelocity);

		if (speed > m_maxWalkingSpeed) {
			Mona::BlendType blendType = m_walkingAnimation == animController.GetCurrentAnimation() ? Mona::BlendType::KeepSynchronize : Mona::BlendType::Smooth;
			animController.FadeTo(m_runningAnimation, blendType, m_fadeTime, 0.0f);
		}
		else if (speed > m_maxIdleSpeed) {
			Mona::BlendType blendType = m_runningAnimation == animController.GetCurrentAnimation() ? Mona::BlendType::KeepSynchronize : Mona::BlendType::Smooth;
			animController.FadeTo(m_walkingAnimation, blendType, m_fadeTime, 0.0f);
		}
		else {
			animController.FadeTo(m_idleAnimation, Mona::BlendType::Smooth, m_fadeTime, 0.0f);
		}

	}
public:
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		UpdateTargetPosition(world);
		UpdateSteeringBehaviour();
		UpdateAnimationState();
	};
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(m_debugGUISubcription, this, &Character::OnDebugGUIEvent);

		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->Rotate(glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(90.f));
		m_transform->SetScale(glm::vec3(0.01f));
		m_transform->Translate(glm::vec3(0.0f, 0.0f, 01.1f));
		m_targetPosition = glm::vec3(0.0f);
		glm::fquat offsetRotation = glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(180.f), glm::vec3(0.0f, 0.0f, 1.0f)) * glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(-90.f), glm::vec3(1.0f, 0.0f, 0.0f));

		world.SetAudioListenerTransform(m_transform, offsetRotation);

		auto materialPtr = std::static_pointer_cast<Mona::DiffuseTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseTextured, true));
		auto& textureManager = Mona::TextureManager::GetInstance();
		auto diffuseTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/akai/akai_diffuse.png"));
		materialPtr->SetMaterialTint(glm::vec3(0.1f));
		materialPtr->SetDiffuseTexture(diffuseTexture);

		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& skeletonManager = Mona::SkeletonManager::GetInstance();
		auto& animationManager = Mona::AnimationClipManager::GetInstance();
		auto skeleton = skeletonManager.LoadSkeleton(Mona::SourcePath("Assets/Models/akai_e_espiritu.fbx"));
		auto skinnedMesh = meshManager.LoadSkinnedMesh(skeleton, Mona::SourcePath("Assets/Models/akai_e_espiritu.fbx"), true);
		m_runningAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/running.fbx"), skeleton);
		m_walkingAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/walking.fbx"), skeleton);
		m_idleAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/idle.fbx"), skeleton);
		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_idleAnimation, materialPtr);
		Mona::BoxShapeInformation boxInfo(glm::vec3(.6f, 1.0f, 0.6f));
		m_rigidBody = world.AddComponent<Mona::RigidBodyComponent>(*this, boxInfo, Mona::RigidBodyType::DynamicBody, 1.0f, false, glm::vec3(0.0f, -0.1f, 0.0f));
		m_rigidBody->ClearForces();

	}

	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Character Options:");
		ImGui::SliderFloat("DeacelerationFactor", &(m_deacelerationFactor), 0.0f, 10.0f);
		ImGui::SliderFloat("DistanceThreshold", &(m_distanceThreshold), 0.0f, 10.0f);
		ImGui::SliderFloat("AngularVelocityFactor", &(m_angularVelocityFactor), 0.0f, 10.0f);
		ImGui::SliderFloat("MaxSpeed", &(m_maxSpeed), 0.0f, 10.0f);
		ImGui::SliderFloat("MaxIdleSpeed", &(m_maxIdleSpeed), 0.0f, 10.0f);
		ImGui::SliderFloat("MaxWalkingSpeed", &(m_maxWalkingSpeed), 0.0f, 10.0f);
		ImGui::SliderFloat("FadeTime", &(m_fadeTime), 0.0f, 1.0f);
		ImGui::End();
	}
private:
	float m_deacelerationFactor = 1.1f;
	float m_angularVelocityFactor = 3.0f;
	float m_distanceThreshold = 0.65f;
	float m_maxSpeed = 5.0f;
	float m_maxIdleSpeed = 0.2f;
	float m_maxWalkingSpeed = 3.4f;
	float m_fadeTime = 0.5f;
	bool m_prevIsPress = false;
	glm::vec3 m_targetPosition = glm::vec3(0.0f);
	glm::vec3 m_targetFrontVector = glm::vec3(0.0f, -1.0f, 0.0f);
	Mona::TransformHandle m_transform;
	Mona::RigidBodyHandle m_rigidBody;
	Mona::SkeletalMeshHandle m_skeletalMesh;
	std::shared_ptr<Mona::AnimationClip> m_runningAnimation;
	std::shared_ptr<Mona::AnimationClip> m_walkingAnimation;
	std::shared_ptr<Mona::AnimationClip> m_idleAnimation;
	Mona::SubscriptionHandle m_debugGUISubcription;

};

class Paddle : public Mona::GameObject {
public:
	Paddle(float velocity) : m_paddleVelocity(velocity) {}
	~Paddle() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		glm::vec3 paddleScale(2.0f, 0.5f, 0.5f);
		m_transform->Scale(paddleScale);
		auto paddleMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		paddleMaterial->SetDiffuseColor(glm::vec3(0.3f, 0.3f, 0.75f));
		auto& meshManager = Mona::MeshManager::GetInstance();
		world.AddComponent<Mona::StaticMeshComponent>(*this, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), paddleMaterial);
		Mona::BoxShapeInformation boxInfo(paddleScale);
		Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(*this, boxInfo, Mona::RigidBodyType::KinematicBody);
		rb->SetFriction(0.0f);
		rb->SetRestitution(1.0f);
		
		auto& audioClipManager = Mona::AudioClipManager::GetInstance();
		m_ballBounceSound = audioClipManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/ballBounce.wav"));

		auto ball = world.CreateGameObject<Mona::GameObject>();
		float ballRadius = 0.5f;
		m_ballTransform = world.AddComponent<Mona::TransformComponent>(ball);
		m_ballTransform->SetRotation(m_transform->GetLocalRotation());
		m_ballTransform->SetTranslation(m_transform->GetLocalTranslation() + glm::vec3(0.0f, 2.0f, 0.0f));
		m_ballTransform->SetScale(glm::vec3(ballRadius));
		auto ballMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		ballMaterial->SetDiffuseColor(glm::vec3(0.75f, 0.3f, 0.3f));
		world.AddComponent<Mona::StaticMeshComponent>(ball, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Sphere), ballMaterial);
		
		Mona::SphereShapeInformation sphereInfo(ballRadius);
		m_ballRigidBody = world.AddComponent<Mona::RigidBodyComponent>(ball, sphereInfo, Mona::RigidBodyType::DynamicBody);
		m_ballRigidBody->SetRestitution(1.0f);
		m_ballRigidBody->SetFriction(0.0f);
		auto callback = [ballTransform = m_ballTransform, ballSound = m_ballBounceSound](Mona::World& world, Mona::RigidBodyHandle& otherRigidBody, bool isSwaped, Mona::CollisionInformation& colInfo) mutable {
			world.PlayAudioClip3D(ballSound, ballTransform->GetLocalTranslation(),0.3f);
		};
		m_ballRigidBody->SetStartCollisionCallback(callback);
	}

	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		auto& input = world.GetInput();
		if (input.IsKeyPressed(MONA_KEY_A))
		{
			m_transform->Translate(glm::vec3(-m_paddleVelocity * timeStep, 0.0f, 0.0f));
		}
		else if (input.IsKeyPressed(MONA_KEY_D))
		{
			m_transform->Translate(glm::vec3(m_paddleVelocity * timeStep, 0.0f, 0.0f));
		}

		if (input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1)) {
			m_ballRigidBody->SetLinearVelocity(glm::vec3(0.0f,15.0f,0.0f));
			
		}


	}

private:
	Mona::TransformHandle m_transform;
	Mona::TransformHandle m_ballTransform;
	Mona::RigidBodyHandle m_ballRigidBody;
	std::shared_ptr<Mona::AudioClip> m_ballBounceSound;
	float m_paddleVelocity;
};



class Breakout : public Mona::Application {
public:
	Breakout() = default;
	~Breakout() = default;
	virtual void UserStartUp(Mona::World & world) noexcept override {
		world.SetGravity(glm::vec3(0.0f,0.0f,0.0f));
		world.SetAmbientLight(glm::vec3(0.3f));
		CreateBasicCameraWithMusicAndLight(world);
		world.CreateGameObject<Paddle>(20.0f);
		

		//Crear el los bloques destructibles del nivel
		glm::vec3 blockScale(1.0f, 0.5f, 0.5f);
		auto& audioClipManager = Mona::AudioClipManager::GetInstance();
		m_blockBreakingSound = audioClipManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/boxBreaking.wav"));
		auto& meshManager = Mona::MeshManager::GetInstance();
		Mona::BoxShapeInformation boxInfo(blockScale);
		auto blockMaterial = world.CreateMaterial(Mona::MaterialType::DiffuseFlat);
		for (int i = -2; i < 3; i++) {
			float x = 4.0f * i;
			for (int j = -2; j < 3; j++)
			{
				float y = 2.0f * j;
				auto block = world.CreateGameObject<Mona::GameObject>();
				auto transform = world.AddComponent<Mona::TransformComponent>(block, glm::vec3( x, 15.0f + y, 0.0f));
				transform->Scale(blockScale);
				world.AddComponent<Mona::StaticMeshComponent>(block, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), blockMaterial);
				Mona::RigidBodyHandle rb =world.AddComponent<Mona::RigidBodyComponent>(block, boxInfo, Mona::RigidBodyType::StaticBody, 1.0f);
				rb->SetRestitution(1.0f);
				rb->SetFriction(0.0f);
				auto callback = [block, blockSound = m_blockBreakingSound](Mona::World& world, Mona::RigidBodyHandle& otherRigidBody, bool isSwaped, Mona::CollisionInformation& colInfo) mutable {
					world.PlayAudioClip2D(blockSound, 1.0f, 1.0f);
					world.DestroyGameObject(block);
				};

				rb->SetStartCollisionCallback(callback);
				
			}
		}

		//Crear las paredes
		auto wallMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		wallMaterial->SetDiffuseColor(glm::vec3(0.15f, 0.15f, 0.15f));


		CreateWall(world, glm::vec3(0.0f, 26.0f, 0.0f), glm::vec3(18.0f, 1.0f, 1.0f), wallMaterial);

		glm::vec3 sideWallScale(1.0f, 27.0f, 1.0f);
		float sideWallOffset = 19.0f;
		CreateWall(world, glm::vec3(-sideWallOffset, 0.0f, 0.0f), sideWallScale, wallMaterial);
		CreateWall(world, glm::vec3(sideWallOffset, 0.0f, 0.0f), sideWallScale, wallMaterial);

	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World & world, float timeStep) noexcept override {
	}
	std::shared_ptr<Mona::AudioClip> m_blockBreakingSound;
};
int main()
{
	Breakout breakout;
	Mona::Engine engine(breakout);
	engine.StartMainLoop();
}