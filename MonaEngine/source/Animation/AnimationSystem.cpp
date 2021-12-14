#include "AnimationSystem.hpp"
#include "../World/ComponentManager.hpp"
namespace Mona {
	void AnimationSystem::UpdateAllPoses(ComponentManager<SkeletalMeshComponent>& skeletalMeshDataManager, float timeStep) noexcept {
		//Se itera sobre todas las componentes de animaci�n, los animation controller son los responsables de la logica de
		//actualizaci�n.
		for (uint32_t i = 0; i < skeletalMeshDataManager.GetCount(); i++) {
			SkeletalMeshComponent& skeletalMesh = skeletalMeshDataManager[i];
			auto& animationController = skeletalMesh.GetAnimationController();
			animationController.UpdateCurrentPose(timeStep);
		}
	}
}