#pragma once
#ifndef SKELETONMANAGER_HPP
#define SKELETONMANAGER_HPP
#include <memory>
#include <string>
#include <filesystem>
#include <unordered_map>
#include <assimp/scene.h>
namespace Mona {
	class Skeleton;
	class SkeletonManager {
		friend class World;
	public:
		using SkeletonMap = std::unordered_map<std::string, std::shared_ptr<Skeleton>>;
		SkeletonManager(SkeletonManager const&) = delete;
		SkeletonManager& operator=(SkeletonManager const&) = delete;
		std::shared_ptr<Skeleton> LoadSkeleton(const std::filesystem::path& filePath) noexcept;
		std::shared_ptr<Skeleton> LoadSkeleton(const std::string& name, aiScene* scene) noexcept;
		void CleanUnusedSkeletons() noexcept;
		static SkeletonManager& GetInstance() noexcept {
			static SkeletonManager manager;
			return manager;
		}
	private:
		SkeletonManager() = default;
		void ShutDown() noexcept;
		SkeletonMap m_skeletonMap;
	};
}
#endif