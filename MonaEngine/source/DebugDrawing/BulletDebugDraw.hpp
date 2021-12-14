#pragma once
#ifndef BULLETDEBUGDRAW_HPP
#define BULLETDEBUGDRAW_HPP
#include "LinearMath/btIDebugDraw.h"
#include <cstdint>
namespace Mona {
	class BulletDebugDraw : public btIDebugDraw {
	public:
		void StartUp() noexcept;
		virtual void setDebugMode(int debugMode) override;
		virtual int getDebugMode() const override;
		virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) override;
		virtual void reportErrorWarning(const char* warningString) override {}
		virtual void draw3dText(const btVector3& location, const char* textString) override {}
		virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;
		bool m_bDrawWireframe = false;
		bool m_bDrawAABB = false;
		bool m_bDrawContactsPoints = false;
	private:
		uint32_t VAO;
	};
}
#endif