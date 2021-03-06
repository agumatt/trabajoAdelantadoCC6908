#pragma once
#ifndef IKUTILS_HPP
#define IKUTILS_HPP

namespace Mona {
	class IKUtils {

	public:
		static aiVector3D getMatrixTranslation(aiMatrix4x4 mat) {
			aiVector3D pos;
			aiQuaternion rot;
			aiVector3D scl;
			mat.Decompose(scl, rot, pos);
			return pos;
		}

		static aiVector3D getMatrixScaling(aiMatrix4x4 mat) {
			aiVector3D pos;
			aiQuaternion rot;
			aiVector3D scl;
			mat.Decompose(scl, rot, pos);
			return scl;
		}

		static aiQuaternion getMatrixRotation(aiMatrix4x4 mat) {
			aiVector3D pos;
			aiQuaternion rot;
			aiVector3D scl;
			mat.Decompose(scl, rot, pos);
			return rot;
		}

		static aiMatrix4x4 createRotationMatrix(aiQuaternion rot) {
			aiVector3D pos = aiVector3D(0.0f, 0.0f, 0.0f);
			aiVector3D scl = aiVector3D(1.0f, 1.0f, 1.0f);
			return aiMatrix4x4(scl, rot, pos);
		}

		static aiMatrix4x4 createTranslationMatrix(aiVector3D trans) {
			aiQuaternion rot = aiQuaternion(0.0f, 0.0f, 0.0f);
			aiVector3D scl = aiVector3D(1.0f, 1.0f, 1.0f);
			return aiMatrix4x4(scl, rot, trans);
		}

		static aiMatrix4x4 createScalingMatrix(aiVector3D scl) {
			aiQuaternion rot = aiQuaternion(0.0f, 0.0f, 0.0f);
			aiVector3D pos = aiVector3D(0.0f, 0.0f, 0.0f);
			return aiMatrix4x4(scl, rot, pos);
		}

		static aiMatrix4x4 identityMatrix() {
			aiVector3D trans = aiVector3D(0.0f, 0.0f, 0.0f);
			aiQuaternion rot = aiQuaternion(0.0f, 0.0f, 0.0f);
			aiVector3D scal = aiVector3D(1.0f, 1.0f, 1.0f);
			return aiMatrix4x4(scal, rot, trans);
		}

	};

}




#endif