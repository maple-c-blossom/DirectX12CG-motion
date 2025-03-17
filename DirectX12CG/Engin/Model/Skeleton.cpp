#include "Skeleton.h"

using namespace MCB;
using namespace std;
using namespace MCBRef;

void MCBRef::Skeleton::SetBone(SetBoneData boneData)
{
	unique_ptr<Bone> addBone = make_unique<Bone>();
	addBone->SetName(boneData.name);
	addBone->SetTranslate(boneData.translation_);
	addBone->SetRotation(boneData.rotation_);
	addBone->SetScale(boneData.scale_);
	addBone->SetInverseBindMatrix(boneData.inverseBindMatrix_);
	addBone->CalculateLocalMatrix();

	Bone* parentBonePtr = FindBone(boneData.parentBoneName_);
	if (parentBonePtr)
	{
		addBone->SetParent(parentBonePtr);
		parentBonePtr->AddChild(addBone.get());
	}
	else
	{
		rootBone_ = addBone.get();
	}

	bones_[boneData.name] = move(addBone);

	
}

Bone* MCBRef::Skeleton::FindBone(std::string findBoneName)
{

	unordered_map<std::string, std::unique_ptr<Bone>>::iterator findedBoneItr = bones_.find(findBoneName);

	if (findedBoneItr != bones_.end())
	{
		return findedBoneItr->second.get();
	}

	return nullptr;
}
