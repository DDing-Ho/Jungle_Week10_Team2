// FBX SDK based importer entry points.
#pragma once

#include "Core/CoreTypes.h"

struct FSkeletalMesh;

struct FFbxImporter
{
    static bool ImportSkeletalMesh(const FString& FbxFilePath, FSkeletalMesh& OutMesh);
};

