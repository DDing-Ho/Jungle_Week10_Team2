#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

#include "Math/Vector.h"
#include "Math/Matrix.h"
#include "Math/Transform.h"
#include "Object/Object.h"
#include "Render/RHI/D3D11/Buffers/VertexTypes.h"

#include "Engine/Serialization/Archive.h"
#include "Render/RHI/D3D11/Buffers/DynamicMeshBuffer.h"

struct FSkeletalBone
{
    FString Name;

    int32 ParentIndex = -1;

    FTransform RefLocalTransform;
    FMatrix RefGlobalMatrix;
    FMatrix InverseBindPose;

    friend FArchive& operator<<(FArchive& Ar, FSkeletalBone& Bone)
    {
        Ar << Bone.Name;
        Ar << Bone.ParentIndex;
        Ar << Bone.RefLocalTransform.Location;
        Ar << Bone.RefLocalTransform.Rotation;
        Ar << Bone.RefLocalTransform.Scale;
        return Ar;
    }
};

struct FSkeletalMeshSection
{
    int32 MaterialIndex = -1;
    FString MaterialSlotName;
    uint32 FirstIndex;
    uint32 NumTriangles;

    friend FArchive& operator<<(FArchive& Ar, FSkeletalMeshSection& Section)
    {
        Ar << Section.MaterialSlotName << Section.FirstIndex << Section.NumTriangles;
        return Ar;
    }
};

struct FSkeletalMesh
{
    FString PathFileName;

    TArray<FSkeletalBone> Bones;

    TArray<FSkeletalMeshVertex> Vertices;
    TArray<uint32> Indices;
    TArray<FSkeletalMeshSection> Sections;

    std::unique_ptr<FDynamicMeshBuffer> RenderBuffer;

    FVector BoundsCenter = FVector(0, 0, 0);
    FVector BoundsExtent = FVector(0, 0, 0);
    bool bBoundsValid = false;

    void CacheBounds()
    {
        bBoundsValid = false;
        if (Vertices.empty())
            return;

        FVector LocalMin = Vertices[0].Position;
        FVector LocalMax = Vertices[0].Position;
        for (const FSkeletalMeshVertex& V : Vertices)
        {
            LocalMin.X = (std::min)(LocalMin.X, V.Position.X);
            LocalMin.Y = (std::min)(LocalMin.Y, V.Position.Y);
            LocalMin.Z = (std::min)(LocalMin.Z, V.Position.Z);
            LocalMax.X = (std::max)(LocalMax.X, V.Position.X);
            LocalMax.Y = (std::max)(LocalMax.Y, V.Position.Y);
            LocalMax.Z = (std::max)(LocalMax.Z, V.Position.Z);
        }

        BoundsCenter = (LocalMin + LocalMax) * 0.5f;
        BoundsExtent = (LocalMax - LocalMin) * 0.5f;
        bBoundsValid = true;
    }

    void Serialize(FArchive& Ar)
    {
        Ar << PathFileName;
        Ar << Bones;
        Ar << Vertices;
        Ar << Indices;
        Ar << Sections;

        // 나중에 추가할 경우:
        // Ar << LODs; 
    }
};