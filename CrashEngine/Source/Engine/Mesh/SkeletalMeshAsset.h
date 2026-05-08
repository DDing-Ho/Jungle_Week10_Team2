// Skeletal mesh CPU asset data parsed from FBX.
#pragma once

#include "Core/CoreTypes.h"
#include "Math/Matrix.h"
#include "Mesh/StaticMeshAsset.h"
#include "Render/RHI/D3D11/Buffers/VertexTypes.h"
#include "Serialization/Archive.h"

#include <algorithm>

static constexpr int32 MAX_BONE_INFLUENCES = 4;

inline FArchive& SerializeMatrix(FArchive& Ar, FMatrix& Matrix)
{
    Ar.Serialize(Matrix.Data, sizeof(Matrix.Data));
    return Ar;
}

struct FSkinWeight
{
    int32 BoneIndices[MAX_BONE_INFLUENCES] = { -1, -1, -1, -1 };
    float BoneWeights[MAX_BONE_INFLUENCES] = { 0.0f, 0.0f, 0.0f, 0.0f };

    void Normalize()
    {
        float Sum = 0.0f;
        for (float Weight : BoneWeights)
        {
            Sum += Weight;
        }

        if (Sum <= 0.00001f)
        {
            BoneIndices[0] = 0;
            BoneWeights[0] = 1.0f;
            for (int32 i = 1; i < MAX_BONE_INFLUENCES; ++i)
            {
                BoneIndices[i] = -1;
                BoneWeights[i] = 0.0f;
            }
            return;
        }

        const float InvSum = 1.0f / Sum;
        for (float& Weight : BoneWeights)
        {
            Weight *= InvSum;
        }
    }
};

struct FSkeletonBone
{
    FString Name;
    int32 ParentIndex = -1;

    FMatrix LocalBindPose = FMatrix::Identity;
    FMatrix GlobalBindPose = FMatrix::Identity;
    FMatrix InverseBindPose = FMatrix::Identity;
    FMatrix CurrentGlobalPose = FMatrix::Identity;

    friend FArchive& operator<<(FArchive& Ar, FSkeletonBone& Bone)
    {
        Ar << Bone.Name;
        Ar << Bone.ParentIndex;
        SerializeMatrix(Ar, Bone.LocalBindPose);
        SerializeMatrix(Ar, Bone.GlobalBindPose);
        SerializeMatrix(Ar, Bone.InverseBindPose);
        SerializeMatrix(Ar, Bone.CurrentGlobalPose);
        return Ar;
    }
};

struct FSkeletalMesh
{
    FString PathFileName;
    TArray<FVertexPNCT_T> RefVertices;
    TArray<FVertexPNCT_T> SkinnedVertices;
    TArray<FSkinWeight> SkinWeights;
    TArray<uint32> Indices;
    TArray<FStaticMeshSection> Sections;
    TArray<FSkeletonBone> Bones;
    TArray<FStaticMaterial> Materials;

    FVector BoundsCenter = FVector(0.0f, 0.0f, 0.0f);
    FVector BoundsExtent = FVector(0.0f, 0.0f, 0.0f);
    bool bBoundsValid = false;

    void CacheBounds()
    {
        bBoundsValid = false;
        if (RefVertices.empty())
        {
            return;
        }

        FVector LocalMin = RefVertices[0].Position;
        FVector LocalMax = RefVertices[0].Position;
        for (const FVertexPNCT_T& Vertex : RefVertices)
        {
            LocalMin.X = (std::min)(LocalMin.X, Vertex.Position.X);
            LocalMin.Y = (std::min)(LocalMin.Y, Vertex.Position.Y);
            LocalMin.Z = (std::min)(LocalMin.Z, Vertex.Position.Z);
            LocalMax.X = (std::max)(LocalMax.X, Vertex.Position.X);
            LocalMax.Y = (std::max)(LocalMax.Y, Vertex.Position.Y);
            LocalMax.Z = (std::max)(LocalMax.Z, Vertex.Position.Z);
        }

        BoundsCenter = (LocalMin + LocalMax) * 0.5f;
        BoundsExtent = (LocalMax - LocalMin) * 0.5f;
        bBoundsValid = true;
    }

    void Serialize(FArchive& Ar)
    {
        Ar << PathFileName;
        Ar << RefVertices;
        Ar << SkinnedVertices;
        Ar << SkinWeights;
        Ar << Indices;
        Ar << Sections;
        Ar << Bones;
        Ar << Materials;
    }
};
