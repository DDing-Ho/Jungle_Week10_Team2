#pragma once

#include <vector>

#include "SkeletalMeshAsset.h"
#include "Object/Object.h"
#include "StaticMeshAsset.h"
#include "Collision/BVH/MeshTriangleBVH.h"
#include "Render/RHI/D3D11/Buffers/VertexTypes.h"

using FMeshMaterial = FStaticMaterial;

class USkeletalMesh : public UObject
{
public:
    DECLARE_CLASS(USkeletalMesh, UObject)

    static constexpr uint32 MAX_LOD_COUNT = 4;

public:
    USkeletalMesh() = default;
    ~USkeletalMesh() override;

    void Serialize(FArchive& Ar);

    const FString& GetAssetPathFileName() const;

    void SetSkeletalMeshAsset(FSkeletalMesh* InMesh);
    FSkeletalMesh* GetSkeletalMeshAsset() const;

    void SetMaterials(TArray<FMeshMaterial>&& InMaterials);
    const TArray<FMeshMaterial>& GetMaterials() const;

    void InitResources(ID3D11Device* InDevice);

    void EnsureMeshTrianglePickingBVHBuilt() const;
    bool RaycastMeshTrianglesWithBVHLocal(const FVector& LocalOrigin, const FVector& LocalDirection, FHitResult& OutHitResult) const;

    uint32 GetLODCount() const { return 1; }

private:
    FSkeletalMesh* SkeletalMeshAsset = nullptr;
    TArray<FMeshMaterial> Materials;
    mutable FMeshTriangleBVH MeshTrianglePickingBVH;
};