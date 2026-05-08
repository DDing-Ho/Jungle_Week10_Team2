#pragma once

#include "Component/MeshComponent.h"
#include "Core/PropertyTypes.h"
#include "Mesh/SkeletalMesh.h"

class UMaterial;
class FPrimitiveProxy;
class FDynamicMeshBuffer;

namespace json
{
    class JSON;
}

class USkeletalMeshComponent : public UMeshComponent
{
public:
    DECLARE_CLASS(USkeletalMeshComponent, UMeshComponent)

    USkeletalMeshComponent() = default;
    ~USkeletalMeshComponent() override = default;

    FDynamicMeshBuffer* GetDynamicMeshBuffer() const override;
    FMeshDataView GetMeshDataView() const override;
    bool LineTraceComponent(const FRay& Ray, FHitResult& OutHitResult) override;
    void UpdateWorldAABB() const override;

    // FPrimitiveProxy* CreateSceneProxy() override;

    void SetSkeletalMesh(USkeletalMesh* InMesh);
    USkeletalMesh* GetSkeletalMesh() const;

    void SetMaterial(int32 ElementIndex, UMaterial* InMaterial) override;
    UMaterial* GetMaterial(int32 ElementIndex) const override;
    int32 GetNumMaterials() const override { return static_cast<int32>(OverrideMaterials.size()); }
    const TArray<UMaterial*>& GetOverrideMaterials() const { return OverrideMaterials; }

    void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction& ThisTickFunction) override;

    const TArray<FMatrix>& GetComponentSpaceTransforms() const { return ComponentSpaceTransforms; }

    void Serialize(FArchive& Ar) override;
    void PostDuplicate() override;

    void GetEditableProperties(TArray<FPropertyDescriptor>& OutProps) override;
    void PostEditProperty(const char* PropertyName) override;

    const FString& GetSkeletalMeshPath() const { return SkeletalMeshPath; }

private:
    void CacheLocalBounds();
    void RefreshComponentSpaceTransforms();

    USkeletalMesh* SkeletalMesh = nullptr;
    FString SkeletalMeshPath = "None";
    TArray<UMaterial*> OverrideMaterials;
    TArray<FMaterialSlot> MaterialSlots;

    TArray<FMatrix> ComponentSpaceTransforms;

    FVector CachedLocalCenter = { 0, 0, 0 };
    FVector CachedLocalExtent = { 0.5f, 0.5f, 0.5f };
    bool bHasValidBounds = false;
};
